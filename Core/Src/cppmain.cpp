extern "C" {

#include <main.h>

extern ADC_HandleTypeDef hadc1;
extern CAN_HandleTypeDef hcan;
}

#include <span>

#include <Stuff/IO/ACS.hpp>
#include <Stuff/IO/NTC.hpp>

static std::array<uint32_t, 10> raw_adc_results {};
static std::array<Stf::IO::ACS::ACS, 4> current_sensors;

static std::array<uint8_t, 8> s_tx_data {};
static CAN_TxHeaderTypeDef s_tx_header {
    .IDE = CAN_ID_STD,
};
static uint32_t s_tx_mailbox;

// extern "C" void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* handle) {}

/***
 * nmos ones:\n
 * RELAY_AC_IN\n
 * RELAY_SMPS_BAT\n
 * RELAY_ENGINE_96V\n
 * set false
 */
void set_xmos(GPIO_TypeDef *port, uint16_t pin, bool is_pmos, bool wanted) {
    /*
     * wanted pmos res
     * 0      0    0
     * 0      1    1
     * 1      0    1
     * 1      1    0
     */

    bool state = is_pmos != wanted;
    HAL_GPIO_WritePin(port, pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

extern "C" void cpp_init() {
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

    // p channel
    HAL_GPIO_WritePin(SIGN_BMS_GPIO_Port, SIGN_BMS_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RELAY_ENGINE_12V_GPIO_Port, RELAY_ENGINE_12V_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RELAY_SMPS_12V_GPIO_Port, RELAY_SMPS_12V_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SIGN_Telemetry_GPIO_Port, SIGN_Telemetry_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RELAY_MAIN_GPIO_Port, RELAY_MAIN_Pin, GPIO_PIN_RESET);

    // n channel
    HAL_GPIO_WritePin(RELAY_AC_IN_GPIO_Port, RELAY_AC_IN_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RELAY_SMPS_BAT_GPIO_Port, RELAY_SMPS_BAT_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RELAY_ENGINE_96V_GPIO_Port, RELAY_ENGINE_96V_Pin, GPIO_PIN_SET);

    Stf::IO::ACS::ACS sensor {
        .type = Stf::IO::ACS::Type::B150,
        .zero_point = 3.3f / 2.f,
        .voltage_ceil = 3.3f,
        .voltage_division = 4096.f,
    };

    std::fill(current_sensors.begin(), current_sensors.end(), sensor);

    if (HAL_ADC_Start_DMA(&hadc1, raw_adc_results.data(), raw_adc_results.size()) != HAL_OK)
        Error_Handler();

    CAN_FilterTypeDef filter = {
        .FilterIdHigh = 0,
        .FilterIdLow = 0,
        .FilterMaskIdHigh = 0,
        .FilterMaskIdLow = 0,
        .FilterFIFOAssignment = CAN_RX_FIFO0,
        .FilterBank = 0,
        .FilterMode = CAN_FILTERMODE_IDMASK,
        .FilterScale = CAN_FILTERSCALE_32BIT,
        .FilterActivation = ENABLE,
        .SlaveStartFilterBank = 14,
    };

    if (HAL_CAN_ConfigFilter(&hcan, &filter) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_CAN_Start(&hcan) != HAL_OK) {
        Error_Handler();
    }
}

static std::array<float, 2> get_temperatures(std::span<const float, 2> voltages) {
    const auto ntc_nominal = Stf::NTC::Nominal::R10K;
    const auto ntc_resistor = 10000;

    std::array<float, 2> ret;

    for (size_t i = 0; i < 2; i++) {
        auto [r_0, r_1] = Stf::NTC::solve_voltage_divider(ntc_resistor, 3.3f, voltages[i]);
        ret[i] = Stf::NTC::calculate_ntc(ntc_nominal, r_0) - 273.15;
    }

    return ret;
};

extern "C" void cpp_loop() {
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

    // HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, status);

    static bool prev_ac_status = false;
    const bool ac_status = HAL_GPIO_ReadPin(IN_AC_DETECT_GPIO_Port, IN_AC_DETECT_Pin) == GPIO_PIN_SET;
    if (ac_status != prev_ac_status) {
        prev_ac_status = ac_status;
        if (ac_status) {
            set_xmos(RELAY_ENGINE_12V_GPIO_Port, RELAY_ENGINE_12V_Pin, true, false);
            set_xmos(RELAY_ENGINE_96V_GPIO_Port, RELAY_ENGINE_96V_Pin, false, false);
            HAL_Delay(500);
            set_xmos(RELAY_SMPS_BAT_GPIO_Port, RELAY_SMPS_BAT_Pin, false, true);
            HAL_Delay(10);
            set_xmos(RELAY_AC_IN_GPIO_Port, RELAY_AC_IN_Pin, false, true);
            HAL_Delay(1000);
            set_xmos(RELAY_SMPS_12V_GPIO_Port, RELAY_SMPS_12V_Pin, true, true);
        } else {
            set_xmos(RELAY_SMPS_12V_GPIO_Port, RELAY_SMPS_12V_Pin, true, false);
            HAL_Delay(1000);
            set_xmos(RELAY_AC_IN_GPIO_Port, RELAY_AC_IN_Pin, false, false);
            HAL_Delay(10);
            set_xmos(RELAY_SMPS_BAT_GPIO_Port, RELAY_SMPS_BAT_Pin, false, false);
            HAL_Delay(500);
            set_xmos(RELAY_ENGINE_96V_GPIO_Port, RELAY_ENGINE_96V_Pin, false, true);
            set_xmos(RELAY_ENGINE_12V_GPIO_Port, RELAY_ENGINE_12V_Pin, true, true);
        }
    }

    std::array<float, 10> voltage_results;
    for (auto it = voltage_results.begin(); const auto res : raw_adc_results)
        *it = res * (3.3f / 4095.f);

    for (size_t i = 0; i < 4; i++)
        current_sensors[i].bump(voltage_results[i * 2 + 3]);

    std::array<std::array<float, 2>, 5> data_to_send;
    data_to_send[0] = get_temperatures(std::span<const float, 2> { voltage_results.data(), 2 });
    for (size_t i = 0; i < 4; i++)
        data_to_send[i] = { voltage_results[2 + i * 2], current_sensors[i].measure_amperes(true) };

    for (uint32_t i = 0; i < 5; i++) {
        s_tx_header.StdId = 0x21 + i;
        s_tx_header.DLC = 8;

        const auto f_0 = std::bit_cast<std::array<uint8_t, 4>>(data_to_send[i][0]);
        const auto f_1 = std::bit_cast<std::array<uint8_t, 4>>(data_to_send[i][0]);

        std::copy(f_0.cbegin(), f_0.cend(), s_tx_data.begin());
        std::copy(f_1.cbegin(), f_1.cend(), s_tx_data.begin() + 4);

        // while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan) != 0)
        HAL_Delay(50);
        HAL_CAN_AddTxMessage(&hcan, &s_tx_header, s_tx_data.data(), &s_tx_mailbox);
    }

    HAL_Delay(50);
    s_tx_header.StdId = 0x26;
    s_tx_header.DLC = 1;
    s_tx_data[0] = ac_status ? 255 : 0;
    HAL_CAN_AddTxMessage(&hcan, &s_tx_header, s_tx_data.data(), &s_tx_mailbox);
}
