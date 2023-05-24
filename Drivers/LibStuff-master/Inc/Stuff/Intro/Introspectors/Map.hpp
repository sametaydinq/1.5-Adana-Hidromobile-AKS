#pragma once

#include <functional>
#include <map>

#include <Stuff/Intro/Introspector.hpp>

namespace std { // NOLINT(cert-dcl58-cpp)

template<typename T> struct Introspector;

template<typename Key, typename T, typename Compare, typename Allocator>
struct Introspector<map<Key, T, Compare, Allocator>> {
    using type = unordered_map<Key, T, Compare, Allocator>;
    using key_type = Key;
    using member_type = T;

    static constexpr size_t size(type const& v) { return v.size(); }

    static T const& at(type const& v, Key const& key) { return v.at(key); }
    static T& at(type& v, Key const& key) { return v.at(key); }

    template<typename Fn> static void iterate(type const& v, Fn&& fn) {
        for (auto const& [key, value] : v) {
            std::invoke(fn, key, value);
        }
    }

    template<typename Fn> static void iterate(type& v, Fn&& fn) {
        iterate(v, [fn = std::forward<Fn>(fn)](auto const& k, auto const& v) {
            std::invoke(fn, k, const_cast<key_type&>(v));
        });
    }
};

template<typename Key, typename T, typename Compare, typename Allocator>
STF_MAKE_ADL_INTROSPECTOR(map<Key, T, Compare, Allocator>)

}
