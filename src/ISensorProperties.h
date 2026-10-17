//===========================================================================//
//
// Copyright (C) 2020 LP-Research Inc.
//
// This file is part of OpenZen, under the MIT License.
// See https://bitbucket.org/lpresearch/openzen/src/master/LICENSE for details
// SPDX-License-Identifier: MIT
//
//===========================================================================//

#ifndef ZEN_ISENSORPROPERTIES_H_
#define ZEN_ISENSORPROPERTIES_H_

#include <cstdint>
#include <cstddef>
#include <functional>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

#include <gsl/span>
#include <nonstd/expected.hpp>

#include "ZenTypes.h"

namespace zen
{
    using SensorPropertyValue = std::variant<
        bool,
        float,
        int32_t,
        uint64_t,
        gsl::span<const std::byte>
    >;

    using SensorPropertyChangeCallback = std::function<void(SensorPropertyValue)>;

    class ISensorProperties
    {
    public:
        virtual ~ISensorProperties() = default;

        /** If successful executes the property, otherwise returns an error. */
        virtual ZenError execute(ZenProperty_t property) noexcept = 0;

        /** If successful fills the buffer with the array of properties and sets the buffer's size.
         * Otherwise, returns an error and potentially sets the desired buffer size - if it is too small.
         */
        virtual std::pair<ZenError, size_t> getArray(ZenProperty_t, ZenPropertyType, gsl::span<std::byte>) noexcept {
            return std::make_pair(ZenError_UnknownProperty, 0);
        }

        /** If successful fills the value with the property's boolean value, otherwise returns an error. */
        virtual nonstd::expected<bool, ZenError> getBool(ZenProperty_t) noexcept {
            return nonstd::make_unexpected(ZenError_UnknownProperty); }

        /** If successful fills the value with the property's floating-point value, otherwise returns an error. */
        virtual nonstd::expected<float, ZenError> getFloat(ZenProperty_t) noexcept {
            return nonstd::make_unexpected(ZenError_UnknownProperty); }

        /** If successful fills the value with the property's signed integer value, otherwise returns an error. */
        virtual nonstd::expected<int32_t, ZenError> getInt32(ZenProperty_t) noexcept {
            return nonstd::make_unexpected(ZenError_UnknownProperty); }

        /** If successful fills the value with the property's unsigned integer value, otherwise returns an error. */
        virtual nonstd::expected<uint64_t, ZenError> getUInt64(ZenProperty_t) noexcept {
            return nonstd::make_unexpected(ZenError_UnknownProperty); }

        /** If successful sets the array properties, otherwise returns an error. */
        virtual ZenError setArray(ZenProperty_t, ZenPropertyType, gsl::span<const std::byte>) noexcept {
            return ZenError_UnknownProperty;
        }

        /** If successful sets the boolean property, otherwise returns an error. */
        virtual ZenError setBool(ZenProperty_t, bool) noexcept {
            return ZenError_UnknownProperty;
        }

        /** If successful sets the floating-point property, otherwise returns an error. */
        virtual ZenError setFloat(ZenProperty_t, float) noexcept {
            return ZenError_UnknownProperty;
        }

        /** If successful sets the integer property, otherwise returns an error. */
        virtual ZenError setInt32(ZenProperty_t, int32_t) noexcept {
            return ZenError_UnknownProperty;
        }

        /** If successful sets the unsigned integer property, otherwise returns an error. */
        virtual ZenError setUInt64(ZenProperty_t, uint64_t) {
            return ZenError_UnknownProperty;
        }

        /** Returns whether the property is an array type */
        virtual bool isArray(ZenProperty_t) const noexcept {
            return false;
        }

        /** Returns whether the property is constant. If so, the property cannot be set */
        virtual bool isConstant(ZenProperty_t) const noexcept {
            return false;
        }

        /** Returns whether the property can be executed as a command */
        virtual bool isExecutable(ZenProperty_t) const noexcept {
            return false;
        }

        /** Returns the type of the property */
        virtual ZenPropertyType type(ZenProperty_t) const noexcept = 0;

        /** Subscribes to change notifications of the property */
        void subscribeToPropertyChanges(ZenProperty_t property, SensorPropertyChangeCallback callback) noexcept;

        // Used by sensor implementations to make sure command mode is set and reset around an interaction
        // with the sensor.
        template<typename T, typename... Args>
        auto runInCommandMode(T&& F, Args&&... args);

    protected:
        /** Trigger  */
        void notifyPropertyChange(ZenProperty_t property, SensorPropertyValue value) const noexcept;

    private:
        std::unordered_map<ZenProperty_t, std::vector<SensorPropertyChangeCallback>> m_subscriberCallbacks;
    };

    namespace details {
        template<typename>
        struct is_nonstd_expected : std::false_type {};

        template<typename T, typename U>
        struct is_nonstd_expected<nonstd::expected<T, U>> : std::true_type {};

        // getArray returns a std::pair, the setters return ZenError, all the other functions
        // return nonstd::unexpected, so we need to allow for that in the runInCommandMode
        // helper below.
        //
        // We achieve this by wrapping the early return in the case where setting command
        // mode fails and using an explicit specialization for the getArray case.  This is a
        // variadic template in order to handle getArray returning the buffer size along with
        // a potential error.
        template<typename T, typename... Args>
        struct ErrorBuilder {
            using ReturnType = decltype(std::declval<T>()());
            static_assert(sizeof...(Args) == 0, "This specialization takes no additional args");
            auto operator()(ZenError error) -> ReturnType {
                if constexpr (is_nonstd_expected<ReturnType>::value)
                    return nonstd::make_unexpected(error);
                else {
                    // setters and execute() returns the error directly
                    static_assert(std::is_same_v<ReturnType, ZenError>);
                    return error;
                }
            }
        };

        template<typename T>
        struct ErrorBuilder<T, size_t> {
            // Used for the case of getArray.
            using ReturnType = decltype(std::declval<T>()());
            static_assert(std::is_same_v<ReturnType, std::pair<ZenError, size_t>>,
                "Should match getArray's return type");

            ErrorBuilder() = delete; // Ensure initialization of bufSize.

            size_t bufSize;
            auto operator()(ZenError error) -> ReturnType
            {
                return std::make_pair(error, bufSize);
            }
        };
    }

    template<typename T, typename... Args>
    auto ISensorProperties::runInCommandMode(T&& F, Args&&... args) {
        static_assert(
            sizeof...(Args) == 0
            || (sizeof...(Args) == 1 && (std::is_same_v<Args, size_t> &&...)),
            "Only zero args or one size_t arg are supported");

        auto pStreaming = getBool(ZenImuProperty_StreamData);
        if (!pStreaming.has_value())
            return details::ErrorBuilder<T, Args...>{args...}(pStreaming.error());

        auto guard = finally([&]() {
            if (pStreaming.value())
                setBool(ZenImuProperty_StreamData, true);
            });

        return F();
    }

}

#endif
