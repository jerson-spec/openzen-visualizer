//===========================================================================//
//
// Copyright (C) 2020 LP-Research Inc.
//
// This file is part of OpenZen, under the MIT License.
// See https://bitbucket.org/lpresearch/openzen/src/master/LICENSE for details
// SPDX-License-Identifier: MIT
//
//===========================================================================//

#include "LegacyCoreProperties.h"

#include <cstring>

#include "properties/BaseSensorPropertiesV0.h"
#include "utility/Finally.h"
#include "utility/StringView.h"

namespace zen
{
    LegacyCoreProperties::LegacyCoreProperties(SyncedModbusCommunicator& communicator, ISensorProperties& imu)
        : m_communicator(communicator)
        , m_imu(imu)
    {
    }

    ZenError LegacyCoreProperties::execute(ZenProperty_t command) noexcept
    {
        if (!isExecutable(command))
            return ZenError_UnknownProperty;

        return m_imu.runInCommandMode(
            [command, this] {
                const auto function = static_cast<DeviceProperty_t>(base::v0::mapCommand(command));
                return m_communicator.sendAndWaitForAck(0, function, function, {});
            });
    }

    std::pair<ZenError, size_t> LegacyCoreProperties::getArray(
        ZenProperty_t property, ZenPropertyType propertyType, gsl::span<std::byte> buffer) noexcept
    {
        if (!isArray(property))
            return std::make_pair(ZenError_UnknownProperty, buffer.size());

        if (propertyType != type(property))
            return std::make_pair(ZenError_WrongDataType, buffer.size());

        if (property == ZenSensorProperty_SupportedBaudRates)
            return supportedBaudRates(buffer);

        // older sensor don't support getting the sensor model, just
        // output "legacy" instead
        if (property == ZenSensorProperty_SensorModel)
        {
            const std::string sensorModelLegacy = "Legacy";
            util::stringToSpan(sensorModelLegacy, buffer);
            return std::make_pair(ZenError_None, sensorModelLegacy.size());
        }

        return m_imu.runInCommandMode([property, propertyType, buffer, this]() -> std::pair<ZenError, size_t> {
            const DeviceProperty_t function = static_cast<DeviceProperty_t>(base::v0::map(property, true));
            // [LEGACY] The core sensor only supports integer properties
            switch (propertyType)
            {
            case ZenPropertyType_Byte:
                return m_communicator.sendAndWaitForArray<std::byte>(0, function, function, {}, buffer);
            case ZenPropertyType_Int32:
            {
                // As the communication protocol only supports uint32_t for getting arrays, we need to cast all values to guarantee the correct sign
                // This only applies to Sensor, as IMUComponent has no integer array properties
                // FIXME 20251027 the size calculation appears wrong as it uses the byte count
                //                to size the uint32 span.
                uint32_t* uiBuffer = reinterpret_cast<uint32_t*>(buffer.data());
                const auto result = m_communicator.sendAndWaitForArray(0, function, function, {}, gsl::make_span(uiBuffer, buffer.size()));
                const auto& [error, count] = result;
                if (error)
                    return result;

                int32_t* iBuffer = reinterpret_cast<int32_t*>(buffer.data());
                for (size_t idx = 0; idx < count; ++idx)
                    iBuffer[idx] = static_cast<int32_t>(uiBuffer[idx]);

                // Some properties need to be reversed
                const bool reverse = property == ZenSensorProperty_FirmwareVersion;
                if (reverse)
                    std::reverse(iBuffer, iBuffer + count);

                return { ZenError_None, count };
            }
            default:
                return { ZenError_WrongDataType, buffer.size() };
            }

            }, buffer.size());
    }

    nonstd::expected<bool, ZenError> LegacyCoreProperties::getBool(ZenProperty_t property) noexcept
    {
        // The base sensor has no boolean properties that can be retrieved, so no need to add backwards compatibility
        if (type(property) != ZenPropertyType_Bool)
            return nonstd::make_unexpected(ZenError_UnknownProperty);

        return m_imu.runInCommandMode([property, this] {
            const auto function = static_cast<uint8_t>(property);
            return
                m_communicator
                .sendAndWaitForResult<uint32_t>(0, function, function, {})
                .transform([](uint32_t result) { return result != 0; });
            });
    }

    nonstd::expected<float, ZenError> LegacyCoreProperties::getFloat(ZenProperty_t property) noexcept
    {
        if (isArray(property) || type(property) != ZenPropertyType_Float)
            return nonstd::make_unexpected(ZenError_UnknownProperty);

        const auto function = static_cast<DeviceProperty_t>(base::v0::map(property, true));

        // Skip command mode switching for battery level and voltage reading
        if (function == static_cast<DeviceProperty_t>(EDevicePropertyV0::GetBatteryLevel)
            || function == static_cast<DeviceProperty_t>(EDevicePropertyV0::GetBatteryVoltage)) {
            return m_communicator.sendAndWaitForResult<float>(0, function, function, {});
        }

        return m_imu.runInCommandMode(
            [function, this] {
                return m_communicator.sendAndWaitForResult<float>(0, function, function, {});
            });
    }

    nonstd::expected<int32_t, ZenError> LegacyCoreProperties::getInt32(ZenProperty_t property) noexcept
    {
        if (property == ZenSensorProperty_BaudRate)
            return m_communicator.baudRate();

        if (property != ZenSensorProperty_TimeOffset)
            return ZenError_UnknownProperty;

        return m_imu.runInCommandMode([property, this] {
            const auto function = static_cast<DeviceProperty_t>(base::v0::map(property, true));
            return m_communicator
                .sendAndWaitForResult<uint32_t>(0, function, function, {})
                .transform([](uint32_t result) { return static_cast<int32_t>(result); });
            });
    }

    ZenError LegacyCoreProperties::setInt32(ZenProperty_t property, int32_t value) noexcept
    {
        if (isConstant(property) || isArray(property) || type(property) != ZenPropertyType_Int32)
            return ZenError_UnknownProperty;

        if (property == ZenSensorProperty_BaudRate)
            return m_communicator.setBaudRate(value);
        
        return m_imu.runInCommandMode(
            [property, value, this] {
                // Legacy communication protocol only supports uint32_t
                const uint32_t uiValue = static_cast<uint32_t>(value);

                const auto function = static_cast<DeviceProperty_t>(base::v0::map(property, false));
                if (auto error = m_communicator.sendAndWaitForAck(0, function, function, gsl::make_span(reinterpret_cast<const std::byte*>(&uiValue), sizeof(uiValue))))
                    return error;

                notifyPropertyChange(property, value);
                return ZenError_None;
            });
    }

    bool LegacyCoreProperties::isArray(ZenProperty_t property) const noexcept
    {
        switch (property)
        {
        case ZenSensorProperty_DeviceName:
        case ZenSensorProperty_FirmwareInfo:
        case ZenSensorProperty_FirmwareVersion:
        case ZenSensorProperty_SerialNumber:
        case ZenSensorProperty_SupportedBaudRates:
        case ZenSensorProperty_SensorModel:
            return true;

        default:
            return false;
        }
    }

    bool LegacyCoreProperties::isConstant(ZenProperty_t property) const noexcept
    {
        switch (property)
        {
        case ZenSensorProperty_DeviceName:
        case ZenSensorProperty_FirmwareInfo:
        case ZenSensorProperty_FirmwareVersion:
        case ZenSensorProperty_SerialNumber:
        case ZenSensorProperty_SupportedBaudRates:
        case ZenSensorProperty_BatteryLevel:
        case ZenSensorProperty_BatteryVoltage:
            return true;

        default:
            return false;
        }
    }

    bool LegacyCoreProperties::isExecutable(ZenProperty_t property) const noexcept
    {
        switch (property)
        {
        case ZenSensorProperty_RestoreFactorySettings:
        case ZenSensorProperty_StoreSettingsInFlash:
            return true;

        default:
            return false;
        }
    }

    ZenPropertyType LegacyCoreProperties::type(ZenProperty_t property) const noexcept
    {
        switch (property)
        {
        case ZenSensorProperty_DeviceName:
        case ZenSensorProperty_FirmwareInfo:
        case ZenSensorProperty_SerialNumber:
        case ZenSensorProperty_SensorModel:
            return ZenPropertyType_Byte;

        case ZenSensorProperty_BatteryCharging:
            return ZenPropertyType_Bool;

        case ZenSensorProperty_BatteryLevel:
        case ZenSensorProperty_BatteryVoltage:
            return ZenPropertyType_Float;

        case ZenSensorProperty_FirmwareVersion:
        case ZenSensorProperty_BaudRate:
        case ZenSensorProperty_SupportedBaudRates:
        case ZenSensorProperty_DataMode:
        case ZenSensorProperty_TimeOffset:
            return ZenPropertyType_Int32;

        default:
            return ZenPropertyType_Invalid;
        }
    }

    std::pair<ZenError, size_t> LegacyCoreProperties::supportedBaudRates(gsl::span<std::byte> buffer) const noexcept
    {
        if (auto baudRates = m_communicator.supportedBaudRates())
        {
            if (static_cast<size_t>(buffer.size()) < baudRates->size())
                return std::make_pair(ZenError_BufferTooSmall, baudRates->size());

            if (buffer.data() == nullptr)
                return std::make_pair(ZenError_IsNull, baudRates->size());

            std::memcpy(buffer.data(), baudRates->data(), baudRates->size() * sizeof(int32_t));
            return std::make_pair(ZenError_None, baudRates->size());
        }
        else
        {
            return std::make_pair(baudRates.error(), buffer.size());
        }
    }
}
