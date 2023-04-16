//! @file
//! @brief A program for testing some Cosmos commands via UART
//!
//! Preparation

#include <Sts1CobcSw/Hal/Communication.hpp>
#include <Sts1CobcSw/Hal/IoNames.hpp>
#include <Sts1CobcSw/Serial/Byte.hpp>
#include <Sts1CobcSw/Serial/Serial.hpp>
#include <Sts1CobcSw/Utility/Crc32.hpp>
#include <Sts1CobcSw/Utility/Time.hpp>

#include <type_safe/types.hpp>

#include <random.h>

#include <rodos_no_using_namespace.h>

#include <array>
#include <span>


namespace sts1cobcsw
{
namespace ts = type_safe;
using ts::operator""_u8;
using ts::operator""_u32;
using ts::operator""_i16;
using ts::operator""_i32;
using sts1cobcsw::serial::Byte;
using sts1cobcsw::serial::DeserializeFrom;
using sts1cobcsw::serial::SerializeTo;


auto uciUart = RODOS::HAL_UART(hal::uciUartIndex, hal::uciUartTxPin, hal::uciUartRxPin);
constexpr auto basicCommunicationCmdId = 118_u8;
constexpr auto dataCollectionCmdId = 129_u8;
constexpr auto statusCollectionMode = 0_u8;
constexpr auto temperatureCollectionMode = 1_u8;
constexpr auto temperaturePadding = 3;
constexpr auto statusPadding = 3;

struct CosmosTestCommand
{
    ts::uint8_t id = 0_u8;
    ts::int32_t timestamp = 0_i32;
    ts::int16_t length = 0_i16;
    ts::uint8_t data = 0_u8;
};


struct BasicTelemetry
{
    ts::uint8_t id = 136_u8;
    ts::int32_t timestamp = 0_i32;
    ts::int16_t length = 5_i16;
    ts::uint8_t padding = 0_u8;
};


struct StatusTelemetry
{
    ts::uint8_t id = 0_u8;
    ts::int32_t timestamp = 0_i32;
    ts::int16_t length = 9_i16;
    ts::uint8_t status1 = 0_u8;
    ts::uint8_t status2 = 0_u8;
};


struct TemperatureTelemetry
{
    ts::uint8_t id = 0_u8;
    ts::int32_t timestamp = 0_i32;
    ts::int16_t length = 9_i16;
    ts::int16_t temperature = 0_i16;
};


namespace serial
{
template<>
constexpr std::size_t serialSize<CosmosTestCommand> =
    totalSerialSize<decltype(CosmosTestCommand::id),
                    decltype(CosmosTestCommand::timestamp),
                    decltype(CosmosTestCommand::length),
                    decltype(CosmosTestCommand::data)>;

template<>
constexpr std::size_t serialSize<BasicTelemetry> =
    totalSerialSize<decltype(BasicTelemetry::id),
                    decltype(BasicTelemetry::timestamp),
                    decltype(BasicTelemetry::length),
                    decltype(BasicTelemetry::padding)>;

template<>
constexpr std::size_t serialSize<StatusTelemetry> =
    totalSerialSize<decltype(StatusTelemetry::id),
                    decltype(StatusTelemetry::timestamp),
                    decltype(StatusTelemetry::length),
                    decltype(StatusTelemetry::status1),
                    decltype(StatusTelemetry::status2)>
    + statusPadding;

template<>
constexpr std::size_t serialSize<TemperatureTelemetry> =
    totalSerialSize<decltype(TemperatureTelemetry::id),
                    decltype(TemperatureTelemetry::timestamp),
                    decltype(TemperatureTelemetry::length),
                    decltype(TemperatureTelemetry::temperature)>
    + temperaturePadding;
}


auto DeserializeFrom(Byte * source, CosmosTestCommand * data) -> Byte *
{
    source = DeserializeFrom(source, &(data->id));
    source = DeserializeFrom(source, &(data->timestamp));
    source = DeserializeFrom(source, &(data->length));
    source = DeserializeFrom(source, &(data->data));
    return source;
}


auto SerializeTo(Byte * destination, BasicTelemetry const & data) -> Byte *
{
    destination = SerializeTo(destination, data.id);
    destination = SerializeTo(destination, data.timestamp);
    destination = SerializeTo(destination, data.length);
    destination = SerializeTo(destination, data.padding);
    return destination;
}

auto SerializeTo(Byte * destination, StatusTelemetry const & data) -> Byte *
{
    destination = SerializeTo(destination, data.id);
    destination = SerializeTo(destination, data.timestamp);
    destination = SerializeTo(destination, data.length);
    destination = SerializeTo(destination, data.status1);
    destination = SerializeTo(destination, data.status2);
    for(auto i = 0; i < statusPadding; i++)
    {
        destination = SerializeTo(destination, 0_u8);
    }

    return destination;
}

auto SerializeTo(Byte * destination, TemperatureTelemetry const & data) -> Byte *
{
    destination = SerializeTo(destination, data.id);
    destination = SerializeTo(destination, data.timestamp);
    destination = SerializeTo(destination, data.length);
    destination = SerializeTo(destination, data.temperature);
    for(auto i = 0; i < temperaturePadding; i++)
    {
        destination = SerializeTo(destination, 0_u8);
    }
    return destination;
}

template<typename T>
auto SendPacket(T & packet, RODOS::HAL_UART * uart)
{
    packet.timestamp = static_cast<ts::int32_t>(utility::GetUnixUtc());

    auto packetBuffer = serial::Serialize(packet);
    auto crc32 = utility::Crc32(packetBuffer);
    auto crc32Buffer = serial::Serialize(crc32);

    hal::WriteTo(uart, std::span<Byte, std::size(packetBuffer)>(packetBuffer));
    hal::WriteTo(uart, std::span<Byte, std::size(crc32Buffer)>(crc32Buffer));
}

class CosmosUartTest : public RODOS::StaticThread<>
{
    void init() override
    {
        constexpr auto uartBaudRate = 115'200;
        uciUart.init(uartBaudRate);
    }


    void run() override
    {
        while(true)
        {
            auto commandBuffer = serial::SerialBuffer<CosmosTestCommand>{};
            auto commandCrc32Buffer = serial::SerialBuffer<ts::uint32_t>{};

            hal::ReadFrom(&uciUart, std::span<Byte, std::size(commandBuffer)>(commandBuffer));
            hal::ReadFrom(&uciUart,
                          std::span<Byte, std::size(commandCrc32Buffer)>(commandCrc32Buffer));

            auto command = serial::Deserialize<CosmosTestCommand>(commandBuffer);
            auto commandCrc32 = serial::Deserialize<ts::uint32_t>(commandCrc32Buffer);
            auto checkCrc32 = utility::Crc32(commandBuffer);

            if(commandCrc32 != checkCrc32)
            {
                RODOS::PRINTF("Received: %lu\n", static_cast<std::uint32_t>(commandCrc32));
                RODOS::PRINTF("Calculated: %lu\n", static_cast<std::uint32_t>(checkCrc32));
                hal::WriteTo(&uciUart, "CRC32 error");
                continue;
            }

            if(command.id == basicCommunicationCmdId)
            {
                auto basicTelemetry = BasicTelemetry{};
                SendPacket(basicTelemetry, &uciUart);
            }
            else if(command.id == dataCollectionCmdId)
            {
                auto randomUint32 = RODOS::uint32Rand();
                auto randomByte1 = static_cast<std::uint8_t>(randomUint32);
                auto randomByte2 = static_cast<std::uint8_t>(randomUint32 >> 8);
                auto randomDoubleByte = static_cast<std::int16_t>(randomUint32 >> 16);
                if(command.data == statusCollectionMode)
                {
                    auto statusTelemetry = StatusTelemetry();
                    statusTelemetry.status1 = 9_u8;
                    statusTelemetry.status2 = 10_u8;
                    SendPacket(statusTelemetry, &uciUart);
                }
                else if(command.data == temperatureCollectionMode)
                {
                    auto temperatureTelemetry = TemperatureTelemetry();
                    temperatureTelemetry.temperature = 42_i16;
                    SendPacket(temperatureTelemetry, &uciUart);
                }
            }
            else
            {
                hal::WriteTo(&uciUart, "Error");
            }
        }
    }
};


auto const cosmosUartTest = CosmosUartTest();
}
