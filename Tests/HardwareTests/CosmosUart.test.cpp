//! @file
//! @brief A program for testing some Cosmos commands via UART
//!
//! Preparation

#include <Sts1CobcSw/Hal/Communication.hpp>
#include <Sts1CobcSw/Hal/IoNames.hpp>
#include <Sts1CobcSw/Serial/Byte.hpp>
#include <Sts1CobcSw/Serial/Serial.hpp>

#include <type_safe/types.hpp>

#include <rodos_no_using_namespace.h>

#include <array>
#include <span>


namespace sts1cobcsw
{
namespace ts = type_safe;
using ts::operator""_u8;
using ts::operator""_u16;
using ts::operator""_u32;
using ts::operator""_i16;
using ts::operator""_i32;
using sts1cobcsw::serial::Byte;
using sts1cobcsw::serial::DeserializeFrom;
using sts1cobcsw::serial::SerializeTo;


auto uciUart = RODOS::HAL_UART(hal::uciUartIndex, hal::uciUartTxPin, hal::uciUartRxPin);
constexpr auto basicTelemetryLength = 5_u8;


struct CosmosTestCommand
{
    ts::uint8_t id = 0_u8;
    ts::int32_t timestamp = 0_i32;
    ts::int16_t length = 0_i16;
    ts::uint8_t data = 0_u8;
    ts::uint32_t crc32 = 0_u32;
};


struct BasicTelemetry
{
    ts::uint8_t id = 136_u8;
    ts::int32_t timestamp = 0_i32;
    ts::int16_t length = 5_u8;
    ts::uint8_t padding = 0_u8;
    ts::uint32_t crc32 = 0_u32;
};


struct StatusTelemetry
{
    ts::uint8_t id = 0_u8;
    ts::int32_t timestamp = 0_i32;
    ts::int16_t length = 0_i16;
    ts::uint8_t status1 = 0_u8;
    ts::uint8_t status2 = 0_u8;
    std::array<ts::uint8_t, 3> padding = {0_u8, 0_u8, 0_u8};
    ts::uint32_t crc32 = 0_u32;
};


struct TemperatureTelemetry
{
    ts::uint8_t id = 0_u8;
    ts::int32_t timestamp = 0_i32;
    ts::int16_t length = 0_i16;
    ts::uint16_t temperature = 0_u16;
    std::array<ts::uint8_t, 3> padding = {0_u8, 0_u8, 0_u8};
    ts::uint32_t crc32 = 0_u32;
};


namespace serial
{
template<>
constexpr std::size_t serialSize<CosmosTestCommand> =
    totalSerialSize<decltype(CosmosTestCommand::id),
                    decltype(CosmosTestCommand::timestamp),
                    decltype(CosmosTestCommand::length),
                    decltype(CosmosTestCommand::data),
                    decltype(CosmosTestCommand::crc32)>;

template<>
constexpr std::size_t serialSize<BasicTelemetry> =
    totalSerialSize<decltype(BasicTelemetry::id),
                    decltype(BasicTelemetry::timestamp),
                    decltype(BasicTelemetry::length),
                    decltype(BasicTelemetry::padding),
                    decltype(BasicTelemetry::crc32)>;

template<>
constexpr std::size_t serialSize<StatusTelemetry> =
    totalSerialSize<decltype(StatusTelemetry::id),
                    decltype(StatusTelemetry::timestamp),
                    decltype(StatusTelemetry::length),
                    decltype(StatusTelemetry::status1),
                    decltype(StatusTelemetry::status2),
                    decltype(StatusTelemetry::padding),
                    decltype(StatusTelemetry::crc32)>;

template<>
constexpr std::size_t serialSize<TemperatureTelemetry> =
    totalSerialSize<decltype(TemperatureTelemetry::id),
                    decltype(TemperatureTelemetry::timestamp),
                    decltype(TemperatureTelemetry::length),
                    decltype(TemperatureTelemetry::temperature),
                    decltype(TemperatureTelemetry::padding),
                    decltype(TemperatureTelemetry::crc32)>;
}


auto DeserializeFrom(Byte * source, CosmosTestCommand * data) -> Byte *
{
    source = DeserializeFrom(source, &(data->id));
    source = DeserializeFrom(source, &(data->timestamp));
    source = DeserializeFrom(source, &(data->length));
    source = DeserializeFrom(source, &(data->data));
    source = DeserializeFrom(source, &(data->crc32));
    return source;
}


auto SerializeTo(Byte * destination, BasicTelemetry const & data) -> Byte *
{
    destination = SerializeTo(destination, data.id);
    destination = SerializeTo(destination, data.timestamp);
    destination = SerializeTo(destination, data.length);
    destination = SerializeTo(destination, data.padding);
    destination = SerializeTo(destination, data.crc32);
    return destination;
}


class CosmosUartTest : public RODOS::StaticThread<>
{
    void init() override
    {
        constexpr auto uartBaudRate = 115200;
        uciUart.init(uartBaudRate);
    }


    void run() override
    {
        hal::WriteTo(&uciUart, "Hello UCI UART!\n");

        while(true)
        {
            auto commandBuffer = serial::SerialBuffer<CosmosTestCommand>{};
            auto bufferSpan = std::span<Byte>(commandBuffer);
            hal::ReadFrom(&uciUart, bufferSpan);
            hal::WriteTo(&uciUart, "Done reading\n");
            auto basicTelemetry = BasicTelemetry();
            auto basicSerialBuffer = serial::Serialize(basicTelemetry);
            hal::WriteTo(&uciUart, std::span<Byte>(basicSerialBuffer));
            hal::WriteTo(&uciUart, "Done sending\n");
        }
    }
};


auto const cosmosUartTest = CosmosUartTest();
}
