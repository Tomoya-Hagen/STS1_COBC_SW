#include <Sts1CobcSw/Periphery/Fram.hpp>
#include <Sts1CobcSw/Serial/Byte.hpp>
#include <Sts1CobcSw/Utility/Span.hpp>

#include <Tests/HardwareTests/Utility.hpp>

#include <rodos/support/support-libs/random.h>
#include <rodos_no_using_namespace.h>

#include <algorithm>
#include <cinttypes>
#include <cstdint>


namespace sts1cobcsw
{
using RODOS::PRINTF;
using sts1cobcsw::operator""_b;  // NOLINT(misc-unused-using-decls)


const size_t testDataSize = 11 * 1024;  // 11 KiB
auto testData = std::array<Byte, testDataSize>{};
auto readData = std::array<Byte, testDataSize>{};


auto PrintDeviceId(fram::DeviceId const & deviceId) -> void;
auto WriteAndReadTestData(fram::Address const & address) -> void;


class FramTest : public RODOS::StaticThread<>
{
public:
    FramTest() : StaticThread("FramTest")
    {
    }


private:
    void init() override
    {
        fram::Initialize();
    }


    void run() override
    {
        PRINTF("\nFRAM test\n\n");

        PRINTF("\n");
        auto actualBaudRate = fram::ActualBaudRate();
        PRINTF("Actual baud rate: %" PRIi32 "\n", actualBaudRate);

        PRINTF("\n");
        auto deviceId = fram::ReadDeviceId();
        PRINTF("Device ID: ");
        PrintDeviceId(deviceId);
        PRINTF(" ==\n");
        PRINTF("           0x7F'7F7F'7F7F'7FC2'2E03\n");
        auto correctDeviceId =
            std::to_array({0x03_b, 0x2E_b, 0xC2_b, 0x7F_b, 0x7F_b, 0x7F_b, 0x7F_b, 0x7F_b, 0x7F_b});
        Check(deviceId == correctDeviceId);

        RODOS::setRandSeed(static_cast<std::uint64_t>(RODOS::NOW()));
        constexpr uint32_t nAdressBits = 20U;
        auto address = fram::Address{RODOS::uint32Rand() % (1U << nAdressBits)};

        PRINTF("\n");
        WriteAndReadTestData(address);
        std::fill(testData.begin(), testData.end(), 0xFF_b);
        WriteAndReadTestData(address);
        std::fill(testData.begin(), testData.end(), 0x00_b);
        WriteAndReadTestData(address);
    }
} framTest;


auto PrintDeviceId(fram::DeviceId const & deviceId) -> void
{
    PRINTF("0x");
    PRINTF("%02x", static_cast<unsigned int>(deviceId[8]));
    PRINTF("'");
    PRINTF("%02x", static_cast<unsigned int>(deviceId[7]));
    PRINTF("%02x", static_cast<unsigned int>(deviceId[6]));
    PRINTF("'");
    PRINTF("%02x", static_cast<unsigned int>(deviceId[5]));
    PRINTF("%02x", static_cast<unsigned int>(deviceId[4]));
    PRINTF("'");
    PRINTF("%02x", static_cast<unsigned int>(deviceId[3]));
    PRINTF("%02x", static_cast<unsigned int>(deviceId[2]));
    PRINTF("'");
    PRINTF("%02x", static_cast<unsigned int>(deviceId[1]));
    PRINTF("%02x", static_cast<unsigned int>(deviceId[0]));
}


auto WriteAndReadTestData(fram::Address const & address) -> void
{
    auto nBytesToPrint = 10U;

    PRINTF("\n");
    PRINTF("Writing %d bytes to address   0x%08x ...\n",
           static_cast<int>(testDataSize),
           static_cast<unsigned int>(address));
    auto begin = RODOS::NOW();
    fram::WriteTo(address, Span(testData));
    auto end = RODOS::NOW();
    PRINTF("  took %d us\n", static_cast<int>((end - begin) / RODOS::MICROSECONDS));

    PRINTF("Reading %d bytes from address 0x%08x ...\n",
           static_cast<int>(testDataSize),
           static_cast<unsigned int>(address));
    begin = RODOS::NOW();
    fram::ReadFrom(address, Span(&readData));
    end = RODOS::NOW();
    PRINTF("  took %d us\n", static_cast<int>((end - begin) / RODOS::MICROSECONDS));

    PRINTF("\n");
    PRINTF("Comparing first %d written and read bytes:\n", nBytesToPrint);
    PRINTF("  ");
    for(auto byte : Span(testData).first(nBytesToPrint))
    {
        PRINTF("0x%02x ", static_cast<unsigned char>(byte));
    }
    PRINTF("\n  ");
    for(auto byte : Span(readData).first(nBytesToPrint))
    {
        PRINTF("0x%02x ", static_cast<unsigned char>(byte));
    }
    PRINTF("\n");
    PRINTF("Comparing the full arrays ...\n");
    Check(readData == testData);
}
}
