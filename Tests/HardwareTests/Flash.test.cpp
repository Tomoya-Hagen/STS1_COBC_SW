#include <Sts1CobcSw/Periphery/Flash.hpp>
#include <Sts1CobcSw/Serial/Byte.hpp>

#include <Tests/HardwareTests/Utility.hpp>

#include <rodos_no_using_namespace.h>

#include <algorithm>
#include <cinttypes>
#include <cstdint>
#include <string_view>


namespace sts1cobcsw
{
using RODOS::PRINTF;


constexpr std::size_t stackSize = 5'000;


auto Print(flash::Page const & page) -> void;


class FlashTest : public RODOS::StaticThread<stackSize>
{
public:
    FlashTest() : StaticThread("FlashTest")
    {
    }


private:
    void init() override
    {
        flash::Initialize();
    }


    void run() override
    {
        PRINTF("\nFlash test\n\n");

        PRINTF("\n");
        auto actualBaudRate = flash::ActualBaudRate();
        PRINTF("Actual baud rate: %" PRIi32 "\n", actualBaudRate);

        PRINTF("\n");
        auto jedecId = flash::ReadJedecId();
        PRINTF("Manufacturer ID: 0x%02x == 0xEF\n",
               static_cast<unsigned int>(jedecId.manufacturerId));
        Check(jedecId.manufacturerId == 0xEF);
        PRINTF("Device ID: 0x%04x == 0x4021\n", static_cast<unsigned int>(jedecId.deviceId));
        Check(jedecId.deviceId == 0x4021);

        PRINTF("\n");
        auto statusRegister = flash::ReadStatusRegister(1);
        PRINTF("Status register 1: 0x%02x == 0x00\n", static_cast<unsigned int>(statusRegister));
        Check(statusRegister == 0x00_b);

        statusRegister = flash::ReadStatusRegister(2);
        PRINTF("Status register 2: 0x%02x == 0x02\n", static_cast<unsigned int>(statusRegister));
        Check(statusRegister == 0x02_b);

        statusRegister = flash::ReadStatusRegister(3);
        PRINTF("Status register 3: 0x%02x == 0x41\n", static_cast<unsigned int>(statusRegister));
        Check(statusRegister == 0x41_b);

        std::uint32_t const pageAddress = 0x00'01'00'00U;

        PRINTF("\n");
        PRINTF("Reading page at address 0x%08x:\n", static_cast<unsigned int>(pageAddress));
        auto page = flash::ReadPage(pageAddress);
        Print(page);

        PRINTF("\n");
        std::fill(begin(page), end(page), 0x00_b);
        PRINTF("Programming page at address 0x%08x:\n", static_cast<unsigned int>(pageAddress));
        Print(page);
        auto begin = RODOS::NOW();
        flash::ProgramPage(pageAddress, page);
        auto endPage = RODOS::NOW();

        flash::WaitWhileBusy();
        auto end = RODOS::NOW();
        PRINTF("ProgrammPage took %d us\n",
               static_cast<int>((endPage - begin) / RODOS::MICROSECONDS));
        PRINTF("WaitWhileBusy took %d us\n",
               static_cast<int>((end - endPage) / RODOS::MICROSECONDS));

        PRINTF("\n");
        PRINTF("Reading page at address 0x%08x:\n", static_cast<unsigned int>(pageAddress));
        page = flash::ReadPage(pageAddress);
        Print(page);

        PRINTF("\n");
        PRINTF("Erasing sector containing address 0x%08x:\n",
               static_cast<unsigned int>(pageAddress));
        flash::EraseSector(pageAddress);

        begin = RODOS::NOW();
        flash::WaitWhileBusy();
        end = RODOS::NOW();
        PRINTF("Erasing sector took %d us\n",
               static_cast<int>((end - begin) / RODOS::MICROSECONDS));

        PRINTF("\n");
        PRINTF("Reading page at address 0x%08x:\n", static_cast<unsigned int>(pageAddress));
        page = flash::ReadPage(pageAddress);
        Print(page);
    }
} flashTest;


auto Print(flash::Page const & page) -> void
{
    constexpr auto nRows = 16;
    auto iRow = 0;
    for(auto x : page)
    {
        PRINTF(" 0x%02x", static_cast<unsigned int>(x));
        iRow++;
        if(iRow == nRows)
        {
            PRINTF("\n");
            iRow = 0;
        }
    }
}
}