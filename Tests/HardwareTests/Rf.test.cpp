#include <Sts1CobcSw/Hal/IoNames.hpp>
#include <Sts1CobcSw/Hal/Uart.hpp>
#include <Sts1CobcSw/Periphery/Rf.hpp>
#include <Sts1CobcSw/Utility/Span.hpp>

#include <Tests/HardwareTests/Utility.hpp>

#include <rodos_no_using_namespace.h>

#include <array>
#include <string_view>


namespace sts1cobcsw
{
using periphery::rf::portableCallSign;
using RODOS::PRINTF;


auto uciUart = RODOS::HAL_UART(hal::uciUartIndex, hal::uciUartTxPin, hal::uciUartRxPin);
auto veryLongString = std::string_view(
    "123456789 123456789 123456789 123456789 123456789 "
    "123456789 123456789 123456789 123456789 123456789 "
    "123456789 123456789 123456789 123456789 123456789 "
    "123456789 123456789 123456789 123456789 123456789 "
    "123456789 123456789 123456789 123456789 123456789 "
    "123456789 123456789 123456789 123456789 123456789 "
    "123456789 123456789 123456789 123456789 123456789 "
    "123456789 123456789 123456789 123456789 123456789 "
    "123456789 123456789 123456789 123456789 123456789 "
    "123456789 123456789 123456789 123456789 123456789!");

class RfTest : public RODOS::StaticThread<>
{
public:
    RfTest() : StaticThread("RfTest")
    {
    }


private:
    void init() override
    {
    }


    void run() override
    {
        PRINTF("\nRF test\n\n");

        periphery::rf::Initialize(periphery::rf::TxType::morse);
        PRINTF("RF module initialized\n");

        PRINTF("\n");
        auto correctPartInfo = 0x4463;
        auto partInfo = periphery::rf::ReadPartInfo();
        PRINTF("Part info: 0x%4x == 0x%4x\n", partInfo, correctPartInfo);
        Check(partInfo == correctPartInfo);

        // Here comes the rest of the RF test
        while(true)
        {
            PRINTF("\n");
            PRINTF("What do you want to test?\n");
            PRINTF("  m: morsing\n");
            PRINTF("  t: transmitting a packet\n");
            PRINTF("  r: receiving\n");

            auto command = std::array<char, 1>{0};
            hal::ReadFrom(&uciUart, Span(&command));
            PRINTF("\n");

            auto pauseDuration = 1 * RODOS::SECONDS;
            switch(command[0])
            {
                case 'm':
                {
                    PRINTF("Not implemented\n");
                    break;
                }
                case 't':
                {
                    auto nTransmissions = 5;
                    periphery::rf::SetTxType(periphery::rf::TxType::packet);
                    PRINTF("Transmitting call sign 'OE1XST' %d times\n", nTransmissions);
                    for(auto i = 0; i < nTransmissions; ++i)
                    {
                        RODOS::PRINTF("Transmitting...\n");
                        periphery::rf::Send(
                            reinterpret_cast<std::uint8_t const *>(portableCallSign.data()),
                            portableCallSign.length());
                        RODOS::AT(RODOS::NOW() + 100 * RODOS::MILLISECONDS);
                        periphery::rf::Send(
                            reinterpret_cast<std::uint8_t const *>(veryLongString.data()),
                            veryLongString.length());
                        RODOS::AT(RODOS::NOW() + pauseDuration);
                    }
                    break;
                }
                case 'r':
                {
                    PRINTF("Not implemented\n");
                    break;
                }
                default:
                {
                    PRINTF("Unused character\n");
                    break;
                }
            }
        }
    }
} rfTest;
}
