#pragma once


#include <cstdint>
#include <string_view>


namespace sts1cobcsw::periphery::rf
{
using std::literals::operator""sv;


inline constexpr auto callSign = "OE1XST"sv;
inline constexpr auto portableCallSign = "OE1XST portable"sv;


enum class TxType
{
    morse,  // From GPIO pin
    packet  // From FIFO
};


auto Initialize(TxType txType) -> void;
auto ReadPartInfo() -> std::uint16_t;
auto SetTxType(TxType txType) -> void;
auto Send(std::uint8_t const * data, std::size_t length) -> void;
}