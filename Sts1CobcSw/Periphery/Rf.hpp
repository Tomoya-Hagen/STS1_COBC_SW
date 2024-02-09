#pragma once


#include <cstdint>


namespace sts1cobcsw::periphery::rf
{
enum class TxType
{
    morse,  // From GPIO pin
    packet  // From FIFO
};

auto Initialize(TxType txType) -> void;
auto ReadPartInfo() -> std::uint16_t;
auto SetTxType(TxType txType) -> void;
auto TransmitData(std::uint8_t const * data, std::size_t length) -> void;
}