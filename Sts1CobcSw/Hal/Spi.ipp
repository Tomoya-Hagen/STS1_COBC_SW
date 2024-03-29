#pragma once


#include <Sts1CobcSw/Hal/Spi.hpp>

#include <rodos_no_using_namespace.h>


namespace sts1cobcsw::hal
{
template<typename T, std::size_t extent>
inline auto WriteTo(RODOS::HAL_SPI * spi, std::span<T const, extent> data) -> void
{
    // spi.write() only returns -1 or the given buffer length. It only returns -1 if the SPI is not
    // initialized, which we can check/ensure statically. Therefore, we do not need to check the
    // return value at runtime.
    spi->write(data.data(), data.size_bytes());
}


template<typename T, std::size_t extent>
inline auto ReadFrom(RODOS::HAL_SPI * spi, std::span<T, extent> data) -> void
{
    // spi.read() only returns -1 or the given buffer length. It only returns -1 if the SPI is not
    // initialized, which we can check/ensure statically. Therefore, we do not need to check the
    // return value at runtime.
    spi->read(data.data(), data.size_bytes());
}
}
