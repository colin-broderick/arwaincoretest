#ifndef _GREEVE_SPI_INERFACE
#define _GREEVE_SPI_INERFACE

#include "spi_interface.hpp"

#if !UNIT_TESTS
class LinuxSpiDevice : public I_SPI
{
    TESTABLE: // Attributes
        char *m_spidev = nullptr;
        int m_spifd;
        SpiConfig m_spiconfig;
        bool m_open;

    TESTABLE: // Methods
        bool set_speed(uint32_t p_speed);
        bool set_mode(uint8_t p_mode);
        bool set_bit_per_word(uint8_t p_bit);
        bool set_config(SpiConfig *p_spi_config);

    public:
        LinuxSpiDevice() = default;
        LinuxSpiDevice(const char *p_spidev);
        LinuxSpiDevice(const char *p_spidev, SpiConfig *p_spi_config);
        ~LinuxSpiDevice();
        bool begin();
        bool end();
        int read(uint8_t *p_rxbuffer, uint8_t p_rxlen);
        int write(uint8_t *p_txbuffer, uint8_t p_txlen);
        int xfer(uint8_t *p_txbuffer, uint8_t p_txlen, uint8_t *p_rxbuffer, uint8_t p_rxlen);
};

#endif
#endif
