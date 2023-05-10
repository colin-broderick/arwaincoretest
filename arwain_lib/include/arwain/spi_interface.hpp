#ifndef _GREEVE_SPI_INTERFACE
#define _GREEVE_SPI_INTERFACE

#include <cstring>

struct SpiConfig
{
    uint8_t mode;
    uint8_t bits_per_word;
    uint32_t speed;
    uint16_t delay;
};

class I_SPI
{
    public:
        virtual bool begin() = 0;
        virtual bool end() = 0;
        virtual int xfer(uint8_t *p_txbuffer, uint8_t p_txlen, uint8_t *p_rxbuffer, uint8_t p_rxlen) = 0;
        virtual int write(uint8_t *p_txbuffer, uint8_t p_txlen) = 0;
        virtual int read(uint8_t *p_rxbuffer, uint8_t p_rxlen) = 0;
};

class MockSpiDevice : public I_SPI
{
    public:
        MockSpiDevice() = default;
        MockSpiDevice(const char *p_spidev)
        {
            (void)p_spidev;
        }
        MockSpiDevice(const char *p_spidev, SpiConfig *p_spi_config)
        {
            (void)p_spidev;
            (void)p_spi_config;
        }
        ~MockSpiDevice()
        {
            
        }
        bool begin()
        {
            return true;
        }
        bool end()
        {
            return true;
        }
        int read(uint8_t *p_rxbuffer, uint8_t p_rxlen)
        {
            for (int i = 0; i < p_rxlen; i++)
            {
                p_rxbuffer[i] = 0xcc;
            }
            (void)p_rxbuffer;
            (void)p_rxlen;
            return 0;
        }
        int write(uint8_t *p_txbuffer, uint8_t p_txlen)
        {
            (void)p_txbuffer;
            (void)p_txlen;
            return 0;
        }
        int xfer(uint8_t *p_txbuffer, uint8_t p_txlen, uint8_t *p_rxbuffer, uint8_t p_rxlen)
        {
            switch (p_txbuffer[0])
            {
                case (0x12 & 0x7f): // Checking IRQFlags
                    p_rxbuffer[1] = (1<<3); // Set TXDONE
                    break;
            }

            (void)p_txbuffer;
            (void)p_txlen;
            (void)p_rxbuffer;
            (void)p_rxlen;
            return 0;
        }
};

#endif
