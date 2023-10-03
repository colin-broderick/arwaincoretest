#include <stdio.h>
#include <stdint.h>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <cstring>
#include <linux/spi/spidev.h>

#include "spi.hpp"

#if !UNIT_TESTS
bool LinuxSpiDevice::set_bit_per_word(uint8_t p_bit)
{

    /* Set bits per word*/
    if (ioctl(m_spifd, SPI_IOC_WR_BITS_PER_WORD, &p_bit) < 0)
    {
        return false;
    }
    if (ioctl(m_spifd, SPI_IOC_RD_BITS_PER_WORD, &p_bit) < 0)
    {
        return false;
    }

    m_spiconfig.bits_per_word = p_bit;
    return true;
}

bool LinuxSpiDevice::set_speed(uint32_t p_speed)
{
    /* Set SPI speed*/
    if (ioctl(m_spifd, SPI_IOC_WR_MAX_SPEED_HZ, &p_speed) < 0)
    {
        return false;
    }
    if (ioctl(m_spifd, SPI_IOC_RD_MAX_SPEED_HZ, &p_speed) < 0)
    {
        return false;
    }

    m_spiconfig.mode = p_speed;

    return true;
}

bool LinuxSpiDevice::set_mode(uint8_t p_mode)
{
    /* Set SPI_POL and SPI_PHA */
    if (ioctl(m_spifd, SPI_IOC_WR_MODE, &p_mode) < 0)
    {
        return false;
    }
    if (ioctl(m_spifd, SPI_IOC_RD_MODE, &p_mode) < 0)
    {
        return false;
    }

    m_spiconfig.mode = p_mode;
    return true;
}

int LinuxSpiDevice::xfer(uint8_t *p_txbuffer, uint8_t p_txlen, uint8_t *p_rxbuffer, uint8_t p_rxlen)
{
    struct spi_ioc_transfer spi_message[1];
    std::memset(spi_message, 0, sizeof(spi_message));

    spi_message[0].rx_buf = (unsigned long)p_rxbuffer;
    spi_message[0].tx_buf = (unsigned long)p_txbuffer;
    spi_message[0].len = p_txlen;
    return ioctl(m_spifd, SPI_IOC_MESSAGE(1), spi_message);
}

int LinuxSpiDevice::write(uint8_t *p_txbuffer, uint8_t p_txlen)
{
    struct spi_ioc_transfer spi_message[1];
    std::memset(spi_message, 0, sizeof(spi_message));
    spi_message[0].tx_buf = (unsigned long)p_txbuffer;
    spi_message[0].len = p_txlen;

    return ioctl(m_spifd, SPI_IOC_MESSAGE(1), spi_message);
}

int LinuxSpiDevice::read(uint8_t *p_rxbuffer, uint8_t p_rxlen)
{
    struct spi_ioc_transfer spi_message[1];
    std::memset(spi_message, 0, sizeof(spi_message));

    spi_message[0].rx_buf = (unsigned long)p_rxbuffer;
    spi_message[0].len = p_rxlen;
    return ioctl(m_spifd, SPI_IOC_MESSAGE(1), spi_message);
}

bool LinuxSpiDevice::end()
{
    // What the feck is this here for?!
}

bool LinuxSpiDevice::begin()
{
    /* open spidev device */
    if (m_open == true)
    {
        return true;
    }
    if (m_spidev == nullptr)
    {
        return false;
    }
    m_spifd = open(m_spidev, O_RDWR);

    if (m_spifd < 0)
    {
        return false;
    }
    /* Set SPI_POL and SPI_PHA */
    if (ioctl(m_spifd, SPI_IOC_WR_MODE, &m_spiconfig.mode) < 0)
    {
        close(m_spifd);
        return false;
    }
    if (ioctl(m_spifd, SPI_IOC_RD_MODE, &m_spiconfig.mode) < 0)
    {
        close(m_spifd);
        return false;
    }

    /* Set bits per word*/
    if (ioctl(m_spifd, SPI_IOC_WR_BITS_PER_WORD, &m_spiconfig.bits_per_word) < 0)
    {
        close(m_spifd);
        return false;
    }
    if (ioctl(m_spifd, SPI_IOC_RD_BITS_PER_WORD, &m_spiconfig.bits_per_word) < 0)
    {
        close(m_spifd);
        return false;
    }

    /* Set SPI speed*/
    if (ioctl(m_spifd, SPI_IOC_WR_MAX_SPEED_HZ, &m_spiconfig.speed) < 0)
    {
        close(m_spifd);
        return false;
    }
    if (ioctl(m_spifd, SPI_IOC_RD_MAX_SPEED_HZ, &m_spiconfig.speed) < 0)
    {
        close(m_spifd);
        return false;
    }

    m_open = true;

    return true;
}

LinuxSpiDevice::LinuxSpiDevice(const char *p_spidev)
{
    m_spidev = nullptr;
    if (p_spidev != nullptr)
    {
        m_spidev = (char *)malloc(strlen(p_spidev) + 1);
        if (m_spidev != nullptr)
        {
            strcpy(m_spidev, p_spidev);
        }
    }
    m_open = false;
}

LinuxSpiDevice::LinuxSpiDevice(const char *p_spidev, SpiConfig *p_spi_config)
{
    m_spidev = nullptr;
    if (p_spidev != nullptr)
    {
        m_spidev = (char *)malloc(strlen(p_spidev) + 1);
        if (m_spidev != nullptr)
        {
            strcpy(m_spidev, p_spidev);
        }
    }
    if (p_spi_config != nullptr)
    {
        memcpy(&m_spiconfig, p_spi_config, sizeof(SpiConfig));
    }
    else
    {
        m_spiconfig.mode = 0;
        m_spiconfig.speed = 1000000;
        m_spiconfig.bits_per_word = 8;
        m_spiconfig.delay = 0;
    }
    m_open = false;
}

LinuxSpiDevice::~LinuxSpiDevice()
{
    if (m_spidev != nullptr)
    {
        free(m_spidev);
        m_spidev = nullptr;
    }
    if (m_open)
    {
        close(m_spifd);
    }
}

bool LinuxSpiDevice::set_config(SpiConfig *p_spi_config)
{
    if (p_spi_config != nullptr)
    {
        memcpy(&m_spiconfig, p_spi_config, sizeof(SpiConfig));
        if (m_open)
        {
            /* Set SPI_POL and SPI_PHA */
            if (ioctl(m_spifd, SPI_IOC_WR_MODE, &m_spiconfig.mode) < 0)
            {
                close(m_spifd);
                return false;
            }
            if (ioctl(m_spifd, SPI_IOC_RD_MODE, &m_spiconfig.mode) < 0)
            {
                close(m_spifd);
                return false;
            }

            /* Set bits per word*/
            if (ioctl(m_spifd, SPI_IOC_WR_BITS_PER_WORD, &m_spiconfig.bits_per_word) < 0)
            {
                close(m_spifd);
                return false;
            }
            if (ioctl(m_spifd, SPI_IOC_RD_BITS_PER_WORD, &m_spiconfig.bits_per_word) < 0)
            {
                close(m_spifd);
                return false;
            }

            /* Set SPI speed*/
            if (ioctl(m_spifd, SPI_IOC_WR_MAX_SPEED_HZ, &m_spiconfig.speed) < 0)
            {
                close(m_spifd);
                return false;
            }
            if (ioctl(m_spifd, SPI_IOC_RD_MAX_SPEED_HZ, &m_spiconfig.speed) < 0)
            {
                close(m_spifd);
                return false;
            }
            return true;
        }
    }
    return false;
}

#endif