#include <string>
#include <stdexcept>
#include <iostream>
#include <thread>
#include <chrono>
#include <tuple>

#include "lora.hpp"

LoRa::LoRa(const std::string &address, const bool as_receiver)
{
    // Create SPI device
    spi_config = spi_config_t{0, 8, 1000000, 0};
    spi = new SPI(address.c_str(), &spi_config);
    if (!spi->begin())
    {
        throw std::runtime_error("Error starting SPI device - incorrecct device path?");
    };
    this->configure(868, as_receiver);
}

LoRa::~LoRa()
{
    delete spi;
}

/** \brief Sets radio frequency, bandwidth, spread factor, LNA, PA. */
void LoRa::configure(const int frequency_mhz, const bool rx_only)
{
    uint8_t op_mode = OPMODE_LONGRANGE | OPMODE_SLEEP;

    // Set radio frequency.
    uint32_t FRF_val;
    switch (frequency_mhz)
    {
    case 433:
        FRF_val = 7109363;
        op_mode |= OPMODE_LOWFREQMODEON;
        break;
    case 868:
        FRF_val = 14221348;
        break;
    case 915:
        FRF_val = 14991398;
        break;
    case 923:
        FRF_val = 15122470;
        break;
    }
    uint8_t FRFMSB = ((FRF_val & 0x00FF0000) >> 16);
    int8_t FRFMIB = ((FRF_val & 0x0000FF00) >> 8);
    uint8_t FRFLSB = (FRF_val & 0x000000FF);
    this->write_register(OPMODE_ADDRESS, op_mode);
    this->write_register(FRFMSB_ADDRESS, FRFMSB);
    this->write_register(FRFMIB_ADDRESS, FRFMIB);
    this->write_register(FRFLSB_ADDRESS, FRFLSB);

    // Set bandwidth to 500 KHz.
    uint8_t modem_conf = this->read_register(MODEMCONFIG1_ADDRESS);
    modem_conf &= ~(MODEMCONFIG1_BW_7_8K | MODEMCONFIG1_BW_10_4K | MODEMCONFIG1_BW_15_6K |
                    MODEMCONFIG1_BW_20_8K | MODEMCONFIG1_BW_31_25K | MODEMCONFIG1_BW_41_7K |
                    MODEMCONFIG1_BW_62_5K | MODEMCONFIG1_BW_125K | MODEMCONFIG1_BW_250K | MODEMCONFIG1_BW_500K);
    modem_conf |= MODEMCONFIG1_BW_500K;
    modem_conf &= ~MODEMCONFIG1_IMPLHDR;
    this->write_register(MODEMCONFIG1_ADDRESS, modem_conf);

    // Set spread factor to 12.
    modem_conf = this->read_register(MODEMCONFIG2_ADDRESS);
    modem_conf &= ~(MODEMCONFIG2_SF(6) | MODEMCONFIG2_SF(7) | MODEMCONFIG2_SF(8) | MODEMCONFIG2_SF(9) | MODEMCONFIG2_SF(10) | MODEMCONFIG2_SF(11) | MODEMCONFIG2_SF(12));
    modem_conf |= MODEMCONFIG2_SF(12);
    this->write_register(MODEMCONFIG2_ADDRESS, modem_conf);

    if (rx_only)
    {
        //goto continuous RX
        op_mode &= ~OPMODE_SLEEP;
        op_mode |= OPMODE_RXCONTINUOUS;
    }
    else
    {
        //goto standby mode
        op_mode &= ~OPMODE_SLEEP;
        op_mode |= OPMODE_STDBY;
    }

    // Configure power amplifier.
    uint8_t PA_config = this->read_register(PACONFIG_ADDRESS);
    PA_config |= PACONFIG_PABOOST;
    PA_config |= PACONFIG_MAXPWR(7);
    PA_config |= PACONFIG_OUTPWR(0xf);
    this->write_register(PACONFIG_ADDRESS, PA_config);

    // Enable low noise amplified for high frequencies.
    uint8_t LNA = LNA_GAIN_G1;
    if (frequency_mhz > 525)
    {
        LNA |= LNA_BOOST_HF;
    }
    this->write_register(LNA_ADDRESS, LNA);

    this->write_register(OPMODE_ADDRESS, op_mode);
    this->write_register(IRQFLAGS_ADDRESS, 0xff); //clear flags
}

void LoRa::send_message(uint8_t* message, size_t num_bytes)
{
    uint8_t FIFOTxBase = this->read_register(FIFOTXBASE_ADDRESS); //get FIFO write position

    this->write_register(FIFOPTR_ADDRESS, FIFOTxBase); //set FIFO pointer to write position
    this->write_register(PAYLOADLEN_ADDRESS, num_bytes); //payload is this long
    this->write_FIFO((char*)message, num_bytes); //write data into FIFO
    this->write_register(IRQFLAGS_ADDRESS, 0xff); //clear flags

    // enter TX mode
    uint8_t op_mode = this->read_register(OPMODE_ADDRESS);
    op_mode &= ~(OPMODE_CAD | OPMODE_RXSINGLE | OPMODE_RXCONTINUOUS | OPMODE_FSRX | OPMODE_FSTX | OPMODE_STDBY | OPMODE_SLEEP);
    op_mode |= OPMODE_TX | OPMODE_LONGRANGE;
    this->write_register(OPMODE_ADDRESS, op_mode);

    // Wait until the TX flag is set on.
    uint8_t IRQFlag;
    do
    {
        IRQFlag = this->read_register(IRQFLAGS_ADDRESS);
    } while ((IRQFlag & IRQ_TXDONE) == 0);

    // Clear TX flag.
    this->write_register(IRQFLAGS_ADDRESS, IRQ_TXDONE);
}

/** \brief Transmits arbitrary message encoded as a string. */
void LoRa::send_message(const std::string &message)
{
    send_message((uint8_t*)(message.c_str()), message.size());
}

/** \brief Reads a register which should contain a known value to ensure the chip is detected.
 * Should return 0x1A.
*/
uint8_t LoRa::test_chip()
{
    return read_register(0x02);
}

uint8_t LoRa::read_register(uint8_t address)
{
    uint8_t bf[2] = {address & 0x7F, 0x00};
    spi->xfer(bf, 2, bf, 2);
    return bf[1];
}

void LoRa::write_register(uint8_t address, uint8_t value)
{
    uint8_t bf[2] = {address | 0x80, value};
    spi->write(bf, 2);
}

/** \brief Writes a message to the FIFO. Does not transmit. */
void LoRa::write_FIFO(const char *str, uint8_t num_bytes)
{
    uint8_t bf[LoRa::max_message_size + 1] = {0};
    bf[0] = FIFO_ADDRESS | 0x80;
    for (int i = 0; i < num_bytes && i < max_message_size; i++)
    {
        bf[i + 1] = str[i];
    }
    this->spi->write(bf, num_bytes + 1);
}

void LoRa::read_FIFO(uint8_t num_bytes, uint8_t* out_buffer)
{
    uint8_t addr = FIFO_ADDRESS | ~(0x80);
    this->spi->xfer(&addr, 1, out_buffer, num_bytes);
}

bool LoRa::rx(uint8_t* out_buffer)
{
    uint8_t IRQFlags = this->read_register(IRQFLAGS_ADDRESS);

    if (IRQFlags & IRQMASK_RXDONE)
    {
        this->write_register(IRQFLAGS_ADDRESS, IRQMASK_RXDONE);
        uint8_t num_bytes = this->read_register(RXNBBYTES_ADDRESS);
        this->read_FIFO(num_bytes, out_buffer);
        return true;
    }

    return false;
}

std::tuple<bool, std::string> LoRa::receive()
{
    uint8_t rx_buffer[LoRa::max_message_size] = {0};

    for (int i = 0; i < 100; i++)
    {
        if (rx(rx_buffer))
        {
            return {true, std::string{(char*)rx_buffer}};
        }
        std::this_thread::sleep_for(std::chrono::milliseconds{10});
    }

    return {false, ""};
}
