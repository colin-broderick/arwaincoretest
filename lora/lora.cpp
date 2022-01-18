#include <string>
#include <stdexcept>
#include <iostream>
#include <thread>
#include <chrono>
#include <tuple>
#include <vector>

#include "lora.hpp"

LoRa::LoRa(const std::string &address, const bool as_receiver, const LoRa::Frequency frequency_mhz_, const LoRa::Bandwidth bandwidth_khz_, const LoRa::SpreadFactor spread_factor_)
: is_receiver(as_receiver), frequency_mhz(frequency_mhz_), bandwidth_khz(bandwidth_khz_), spread_factor(spread_factor_)
{
    // Create SPI device
    spi_config = spi_config_t{0, 8, 1000000, 0};
    spi = new SPI(address.c_str(), &spi_config);
    if (!spi->begin())
    {
        throw std::runtime_error("Error starting SPI device - incorrecct device path?");
    };
    this->configure();
}

LoRa::LoRa(const std::string &address, const bool as_receiver)
: LoRa(address, as_receiver, LoRa::Frequency::FREQ_868, LoRa::Bandwidth::BW_500K, LoRa::SpreadFactor::SF_12)
{
}

LoRa::~LoRa()
{
    delete spi;
}

/** \brief Sets radio frequency, bandwidth, spread factor, LNA, PA. */
void LoRa::configure()
{
    uint8_t op_mode = OPMODE_LONGRANGE | OPMODE_SLEEP;

    // Set radio frequency.
    uint32_t FRF_val;
    switch (this->frequency_mhz)
    {
    case Frequency::FREQ_433:
        FRF_val = 7109363;
        op_mode |= OPMODE_LOWFREQMODEON;
        break;
    case Frequency::FREQ_868:
        FRF_val = 14221348;
        break;
    case Frequency::FREQ_915:
        FRF_val = 14991398;
        break;
    case Frequency::FREQ_923:
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

    // Set bandwidth to specified option.
    uint8_t modem_conf = this->read_register(MODEMCONFIG1_ADDRESS);
    modem_conf &= ~((uint8_t)(LoRa::Bandwidth::BW_7_8K) | (uint8_t)(LoRa::Bandwidth::BW_10_4K) |
                    (uint8_t)(LoRa::Bandwidth::BW_15_6K) | (uint8_t)(LoRa::Bandwidth::BW_20_8K) |
                    (uint8_t)(LoRa::Bandwidth::BW_31_25K) | (uint8_t)(LoRa::Bandwidth::BW_41_7K) |
                    (uint8_t)(LoRa::Bandwidth::BW_62_5K) | (uint8_t)(LoRa::Bandwidth::BW_125K) |
                    (uint8_t)(LoRa::Bandwidth::BW_250K) | (uint8_t)(LoRa::Bandwidth::BW_500K));
    modem_conf |= (uint8_t)(this->bandwidth_khz);
    modem_conf &= ~MODEMCONFIG1_IMPLHDR;
    this->write_register(MODEMCONFIG1_ADDRESS, modem_conf);

    // Set spread factor to 12.
    modem_conf = this->read_register(MODEMCONFIG2_ADDRESS);
    modem_conf &= ~(MODEMCONFIG2_SF(6) | MODEMCONFIG2_SF(7) | MODEMCONFIG2_SF(8) | MODEMCONFIG2_SF(9) | MODEMCONFIG2_SF(10) | MODEMCONFIG2_SF(11) | MODEMCONFIG2_SF(12));
    modem_conf |= (uint8_t)(this->spread_factor);
    // Always enable CRC
    modem_conf |= MODEMCONFIG2_RXCRCON;
    this->write_register(MODEMCONFIG2_ADDRESS, modem_conf);

    if (this->is_receiver)
    {
        // Enter continuous RX mode.
        op_mode &= ~OPMODE_SLEEP;
        op_mode |= OPMODE_RXCONTINUOUS;
    }
    else
    {
        // Enter standby mode.
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
    if (frequency_mhz != Frequency::FREQ_433)
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

    uint8_t previous_op_mode = op_mode;
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

    // Set the mode back to what it was before.
    this->write_register(OPMODE_ADDRESS, previous_op_mode);
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
    uint8_t bf[2] = {(uint8_t)(address & 0x7F), 0x00};
    spi->xfer(bf, 2, bf, 2);
    return bf[1];
}

void LoRa::write_register(uint8_t address, uint8_t value)
{
    uint8_t bf[2] = {(uint8_t)(address | 0x80), value};
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
    uint8_t read_addr = this->read_register(FIFORXBYTEADDR_ADDRESS) - num_bytes;
    this->write_register(FIFOPTR_ADDRESS, read_addr);

    uint8_t fifo_addr = FIFO_ADDRESS | 0x7F;
    this->spi->write(&fifo_addr, 1);
    this->spi->read(out_buffer, num_bytes + 1);
}

bool LoRa::rx(uint8_t* out_buffer)
{
    uint8_t IRQFlags = this->read_register(IRQFLAGS_ADDRESS);

    if ((IRQFlags & IRQMASK_RXDONE) && !(IRQFlags & IRQ_PAYLOADCRCERR))
    {
        this->write_register(IRQFLAGS_ADDRESS, IRQMASK_RXDONE);
        uint8_t num_bytes = this->read_register(RXNBBYTES_ADDRESS);
        this->read_FIFO(num_bytes, out_buffer);
        return true;
    }
    else if (IRQFlags & (IRQMASK_RXDONE | IRQ_PAYLOADCRCERR))
    {
        this->write_register(IRQFLAGS_ADDRESS, IRQMASK_RXDONE | IRQ_PAYLOADCRCERR);
    }

    return false;
}

/** \brief Spends 100 iterations of 10 ms checking for a received message.
 * TODO The current timeout mechanism is very approximate. It takes roughly 10 ms per iteration,
 * until the appropriate number of iterations is met. But obviously the sleep doesn't account for
 * all the time spent, so each spin is really 10.something ms. Improve the timing.
 */
std::tuple<bool, std::string> LoRa::receive_string(int timeout_ms)
{
    auto start_time = std::chrono::high_resolution_clock::now();
    auto end_time = start_time + std::chrono::milliseconds{timeout_ms - 10};

    while (std::chrono::high_resolution_clock::now() < end_time)
    {
        uint8_t rx_buffer[LoRa::max_message_size] = {0};
        if (rx(rx_buffer))
        {
            return {true, std::string{(char*)(&(rx_buffer[1]))}};
        }
        std::this_thread::sleep_for(std::chrono::milliseconds{10});
    }

    return {false, ""};
}

std::tuple<bool, std::vector<uint8_t>> LoRa::receive_bytes()
{
    std::vector<uint8_t> rx_msg;
    for (int i = 0; i < 100; i++)
    {
        uint8_t rx_buffer[LoRa::max_message_size] = {0};
        if (rx(rx_buffer))
        {
            for (int j = 1; j < LoRa::max_message_size; j++)
            {
                rx_msg.push_back(rx_buffer[i]);
            }
            return {true, rx_msg};
        }
        std::this_thread::sleep_for(std::chrono::milliseconds{10});
    }

    return {false, rx_msg};
}

std::ostream& operator<<(std::ostream& stream, arwain::LoraPacket packet)
{
    stream << packet.x << " " << packet.y << " " << packet.z << " " << packet.alerts;
    return stream;
}
