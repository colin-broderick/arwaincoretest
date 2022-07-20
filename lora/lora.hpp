#ifndef _ARWAIN_LORA_HPP
#define _ARWAIN_LORA_HPP

#include <tuple>
#include <string>
#include <vector>

#include "spi.hpp"

#define ADDRESS_WRITE_MODIFIER	(1<<7)
#define FIFO_ADDRESS						(0x00)
#define OPMODE_ADDRESS					(0x01)
#define FRFMSB_ADDRESS					(0x06)
#define FRFMIB_ADDRESS					(0x07)
#define FRFLSB_ADDRESS					(0x08)
#define PACONFIG_ADDRESS				(0x09)
#define PARAMP_ADDRESS					(0x0a)
#define OCP_ADDRESS							(0x0b)
#define LNA_ADDRESS							(0x0c)
#define FIFOPTR_ADDRESS					(0x0d)
#define FIFOTXBASE_ADDRESS			(0x0e)
#define FIFORXBASE_ADDRESS			(0x0f)
#define FIFORXCURRENT_ADDRESS		(0x10)
#define IRQFLAGSMASK_ADDRESS		(0x11)
#define IRQFLAGS_ADDRESS				(0x12)
#define RXNBBYTES_ADDRESS				(0x13)
#define RXHDRCNTMSB_ADDRESS			(0x14)
#define RXHDRCNTLSB_ADDRESS			(0x15)
#define RXPACKETCNTMSB_ADDRESS	(0x16)
#define RXPACKETCNTLSB_ADDRESS	(0x17)
#define MODEMSTAT_ADDRESS				(0x18)
#define PKTSNR_ADDRESS					(0x19)
#define PKTRSSI_ADDRESS					(0x1a)
#define RSSI_ADDRESS						(0x1b)
#define HOPCHANNEL_ADDRESS			(0x1c)
#define MODEMCONFIG1_ADDRESS		(0x1d)
#define MODEMCONFIG2_ADDRESS		(0x1e)
#define SYMBTIMEOUT_ADDRESS			(0x1f)
#define PREAMBLELENMSB_ADDRESS	(0x20)
#define PREAMBLELENLSB_ADDRESS	(0x21)
#define PAYLOADLEN_ADDRESS			(0x22)
#define MAXPAYLOADLEN_ADDRESS		(0x23)
#define HOPPERIOD_ADDRESS				(0x24)
#define FIFORXBYTEADDR_ADDRESS	(0x25)
#define MODEMCONFIG3_ADDRESS		(0x26)

#define OPMODE_LONGRANGE				(1<<7)
#define OPMODE_ACCESSSHAREDREGS	(1<<6)
#define OPMODE_LOWFREQMODEON		(1<<3)
#define OPMODE_SLEEP						(0)
#define OPMODE_STDBY						(1)
#define OPMODE_FSTX							(2)
#define OPMODE_TX								(3)
#define OPMODE_FSRX							(4)
#define OPMODE_RXCONTINUOUS			(5)
#define OPMODE_RXSINGLE					(6)
#define OPMODE_CAD							(7)

#define PACONFIG_PABOOST				(1<<7)
#define PACONFIG_MAXPWR(x)			((x&7)<<4)
#define PACONFIG_OUTPWR(x)			(x&0xf)

#define PARAMP_3_4MS						(0)
#define PARAMP_2MS							(1)
#define PARAMP_1MS							(2)
#define PARAMP_500US						(3)
#define PARAMP_250US						(4)
#define PARAMP_125US						(5)
#define PARAMP_100US						(6)
#define PARAMP_62US							(7)
#define PARAMP_50US							(8)
#define PARAMP_40US							(9)
#define PARAMP_31US							(10)
#define PARAMP_25US							(11)
#define PARAMP_20US							(12)
#define PARAMP_15US							(13)
#define PARAMP_12US							(14)
#define PARAMP_10US							(15)

#define OCP_ON									(1<<5)
#define OCP_TRIM(x)							(x&0xf)

#define LNA_GAIN_G1							(1<<5)			//MAX
#define LNA_GAIN_G2							(2<<5)
#define LNA_GAIN_G3							(3<<5)
#define LNA_GAIN_G4							(4<<5)
#define LNA_GAIN_G5							(5<<5)
#define LNA_GAIN_G6							(6<<5)			//MIN
#define LNA_BOOST_HF						(3)

#define IRQMASK_RXTIMEOUT				(1<<7)
#define IRQMASK_RXDONE					(1<<6)
#define IRQMASK_PAYLOADCRCERR		(1<<5)
#define IRQMASK_VALIDHDR				(1<<4)
#define IRQMASK_TXDONE					(1<<3)
#define IRQMASK_CADDONE					(1<<2)
#define IRQMASK_FHSSCHANCHANGE	(1<<1)
#define IRQMASK_CADDETECTED			(1<<0)

#define IRQ_RXTIMEOUT						(1<<7)
#define IRQ_RXDONE							(1<<6)
#define IRQ_PAYLOADCRCERR				(1<<5)
#define IRQ_VALIDHDR						(1<<4)
#define IRQ_TXDONE							(1<<3)
#define IRQ_CADDONE							(1<<2)
#define IRQ_FHSSCHANCHANGE			(1<<1)
#define IRQ_CADDETECTED					(1<<0)

#define MODEMSTAT_MODEMCLEAR		(1<<4)
#define MODEMSTAT_HDRINFOVALID	(1<<3)
#define MODEMSTAT_RXONGOING			(1<<2)
#define MODEMSTAT_SIGSYNCED			(1<<1)
#define MODEMSTAT_SIGDET				(1<<0)

#define HOPCHANNEL_PLLTIMEOUT		(1<<7)
#define HOPCHANNEL_RXCRCON			(1<<6)
#define HOPCHANNEL_FHSSCHAN			(0x1f)

#define MODEMCONFIG1_BW_7_8K		(0<<4)
#define MODEMCONFIG1_BW_10_4K		(1<<4)
#define MODEMCONFIG1_BW_15_6K		(2<<4)
#define MODEMCONFIG1_BW_20_8K		(3<<4)
#define MODEMCONFIG1_BW_31_25K	(4<<4)
#define MODEMCONFIG1_BW_41_7K		(5<<4)
#define MODEMCONFIG1_BW_62_5K		(6<<4)
#define MODEMCONFIG1_BW_125K		(7<<4)
#define MODEMCONFIG1_BW_250K		(8<<4)
#define MODEMCONFIG1_BW_500K		(9<<4)
#define MODEMCONFIG1_ECR_4_5		(1<<1)
#define MODEMCONFIG1_ECR_4_6		(2<<1)
#define MODEMCONFIG1_ECR_4_7		(3<<1)
#define MODEMCONFIG1_ECR_4_8		(4<<1)
#define MODEMCONFIG1_IMPLHDR		(1)

#define MODEMCONFIG2_SF(x)			(x<<4)	//MUST be between 6 and 12
#define MODEMCONFIG2_TXCONTMODE	(1<<3)
#define MODEMCONFIG2_RXCRCON		(1<<2)
#define MODEMCONFIG2_SYMTO_98		(3)

#define MODEMCONFIG3_MOBILENODE	(1<<3)
#define MODEMCONFIG3_AGCAUTOON	(1<<2)

class LoRa
{
    public: // Types
		enum class Frequency { FREQ_433, FREQ_868, FREQ_915, FREQ_923 };
		enum class HeaderMode { HM_EXPLICIT, HM_IMPLICIT };
		enum class CodingRate { CR_45 = 1, CR_46, CR_47, CR_48 };
		enum class LNAGain { LNA_G1 = 1, LNA_G2, LNA_G3, LNA_G4, LNA_G5, LNA_G6, LNA_AGC };
		enum class Bandwidth {
			BW_7_8K = MODEMCONFIG1_BW_7_8K, BW_10_4K = MODEMCONFIG1_BW_10_4K, BW_15_6K = MODEMCONFIG1_BW_15_6K, BW_20_8K = MODEMCONFIG1_BW_20_8K,
			BW_31_25K = MODEMCONFIG1_BW_31_25K, BW_41_7K = MODEMCONFIG1_BW_41_7K, BW_62_5K = MODEMCONFIG1_BW_62_5K, BW_125K = MODEMCONFIG1_BW_125K,
			BW_250K = MODEMCONFIG1_BW_250K, BW_500K = MODEMCONFIG1_BW_500K
		};
		enum class SpreadFactor	{
			SF_6 = (6 << 4), SF_7 = (7 << 4), SF_8 = (8 << 4), SF_9 = (9 << 4),
			SF_10 = (10 << 4), SF_11 = (11 << 4), SF_12 = (12 << 4)
		};

	public: // Methods
		LoRa(const std::string& address, const bool as_receiver);
		LoRa(const std::string &address, const bool as_receiver, const LoRa::Frequency frequency_mhz_, const LoRa::Bandwidth bandwidth_khz_, const LoRa::SpreadFactor spread_factor_);
        ~LoRa();
        uint8_t test_chip();
		void send_message(const std::string& message);
		void send_message(uint8_t* message, size_t num_bytes);
		std::tuple<bool, std::string> receive_string(int timeout_ms);
		std::tuple<bool, std::vector<uint8_t>> receive_bytes();

	private: // Methods
        uint8_t read_register(uint8_t address);
        void write_register(uint8_t address, uint8_t val);
		void configure();
		void read_FIFO(uint8_t num_bytes, uint8_t* out_buffer);
		void write_FIFO(const char* str, uint8_t num_bytes);
		bool rx(uint8_t* out_buffer);

    private: // Attributes
		static const int max_message_size = 63;
		bool is_receiver;
		Frequency frequency_mhz;
		Bandwidth bandwidth_khz;
		SpreadFactor spread_factor;
        SPI *spi = nullptr;
        spi_config_t spi_config;		
};

namespace arwain
{
	struct LoraPacket
	{
		int16_t x = 0; // x position * 100
		int16_t y = 0; // y position * 100
		int16_t z = 0; // z position * 100
		uint8_t alerts = 0; // TODO describe
		uint8_t other = 0; // TODO Complete description; bits(0:2)=activity_intensity.
		uint8_t metadata = 0; // TODO describe
	};
}

std::ostream& operator<<(std::ostream& stream, arwain::LoraPacket packet);

#endif
