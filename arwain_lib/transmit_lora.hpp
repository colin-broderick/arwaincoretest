#ifndef _ARWAIN_TRANSMIT_LORA_HPP
#define _ARWAIN_TRANSMIT_LORA_HPP

void transmit_lora();

namespace arwain
{
	class PosePacket
	{
		public:

			int16_t x = 0; 				// x position * 100
			int16_t y = 0; 				// y position * 100
			int16_t z = 0; 				// z position * 100
			uint8_t alerts = 0; 		// Bit 0: Falling status
										// Bit 1: Entanglement status
										// Bit 2: Attitude (horizontal/vertical)
										// Bit 3: Current stance
										// Bit 4: Current stance
										// Bit 5: Current stance
										// Bit 6:
										// Bit 7:
			uint8_t other = 0;  		// Bit 0: Activity intensity
										// Bit 1: Activity intensity
										// Bit 2: Activity intensity
										// Bit 3:
										// Bit 4:
										// Bit 5:
										// Bit 6:
										// Bit 7:
			uint8_t metadata = 0;		// Full byte, tracking module node ID.
			uint8_t beacon_id = 0;		// Full byte, ID of beacon described in beacon_info
			uint8_t beacon_info = 0;	// Bit 0: Action of [null = 0, new = 1, lost = 2, nearby = 3]
										// Bit 1: Action of [null = 0, new = 1, lost = 2, nearby = 3]
										// Bit 2:
										// Bit 3:
										// Bit 4:
										// Bit 5:
										// Bit 6:
										// Bit 7:
			int8_t partner_distance = -1; // Bit 0-7: Separation in units of 0.5 m.
			
			constexpr static uint8_t beacon_info_mask = 0b11111100;
			constexpr static uint8_t beacon_info_new = 0b00000001;
			constexpr static uint8_t beacon_info_lost = 0b00000010;
			constexpr static uint8_t beacon_info_nearby = 0b00000011;
			
			inline void add_beacon(uint8_t beacon_id_)
			{
				this->beacon_id = beacon_id_;
				this->beacon_info &= beacon_info_mask;
				this->beacon_info |= beacon_info_new;
			}

			inline void drop_beacon(uint8_t beacon_id_)
			{
				this->beacon_id = beacon_id_;
				this->beacon_info &= beacon_info_mask;
				this->beacon_info |= beacon_info_lost;
			}

			inline void nearby_beacon(uint8_t beacon_id_)
			{
				this->beacon_id = beacon_id_;
				this->beacon_info &= beacon_info_mask;
				this->beacon_info |= beacon_info_nearby;
			}
	};

    #if USE_UUBLA == 1
	struct BeaconPacket
	{
		uint8_t beacon_id = 0;
		uint8_t arwain_node_id = 0;
	};
    #endif
}

std::ostream& operator<<(std::ostream& stream, arwain::PosePacket packet);
#if USE_UUBLA == 1
std::ostream& operator<<(std::ostream& stream, arwain::BeaconPacket packet);
void inform_new_uubla_node(const std::string& node_name);
void inform_remove_uubla_node(const std::string& node_name);
#endif

#endif // end include guard
