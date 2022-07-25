#ifndef _ARWAIN_TRANSMIT_LORA_HPP
#define _ARWAIN_TRANSMIT_LORA_HPP

#include "build_config.hpp"

void transmit_lora();

namespace arwain
{
	struct PosePacket
	{
		int16_t x = 0; // x position * 100
		int16_t y = 0; // y position * 100
		int16_t z = 0; // z position * 100
		uint8_t alerts = 0; // TODO describe
		uint8_t other = 0; // TODO Complete description; bits(0:2)=activity_intensity.
		uint8_t metadata = 0; // TODO describe
	};

    #if USE_UUBLA == 1
	struct BeaconPacket
	{
		int16_t x = 0; // x position * 100 // These are the position of the tracking module at the time the beacon is registered.
		int16_t y = 0; // y position * 100
		int16_t z = 0; // z position * 100
		uint8_t id = 0;
		uint8_t node_id = 0;
	};
    #endif
}

std::ostream& operator<<(std::ostream& stream, arwain::PosePacket packet);
#if USE_UUBLA == 1
std::ostream& operator<<(std::ostream& stream, arwain::BeaconPacket packet);
#endif

#endif
