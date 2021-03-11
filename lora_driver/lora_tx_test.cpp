#include <iostream>

#include "lora.h"
#include "packet.h"

int main()
{
    int SPI_CHANNEL = 1;
    int CS_PIN = 26;
    int DIO0_PIN = 15;
    int RESET_PIN = 22;
    LoRa lora(SPI_CHANNEL, CS_PIN, DIO0_PIN, RESET_PIN);
    if (!lora.begin())
    {
        std::cout << "LoRa radio failed to start" << std::endl;
    }
    lora.setFrequency(LoRa::FREQ_868)
        ->setTXPower(17)
        ->setSpreadFactor(LoRa::SF_7)
        ->setBandwidth(LoRa::BW_500k)
        ->setCodingRate(LoRa::CR_45)
        ->setSyncWord(0x12)
        ->setHeaderMode(LoRa::HM_IMPLICIT)
        ->enableCRC();
    
    static const char *message = "Hello";
    LoRaPacket p((unsigned char*)message, strlen(message));
    size_t bytes = lora.transmitPacket(&p);
}