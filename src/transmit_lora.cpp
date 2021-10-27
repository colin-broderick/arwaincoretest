/** \brief Forms and transmits LoRa messages on a loop.
 */
void transmit_lora()
{
    // if (NO_LORA)
    // {
    //     return;
    // }

    // // Turn on the LoRa radio and quit if it fails.
    // int SPI_CHANNEL = 1;
    // int CS_PIN = 26;
    // int DIO0_PIN = 15;
    // int RESET_PIN = 22;
    // LoRa lora(SPI_CHANNEL, CS_PIN, DIO0_PIN, RESET_PIN);
    // if (!lora.begin())
    // {
    //     std::cout << "LoRa radio failed to start" << std::endl;
    //     SHUTDOWN = 1;
    //     return;
    // }
    // else
    // {
    //     std::cout << "LoRa radio started successfully" << std::endl;
    // }

    // // Configura LoRa radio.
    // lora.setFrequency(CONFIG.lora_rf_frequency);
    // lora.setTXPower(CONFIG.lora_tx_power);
    // lora.setSpreadFactor(CONFIG.lora_spread_factor);
    // lora.setBandwidth(CONFIG.lora_bandwidth);
    // lora.setCodingRate(CONFIG.lora_coding_rate);
    // lora.setHeaderMode(CONFIG.lora_header_mode);
    // // lora.setSyncWord(0x12);
    // if (CONFIG.lora_enable_crc)
    // {
    //     // lora.enableCRC();
    // }

    // // TEST Check the radio is set up correctly.
    // // std::cout << lora.getTXPower() << "\n";
    // // printf("%d\n", lora.getTXPower());
    // // std::cout << lora.getFrequency() << "\n";
    // // std::cout << lora.getSpreadFactor() << "\n";
    // // std::cout << lora.getBandwidth() << "\n";
    // // std::cout << lora.getCodingRate() + 4 << "\n";
    // // // std::cout << lora.getSyncWord() << "\n";
    // // printf("%d\n", lora.getSyncWord());
    // // std::cout << lora.getHeaderMode() << std::endl;

    // // Local buffers.
    // arwain::Logger lora_file;
    // std::array<double, 3> position;
    // uint16_t alerts;
    // // FLOAT16 x16, y16, z16;

    // // Set up timing.
    // auto time = std::chrono::system_clock::now();
    // std::chrono::milliseconds interval{arwain::Intervals::LORA_TRANSMISSION_INTERVAL};

    // // Open file handles for data logging.
    // if (LOG_TO_FILE)
    // {
    //     lora_file.open(arwain::folder_date_string + "/lora_log.txt");
    //     lora_file << "# time packet" << "\n";
    // }

    // int xa = 0;

    // uint64_t testval = 12345678910;
    // while (!SHUTDOWN)
    // {
    //     { // Get positions as float16.
    //         std::lock_guard<std::mutex> lock{POSITION_BUFFER_LOCK};
    //         position = POSITION_BUFFER.back();
    //     }
    //     // std::cout << xa << "\n";
    //     position[0] = xa;
    //     position[1] = 3.5;
    //     position[2] = 1.5;
    //     FLOAT16 x16{position[0]};
    //     FLOAT16 y16{position[1]};
    //     FLOAT16 z16{position[2]};
    //     xa++;
    //     // Create alerts flags.
    //     alerts = 0;
    //     alerts |= STATUS.falling;
    //     alerts |= (STATUS.entangled << 1) ;
    //     alerts |= (STATUS.current_stance << 2);

    //     // Reset critical status flags now they have been read.
    //     STATUS.falling = arwain::StanceDetector::NotFalling;
    //     STATUS.entangled = arwain::StanceDetector::NotEntangled;

    //     // TODO Build packet for transmission.
    //     char message[LORA_MESSAGE_LENGTH];


    //     memcpy(message, &testval, sizeof(uint64_t));
    //     testval += 1;
    //     // Copy the float16 position values and the alert flags into the buffer.
    //     // memcpy(message, &(x16.m_uiFormat), sizeof(x16.m_uiFormat));
    //     // memcpy(message + 2, &(y16.m_uiFormat), sizeof(x16.m_uiFormat));
    //     // memcpy(message + 4, &(z16.m_uiFormat), sizeof(x16.m_uiFormat));
    //     // memcpy(message + 6, &alerts, sizeof(alerts));

    //     // This block tests recovery of the float.
    //     // FLOAT16 f;
    //     // memcpy(&(f.m_uiFormat), message, 2);
    //     // float g = FLOAT16::ToFloat32(f);
    //     // std::cout << g << std::endl;

    //     // Send transmission.
    //     LoRaPacket packet{(unsigned char *)message, LORA_MESSAGE_LENGTH};
    //     lora.transmitPacket(&packet);

    //     // TODO: Log LoRa transmission to file, including any success/signal criteria that might be available.
    //     // TODO: Currently logging binary nonsense. Fix.
    //     if (LOG_TO_FILE)
    //     {
    //         lora_file << time.time_since_epoch().count() << " " << message << "\n";
    //     }

    //     // Wait until next tick
    //     time = time + interval;
    //     std::this_thread::sleep_until(time);
    // }

    // // Close log file handle.
    // if (LOG_TO_FILE)
    // {
    //     lora_file.close();
    // }
}
