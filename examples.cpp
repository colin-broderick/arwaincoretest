#include <string>

#include "mbed.h"
#include "LSM6DS33.h"

#define BLINKING_RATE 200ms

void led_example()
{
    // Initialise the digital pin LED1 as an output
    DigitalOut led1(LED1);
    DigitalOut led2(LED2);
    DigitalOut led3(LED3);
    DigitalOut led4(LED4);
    
    led1 = !led1;

    while (true) {      

        ThisThread::sleep_for(BLINKING_RATE);
        led1 = !led1;
        led2 = !led2;
        ThisThread::sleep_for(BLINKING_RATE);
        led2 = !led2;
        led3 = !led3;
        ThisThread::sleep_for(BLINKING_RATE);
        led3 = !led3;
        led4 = !led4;
        ThisThread::sleep_for(BLINKING_RATE);
        led4 = !led4;
        led3 = !led3;
        ThisThread::sleep_for(BLINKING_RATE);
        led3 = !led3;
        led2 = !led2;
        ThisThread::sleep_for(BLINKING_RATE);
        led2 = !led2;
        led1 = !led1;
    }
}


void serial_example()
{
    BufferedSerial serial(USBTX, USBRX);
    serial.set_baud(9600);

    char str[20];
    str[0] = 'l';
    str[1] = 'a';
    str[2] = 'p';
    str[3] = ' ';
    str[5] = '\n';
    int ticker = 0;

    while (true)
    {      
        str[4] = to_string(ticker)[0];
        serial.write(str, 12);
        ThisThread::sleep_for(BLINKING_RATE);
        ticker++;
    }
}

// Uses the onboard LEDs to display the numbers 0-15 in binary.
void led_binary(unsigned int binary)
{
    DigitalOut led1(LED1);
    DigitalOut led2(LED2);
    DigitalOut led3(LED3);
    DigitalOut led4(LED4);

    led1 = binary & 0x01;
    led2 = binary & 0x02;
    led3 = binary & 0x03;
    led4 = binary & 0x04;

    ThisThread::sleep_for(5000ms);

    for (int i = 0; i < 16; i++)
    {
        binary++;
        led1 = binary & 0x01;
        led2 = (binary >> 1) & 0x01;
        led3 = (binary >> 2) & 0x01;
        led4 = (binary >> 3) & 0x01;
        ThisThread::sleep_for(2000ms);
    }
}

void i2c_example(int address)
{
    LSM6DS33 sensor(p28, p27, address);
    uint8_t response = sensor.begin(LSM6DS33::G_SCALE_245DPS, LSM6DS33::A_SCALE_16G, LSM6DS33::G_ODR_104, LSM6DS33::A_ODR_104);

    BufferedSerial serial(USBTX, USBRX);
    serial.set_baud(9600);

    char ans[5] = {0};
    ans[0] = to_string(response)[0];
    ans[1] = to_string(response)[1];
    ans[2] = to_string(response)[2];
    ans[3] = '\n';

    serial.write(ans, 4);

    while (1)
    {
        ThisThread::sleep_for(20ms);
        sensor.readAccel();
        ThisThread::sleep_for(20ms);        

        serial.write(to_string((int)(sensor.ax*100)).c_str(), 10);
        serial.write("\n", 1);
        
    }
}