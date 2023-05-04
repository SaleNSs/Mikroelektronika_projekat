#include "ds18b20.h"

#define TEMP_BUS (gpio_num_t)4u
DeviceAddress tempSensors[NUMBER_OF_CONNECTED_DEVICES];

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println("Initializing 1-Wire bus...");
    ds18b20_init(TEMP_BUS);
    Serial.println("Initializing complete");
    delay(250);
    
    Serial.println("Detecting sensors on the 1-Wire bus...");
    getTempAddresses(tempSensors);
    Serial.println("Sensors detected");

    Serial.println("Setting resolution for all sensors to 12bit...");
    ds18b20_setResolution(tempSensors, NUMBER_OF_CONNECTED_DEVICES, 12);
    Serial.println("Resolution is set");

    Serial.println("Reading temperature");
}

void loop()
{
    ds18b20_requestTemperatures();

    Serial.println();
    for (uint8_t i = 0; i < NUMBER_OF_CONNECTED_DEVICES; i++)
    {
        Serial.print("Device ");
        Serial.print(i + 1);
        float temp = ds18b20_getTempC((DeviceAddress *)tempSensors[i]);
        Serial.print(". temperature is: ");
        Serial.println(temp);
    }
    
    delay(1500);
}