#include <Arduino.h>

struct batteryCapacity
{
    float voltage;
    uint8_t capacity;
};

const batteryCapacity remainingCapacity[] = {
    {4.20, 100},
    {4.10, 96},
    {4.00, 92},
    {3.96, 89},
    {3.92, 85},
    {3.89, 81},
    {3.86, 77},
    {3.83, 73},
    {3.80, 69},
    {3.77, 65},
    {3.75, 62},
    {3.72, 58},
    {3.70, 55},
    {3.66, 51},
    {3.62, 47},
    {3.58, 43},
    {3.55, 40},
    {3.51, 35},
    {3.48, 32},
    {3.44, 26},
    {3.40, 24},
    {3.37, 20},
    {3.35, 17},
    {3.27, 13},
    {3.20, 9},
    {3.10, 6},
    {3.00, 3}};

const int ncell = sizeof(remainingCapacity) / sizeof(struct batteryCapacity);

uint8_t getBatteryCapacity()
{
    uint32_t Vbatt = 0;
    for (int i = 0; i < 16; i++)
    {
        Vbatt += analogReadMilliVolts(A0); // Read and accumulate ADC voltage
    }
    float Vbattf = 2 * Vbatt / 16 / 1000.0; // Adjust for 1:2 divider and convert to volts
    Serial.printf("Battery voltage: %.2fV\r\n", Vbattf);

    for (int i = 0; i < ncell; i++)
    {
        if (Vbattf > remainingCapacity[i].voltage)
        {
            uint8_t capacity = remainingCapacity[i].capacity;
            Serial.printf("Battery capacity: %u %%\r\n", capacity);
            return capacity;
        }
    }
    Serial.println();
    
    return 0;
}
