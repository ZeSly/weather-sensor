// Copyright Sylvain Girard 2025
// www.zesly.net
//
// This software is a computer program whose purpose is to sedn temperature,
// humidity and pressure measurment to a zigbee network.
//
// This software is governed by the CeCILL-B license under French law and
// abiding by the rules of distribution of free software.  You can  use,
// modify and/ or redistribute the software under the terms of the CeCILL-B
// license as circulated by CEA, CNRS and INRIA at the following URL
// "http://www.cecill.info".
//
// As a counterpart to the access to the source code and  rights to copy,
// modify and redistribute granted by the license, users are provided only
// with a limited warranty  and the software's author,  the holder of the
// economic rights,  and the successive licensors  have only  limited
// liability.
//
// In this respect, the user's attention is drawn to the risks associated
// with loading,  using,  modifying and/or developing or reproducing the
// software by the user in light of its specific status of free software,
// that may mean  that it is complicated to manipulate,  and  that  also
// therefore means  that it is reserved for developers  and  experienced
// professionals having in-depth computer knowledge. Users are therefore
// encouraged to load and test the software's suitability as regards their
// requirements in conditions enabling the security of their systems and/or
// data to be ensured and,  more generally, to use and operate it in the
// same conditions as regards security.
//
// The fact that you are presently reading this means that you have had
// knowledge of the CeCILL-B license and that you accept its terms.
//

#include <Arduino.h>

struct batteryCapacity
{
    float voltage;
    float capacity;
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
    const static int nbMeasure = 32;
    uint32_t Vbatt = 0;
    for (int i = 0; i < nbMeasure; i++)
    {
        Vbatt += analogReadMilliVolts(A0);
    }
    float Vbattf = 2 * Vbatt / nbMeasure / 1000.0 + 0.09; // Adjust for 1:2 divider and convert to volts with offset
    Serial.printf("Battery voltage: %.2fV\r\n", Vbattf);

    if (Vbattf >= remainingCapacity[0].voltage)
    return 100;

    for (int i = 0; i < ncell; i++)
    {
        if (Vbattf > remainingCapacity[i].voltage)
        {
            float vref = remainingCapacity[i].voltage;
            float cref = remainingCapacity[i].capacity;
            float dv = remainingCapacity[i - 1].voltage - vref;
            float dc = remainingCapacity[i - 1].capacity - cref;
            float capacity = (Vbattf - vref) * dc / dv + cref;
            Serial.printf("Battery capacity: %.1f %%\r\n", capacity);
            return static_cast<uint8_t>(capacity);
        }
    }
    Serial.println();

    return 0;
}
