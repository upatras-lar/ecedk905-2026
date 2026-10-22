#include <FOC_lar.h>

// Definitions

#define MOTOR_SUPPLY_VOLTAGE 12.0f
#define MOTOR_VOLTAGE_LIMIT 3.0f
#define SENSOR_ALIGN_VOLTAGE 1.0f

// Voltage used during the pole-pair measurement.
// Start low. Increase to 1.5 or 2 V if the rotor doesn't follow the electrical field.
#define FOC_ESTIMATE_VOLTAGE 1.0f

// The real number of pole pairs is not known yet: start with 1.
// SimpleFOCMini v1.0 + Arduino UNO: pole pairs, IN1, IN2, IN3, EN
FOC_lar foc(1, 11, 10, 9, 8);

void setup()
{
    Serial.begin(115200);

    // Optional V1.0 pseudo-ground. Not needed with a real Arduino GND -> Mini GND wire.
    pinMode(12, OUTPUT);
    digitalWrite(12, LOW);

    Serial.println();
    Serial.println(F("AS5600 + SimpleFOCMini V1.0 : Pole Pair Estimator"));

    // sensor + driver + motor init, without FOC alignment
    if (!foc.beginHardware(MOTOR_SUPPLY_VOLTAGE, MOTOR_VOLTAGE_LIMIT, SENSOR_ALIGN_VOLTAGE))
    {
        while (1)
            ;
    }

    // ONE call: estimates pole pairs and sets motor.pole_pairs / motor.sensor_direction
    PolePairsResult result;
    int pp = foc.findPolePairs(FOC_ESTIMATE_VOLTAGE, &result, &Serial);

    if (pp <= 0)
    {
        while (1)
            ;
    }

    Serial.print(F("Result: "));
    Serial.print(result.pole_pairs);
    Serial.print(F(" pole pairs ("));
    Serial.print(result.poles);
    Serial.println(F(" poles)"));

    // optional: continue with the FOC alignment using the measured values
    if (!foc.alignFOC())
    {
        while (1)
            ;
    }

    foc.printSummary(Serial);
    Serial.println(F("Suggested target voltages: 0.0, 0.2, 0.5, 1.0, 1.5 V"));
    foc.printHelp(Serial);
}

void loop()
{
    foc.update();
    foc.receiveSerialCommand();
}
