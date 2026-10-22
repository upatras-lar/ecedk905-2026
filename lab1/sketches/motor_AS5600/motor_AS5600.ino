#include <FOC_lar.h>

// Definitions

#define MOTOR_POLE_PAIRS NOT_SET   // FOC calibration measured approximately that many pole pairs
#define MOTOR_SUPPLY_VOLTAGE 12.0f // motor supply
#define MOTOR_VOLTAGE_LIMIT 3.0f   // maximum target voltage allowed from serial
#define SENSOR_ALIGN_VOLTAGE 1.0f  // voltage used for FOC sensor alignment

// Stored alignment: the shaft does not move at power-up. Leave them as they are to align
// at every power-up. After a normal alignment (shaft free) the summary printed in setup()
// shows the two lines to paste here.
#define STORED_ZERO_ELECTRIC_ANGLE NOT_SET         // e.g. 3.12345f
#define STORED_SENSOR_DIRECTION Direction::UNKNOWN // Direction::CW or Direction::CCW

// SimpleFOCMini v1.0 + Arduino UNO: pole pairs, IN1, IN2, IN3, EN
FOC_lar foc(MOTOR_POLE_PAIRS, 11, 10, 9, 8);

void setup()
{
    Serial.begin(115200);

    // enable velocity EMA filter, setting a value for alpha coefficient
    set_velocity_ema_filter(true, 0.05);

    Serial.println(F(" AS5600 + SimpleFOCMini V1.0 + FOC Voltage + Shaft Monitor :"));

    // sensor + driver + motor init and FOC alignment
    if (!foc.begin(MOTOR_SUPPLY_VOLTAGE, MOTOR_VOLTAGE_LIMIT, SENSOR_ALIGN_VOLTAGE,
                   STORED_ZERO_ELECTRIC_ANGLE, STORED_SENSOR_DIRECTION))
    {
        while (1)
            ;
    }

    foc.printSummary(Serial);
    foc.printHelp(Serial);
}

// MAIN LOOP

void loop()
{
    foc.update(); // FOC algorithm + apply target voltage

    // print at ~10 Hz
    static unsigned long last_print = 0;
    unsigned long now = millis();
    if (foc.isPrinting() && now - last_print >= 100)
    {
        last_print = now;
        Serial.println(foc.getStatusString());
    }

    foc.receiveSerialCommand();
}
