#include <AS5600_lar.h>

// Definitions

#define PRINT_INTERVAL_MS 100   // print period (10 Hz)
#define VELOCITY_EMA_ALPHA 0.05 // velocity filter coefficient (1.0 = no filtering)

double position_zero = 0.0; // reporting zero position (rad), set with 'z'
bool printing = true;       // 'p' toggles the periodic output

void printHelp()
{
    Serial.println(F("***"));
    Serial.println(F("z : set the current position as zero"));
    Serial.println(F("p : pause / resume printing"));
    Serial.println(F("h : this help"));
    Serial.println(F("***"));
}

void setup()
{
    Serial.begin(115200);
    Serial.println();
    Serial.println(F("AS5600 magnetic sensor: read"));

    // enable the velocity EMA filter
    set_velocity_ema_filter(true, VELOCITY_EMA_ALPHA);

    // start the I2C bus and the sensor
    if (!AS5600_lar_begin())
    {
        Serial.println(F("ERROR: Could not read AS5600, check wiring."));
        while (1)
            ;
    }

    print_sensor_info(Serial);
    print_velocity_ema_settings(Serial);
    printHelp();
}

void loop()
{
    // must run as often as possible: updates angle, turns and velocity
    bool read_ok = update_sensor();

    // print at ~10 Hz
    static unsigned long last_print = 0;
    unsigned long now = millis();
    if (printing && now - last_print >= PRINT_INTERVAL_MS)
    {
        last_print = now;

        if (read_ok)
        {
            print_sensor_status(Serial, position_zero);
        }
        else
        {
            Serial.println(F("ERROR: AS5600 read failed."));
        }
    }

    while (Serial.available())
    {
        char c = (char)Serial.read();

        if (c == 'z')
        {
            position_zero = get_position();
            Serial.println(F("Position zeroed."));
        }
        else if (c == 'p')
        {
            printing = !printing;
            Serial.println(printing ? F("#printing resumed") : F("#printing paused"));
        }
        else if (c == 'h')
        {
            printHelp();
        }
    }
}
