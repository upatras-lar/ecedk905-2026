#include <LoadCell_lar.h>


// Definitions

#define HX711_DOUT_PIN 4            // MCU > HX711 DOUT
#define HX711_SCK_PIN 5             // MCU > HX711 SCK
#define DEFAULT_CAL_FACTOR 650.00f  // used if no valid value is stored in EEPROM
#define EEPROM_CAL_ADDRESS 0        // EEPROM address of the calibration factor
#define ROD_LENGTH_M 0.100f         // lever arm: shaft axis -> load cell contact point (m)


LoadCell_lar cell(HX711_DOUT_PIN, HX711_SCK_PIN);


void setup()
{
    Serial.begin(115200);
    Serial.println();
    Serial.println(F("HX711 load cell: calibration and reading"));

    // start, stabilize and tare (keep the rod free during startup)
    if (!cell.begin(DEFAULT_CAL_FACTOR))
    {
        while (1);
    }

    if (cell.loadCalFactor(EEPROM_CAL_ADDRESS))
    {
        Serial.print(F("Calibration factor loaded from EEPROM: "));
    }
    else
    {
        Serial.print(F("Using default calibration factor: "));
    }
    Serial.println(cell.getCalFactor(), 4);

    cell.printHelp();
}


void loop()
{
    float grams;

    // non-blocking: true only when a new sample is available (~10 Hz)
    if (cell.read(grams) && cell.isPrinting())
    {
        cell.printReading(ROD_LENGTH_M);
    }

    // serial commands: t, r, c, s, l, p, h
    cell.handleSerialCommands(EEPROM_CAL_ADDRESS);
}
