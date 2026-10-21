#ifndef LOADCELL_LAR_H
#define LOADCELL_LAR_H

#include <Arduino.h>
#include <HX711_ADC.h>

// LoadCell_lar
//
// Wrapper around the HX711_ADC library (load cell + HX711 amplifier).
//
// Measurements are returned in grams (once calibrated). Helpers convert to
// force (N) and to torque (N*m) for a lever arm pushing on the load cell.
//
// Typical use:
//
//   LoadCell_lar cell(4, 5);            // DOUT pin, SCK pin
//
//   void setup() {
//       Serial.begin(115200);
//       if (!cell.begin(627.24f)) while (1);    // calibration factor
//   }
//   void loop() {
//       float grams;
//       if (cell.read(grams)) Serial.println(grams);   // non-blocking
//   }
//
// Note: HX711 at 10 SPS gives a new sample every 100 ms, and getData() is a
// moving average over the last samples (default 16 + 2 ignored, see the
// HX711_ADC config.h): after a change of the load, wait about 2 s to have
// a settled value.

#define LOADCELL_LAR_GRAVITY 9.80665f // m/s^2

class LoadCell_lar
{
public:
    LoadCell_lar(uint8_t dout_pin, uint8_t sck_pin);

    // Starts the HX711, waits stabilizing_ms (+400 ms) and optionally tares.
    // Returns false on timeout (check wiring and pins).
    // Blocking during startup only.
    bool begin(float cal_factor = 1.0f,
               unsigned long stabilizing_ms = 2000,
               bool do_tare = true,
               Print &log = Serial);

    // Non-blocking. Call on every loop() iteration.
    // Returns true when a new conversion was read out.
    bool update(void);

    // update() + value: returns true (and sets grams) only when a new
    // sample is available.
    bool read(float &grams);

    // Filtered value (moving average) in grams: last available data
    float getGrams(void);

    // Same value converted to force (N): grams / 1000 * g
    float getForceN(void);

    // Torque (N*m) = force (N) * arm length (m).
    // The force must be perpendicular to the lever arm; arm length is measured
    // from the shaft axis to the contact point on the load cell.
    float getTorque(float arm_length_m);

    // Tare (zero)
    void tare(void);        // blocking (~ 2 s)
    void tareNoDelay(void); // non-blocking: keep calling update()
    bool isTareDone(void);  // true ONCE after tareNoDelay() completes

    // Calibration factor: raw data is divided by this value to obtain grams
    void setCalFactor(float cal_factor);
    float getCalFactor(void);

    // Calibration with a known mass (g) placed on the load cell (after tare).
    // Blocking (~ 2 s: refills the averaging dataset). Sets and returns the
    // new calibration factor, or 0 if known_mass_g is 0 (nothing changed).
    float calibrate(float known_mass_g);

    // Persist the calibration factor in EEPROM (AVR, ESP8266, ESP32)
    // Returns false if EEPROM is not available on this platform.
    bool saveCalFactor(int eeprom_address = 0);

    // Loads the factor from EEPROM. Returns true and applies it only if the
    // stored value is valid (finite, non-zero); otherwise nothing is changed.
    bool loadCalFactor(int eeprom_address = 0);

    // Options
    void setReverseOutput(void);       // invert the sign of the measurement
    void setSamplesInUse(int samples); // fewer samples = faster, noisier
    int getSamplesInUse(void);

    // True if the HX711 stopped answering (timeouts)
    bool hasError(void);

    // ---- Serial helpers -------------------------------------------------
    //
    // Ready-made interactive tools on a serial port (Serial by default).
    // The ones that WAIT for the user keep servicing the HX711, but they
    // block loop(): do not use them while a motor loop is running.

    // Waits until one of the characters in 'accepted' is received and returns it.
    // Spaces and newlines are ignored. Blocking.
    char waitForKey(const char *accepted, Stream &serial = Serial);

    // Waits for a line containing a non-zero number and returns it. Blocking.
    float waitForNumber(Stream &serial = Serial);

    // Asks "save to EEPROM? y/n" and saves the calibration factor on 'y'.
    // Returns true if it was saved.
    bool askToSave(int eeprom_address = 0, Stream &serial = Serial);

    // Guided calibration: asks for tare, then the known mass (g), calibrates,
    // then asks whether to save. Returns the new calibration factor.
    float calibrateInteractive(int eeprom_address = 0, Stream &serial = Serial);

    // Shows the current factor, asks for a new one, applies it, asks to save.
    void changeCalFactorInteractive(int eeprom_address = 0, Stream &serial = Serial);

    // Prints the command list handled by handleSerialCommands()
    void printHelp(Print &out = Serial);

    // Prints one line: mass (g), force (N) and torque (mN*m) for the given arm
    void printReading(float arm_length_m, Print &out = Serial);

    // Non-blocking: reads one pending command character (if any) and runs it.
    //   t : tare              r : guided calibration
    //   c : manual factor     s : save to EEPROM      l : load from EEPROM
    //   p : pause/resume printing (see isPrinting())   h : help
    // Newlines and unknown characters are ignored. Also reports "Tare complete."
    void handleSerialCommands(int eeprom_address = 0, Stream &serial = Serial);

    // Printing state toggled by the 'p' command (starts enabled)
    bool isPrinting(void);
    void setPrinting(bool enabled);

private:
    HX711_ADC _cell;
    float _last_grams;
    bool _printing;
};

#endif
