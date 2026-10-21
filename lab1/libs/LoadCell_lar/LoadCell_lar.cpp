#include "LoadCell_lar.h"

#if defined(ESP8266) || defined(ESP32) || defined(AVR)
#include <EEPROM.h>
#define LOADCELL_LAR_HAS_EEPROM 1
#endif

LoadCell_lar::LoadCell_lar(uint8_t dout_pin, uint8_t sck_pin)
    : _cell(dout_pin, sck_pin),
      _last_grams(0.0f),
      _printing(true)
{
}

bool LoadCell_lar::begin(float cal_factor,
                         unsigned long stabilizing_ms,
                         bool do_tare,
                         Print &log)
{
    _cell.begin();
    _cell.start(stabilizing_ms, do_tare);

    if (_cell.getTareTimeoutFlag() || _cell.getSignalTimeoutFlag())
    {
        log.println(F("ERROR: HX711 timeout, check MCU>HX711 wiring and pin designations."));
        return false;
    }

    _cell.setCalFactor(cal_factor);

    // fill the dataset so getGrams() is valid right away
    while (!_cell.update())
        ;
    _last_grams = _cell.getData();

    return true;
}

bool LoadCell_lar::update(void)
{
    if (_cell.update())
    {
        _last_grams = _cell.getData();
        return true;
    }
    return false;
}

bool LoadCell_lar::read(float &grams)
{
    if (update())
    {
        grams = _last_grams;
        return true;
    }
    return false;
}

float LoadCell_lar::getGrams(void)
{
    return _last_grams;
}

float LoadCell_lar::getForceN(void)
{
    return _last_grams / 1000.0f * LOADCELL_LAR_GRAVITY;
}

float LoadCell_lar::getTorque(float arm_length_m)
{
    return getForceN() * arm_length_m;
}

void LoadCell_lar::tare(void)
{
    _cell.tare();
}

void LoadCell_lar::tareNoDelay(void)
{
    _cell.tareNoDelay();
}

bool LoadCell_lar::isTareDone(void)
{
    return _cell.getTareStatus();
}

void LoadCell_lar::setCalFactor(float cal_factor)
{
    _cell.setCalFactor(cal_factor);
}

float LoadCell_lar::getCalFactor(void)
{
    return _cell.getCalFactor();
}

float LoadCell_lar::calibrate(float known_mass_g)
{
    if (known_mass_g <= 0.0f)
    {
        return 0.0f;
    }

    _cell.refreshDataSet();                                   // be sure the known mass fills the whole dataset
    float new_factor = _cell.getNewCalibration(known_mass_g); // also applies it
    _last_grams = _cell.getData();

    return new_factor;
}

bool LoadCell_lar::saveCalFactor(int eeprom_address)
{
#ifdef LOADCELL_LAR_HAS_EEPROM
    float value = _cell.getCalFactor();
#if defined(ESP8266) || defined(ESP32)
    EEPROM.begin(512);
#endif
    EEPROM.put(eeprom_address, value);
#if defined(ESP8266) || defined(ESP32)
    EEPROM.commit();
#endif
    return true;
#else
    (void)eeprom_address;
    return false;
#endif
}

bool LoadCell_lar::loadCalFactor(int eeprom_address)
{
#ifdef LOADCELL_LAR_HAS_EEPROM
    float value = 0.0f;
#if defined(ESP8266) || defined(ESP32)
    EEPROM.begin(512);
#endif
    EEPROM.get(eeprom_address, value);

    if (isnan(value) || isinf(value) || value == 0.0f)
    {
        return false; // empty / corrupted EEPROM
    }

    _cell.setCalFactor(value);
    return true;
#else
    (void)eeprom_address;
    return false;
#endif
}

void LoadCell_lar::setReverseOutput(void)
{
    _cell.setReverseOutput();
}

void LoadCell_lar::setSamplesInUse(int samples)
{
    // HX711_ADC ignores values above its compiled maximum, but a negative value
    // makes its internal shift loop spin forever. Clamp to a safe value.
    if (samples < 0)
    {
        samples = 0;
    }
    _cell.setSamplesInUse(samples);
}

int LoadCell_lar::getSamplesInUse(void)
{
    return _cell.getSamplesInUse();
}

bool LoadCell_lar::hasError(void)
{
    return _cell.getTareTimeoutFlag() || _cell.getSignalTimeoutFlag();
}

// ---- Serial helpers ------------------------------------------------------

char LoadCell_lar::waitForKey(const char *accepted, Stream &serial)
{
    while (true)
    {
        update();

        if (serial.available())
        {
            char c = (char)serial.read();

            if (c == '\n' || c == '\r' || c == ' ')
            {
                continue;
            }
            if (strchr(accepted, c) != NULL)
            {
                return c;
            }
        }
    }
}

float LoadCell_lar::waitForNumber(Stream &serial)
{
    while (true)
    {
        update();

        if (serial.available())
        {
            String line = serial.readStringUntil('\n');
            line.trim();

            float value = line.toFloat();
            if (line.length() > 0 && value != 0.0f)
            {
                return value;
            }
        }
    }
}

bool LoadCell_lar::askToSave(int eeprom_address, Stream &serial)
{
    serial.print(F("Save this value to EEPROM address "));
    serial.print(eeprom_address);
    serial.println(F("? y/n"));

    if (waitForKey("yn", serial) == 'y')
    {
        if (saveCalFactor(eeprom_address))
        {
            serial.println(F("Value saved."));
            return true;
        }
        serial.println(F("EEPROM not available."));
        return false;
    }

    serial.println(F("Value not saved."));
    return false;
}

float LoadCell_lar::calibrateInteractive(int eeprom_address, Stream &serial)
{
    serial.println(F("***"));
    serial.println(F("Start calibration:"));
    serial.println(F("Remove any load (rod resting free), then send 't' to tare."));

    waitForKey("t", serial);
    tareNoDelay();
    while (!isTareDone())
    {
        update();
    }
    serial.println(F("Tare complete."));

    serial.println(F("Now place your known mass on the load cell."));
    serial.println(F("Then send its mass in grams (i.e. 100.0)."));

    float known_mass = waitForNumber(serial);
    serial.print(F("Known mass is: "));
    serial.println(known_mass);
    serial.println(F("Measuring..."));

    float new_factor = calibrate(known_mass);

    serial.print(F("New calibration factor: "));
    serial.println(new_factor, 4);

    askToSave(eeprom_address, serial);

    serial.println(F("End calibration."));
    serial.println(F("***"));

    return new_factor;
}

void LoadCell_lar::changeCalFactorInteractive(int eeprom_address, Stream &serial)
{
    serial.println(F("***"));
    serial.print(F("Current value is: "));
    serial.println(getCalFactor(), 4);
    serial.println(F("Now send the new value, i.e. 696.0"));

    float new_factor = waitForNumber(serial);
    setCalFactor(new_factor);

    serial.print(F("New calibration factor: "));
    serial.println(new_factor, 4);

    askToSave(eeprom_address, serial);
    serial.println(F("***"));
}

void LoadCell_lar::printHelp(Print &out)
{
    out.println(F("***"));
    out.println(F("t : tare"));
    out.println(F("r : calibrate with a known mass"));
    out.println(F("c : enter the calibration factor manually"));
    out.println(F("s : save calibration factor to EEPROM"));
    out.println(F("l : load calibration factor from EEPROM"));
    out.println(F("p : pause / resume printing"));
    out.println(F("h : this help"));
    out.println(F("***"));
}

void LoadCell_lar::printReading(float arm_length_m, Print &out)
{
    out.print(F("Mass = "));
    out.print(getGrams(), 2);
    out.print(F(" g | Force = "));
    out.print(getForceN(), 4);
    out.print(F(" N | Torque = "));
    out.print(getTorque(arm_length_m) * 1000.0f, 2);
    out.println(F(" mN*m"));
}

void LoadCell_lar::handleSerialCommands(int eeprom_address, Stream &serial)
{
    if (isTareDone())
    {
        serial.println(F("Tare complete."));
    }

    if (!serial.available())
    {
        return;
    }

    char c = (char)serial.read();

    switch (c)
    {
    case 't':
        tareNoDelay();
        break;
    case 'r':
        calibrateInteractive(eeprom_address, serial);
        break;
    case 'c':
        changeCalFactorInteractive(eeprom_address, serial);
        break;
    case 's':
        serial.println(saveCalFactor(eeprom_address) ? F("Saved.") : F("EEPROM not available."));
        break;
    case 'l':
        serial.println(loadCalFactor(eeprom_address) ? F("Loaded.") : F("No valid value in EEPROM."));
        break;
    case 'p':
        _printing = !_printing;
        serial.println(_printing ? F("#printing resumed") : F("#printing paused"));
        break;
    case 'h':
        printHelp(serial);
        break;
    default:
        break; // ignore newlines and unknown characters
    }
}

bool LoadCell_lar::isPrinting(void)
{
    return _printing;
}

void LoadCell_lar::setPrinting(bool enabled)
{
    _printing = enabled;
}
