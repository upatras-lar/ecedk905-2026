// Measures the constant that connects the voltage command and the torque of
// the FOC motor (N*m per volt).
//
// Setup: a light rod on the motor shaft pushes on a load cell (HX711), so the
// rotor is blocked (stall). The voltage is increased in DATA_POINTS equal steps
// from 0 V to MOTOR_VOLTAGE_LIMIT. After every step the sketch waits STAB_TIME_S,
// reads the load cell and computes torque = force * ROD_LENGTH_M.
//
// Output (serial, 115200 baud): CSV lines
//     point,voltage_V,mass_g,force_N,torque_Nm,speed_rpm
// All other messages start with '#'. Use torque_fit.py to capture the
// data, plot it and fit the constant.
//
// Procedure:
//   1. Calibrate the load cell first (loadcell_calib_read.ino); the factor is
//      read from EEPROM.
//   2. The FOC alignment moves the shaft (one electrical revolution back and forth)
//      and is disturbed by the weight of the rod. Two ways to handle it:
//        a) Power up with the load cell OUT of reach and the rod hanging vertically
//           (no gravity torque), then bring the load cell in after '#READY'.
//        b) Align ONCE with the shaft free (rod removed), e.g. with motor_AS5600:
//           copy the two STORED_... lines it prints into the defines below. The shaft
//           then does not move at power-up and the rod can stay on.
//   3. When '#READY' appears, bring the rod against the load cell with no preload,
//      then send 's' (the python script does this when you press Enter).
//   4. Send 'x' at any time to abort (0 V).
//
// WARNING: the motor is stalled during the test and the windings heat up.
// Set a current limit on the bench supply and keep MOTOR_VOLTAGE_LIMIT low.

#include <FOC_lar.h>
#include <LoadCell_lar.h>

// Motor definitions

#define MOTOR_POLE_PAIRS NOT_SET   // measured with find_pole_pairs_number
#define MOTOR_SUPPLY_VOLTAGE 12.0f // motor supply
#define MOTOR_VOLTAGE_LIMIT 3.0f   // last voltage of the sweep
#define SENSOR_ALIGN_VOLTAGE 1.0f  // voltage used for FOC sensor alignment

// Stored alignment (see the procedure). Leave them as they are to align at every power-up.
// After a normal alignment with the shaft free, motor_AS5600 prints the two lines to paste here.
#define STORED_ZERO_ELECTRIC_ANGLE NOT_SET         // e.g. 3.12345f  (values printed by motor_AS5600)
#define STORED_SENSOR_DIRECTION Direction::UNKNOWN // Direction::CW or Direction::CCW

// Sweep definitions

#define DATA_POINTS 50   // number of equal voltage increases (DATA_POINTS + 1 rows, the first is 0 V)
#define STAB_TIME_S 4.0f // wait after every increase (s). The load cell average needs about 3 s
#define VOLTAGE_SIGN 1   // +1 or -1: choose the direction that pushes the rod onto the load cell

// Load cell definitions

#define HX711_DOUT_PIN 4          // MCU > HX711 DOUT
#define HX711_SCK_PIN 5           // MCU > HX711 SCK
#define DEFAULT_CAL_FACTOR 650.00 // used if no valid value is stored in EEPROM
#define EEPROM_CAL_ADDRESS 0      // EEPROM address of the calibration factor
#define LOADCELL_REVERSE 0        // 1 = invert the sign of the load cell reading
#define ROD_LENGTH_M NOT_SET      // shaft axis to be set -> load cell contact point (m)

#if DATA_POINTS < 1
#error "DATA_POINTS must be at least 1"
#endif

// SimpleFOCMini v1.0 + Arduino UNO: pole pairs, IN1, IN2, IN3, EN
FOC_lar foc(MOTOR_POLE_PAIRS, 11, 10, 9, 8);
LoadCell_lar cell(HX711_DOUT_PIN, HX711_SCK_PIN);

bool sweep_done_once = false;
bool printing = true;

void printHelp()
{
    Serial.println(F("***"));
    Serial.println(F("s : start the sweep (when ready)"));
    Serial.println(F("x : abort the sweep (0 V)"));
    Serial.println(F("p : pause / resume the CSV log"));
    Serial.println(F("h : this help"));
    Serial.println(F("***"));
}

// Handles the serial keys that work everywhere (idle and during a sweep).
// Returns true when 'x' was received.

bool handleSerial(bool &start_requested)
{
    bool abort = false;

    while (Serial.available())
    {
        char c = (char)Serial.read();

        if (c == 's')
        {
            start_requested = true;
        }
        else if (c == 'x')
        {
            abort = true;
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

    return abort;
}

// Runs the motor loop and the load cell for a given time.
// Returns false if 'x' was received (abort).

bool runFor(unsigned long duration_ms)
{
    unsigned long start = millis();

    while (millis() - start < duration_ms)
    {
        foc.update();
        cell.update();

        bool start_requested = false;
        if (handleSerial(start_requested))
        {
            return false;
        }
    }
    return true;
}

void printRow(int point, float voltage)
{
    Serial.print(point);
    Serial.print(',');
    Serial.print(voltage, 3);
    Serial.print(',');
    Serial.print(cell.getGrams(), 2);
    Serial.print(',');
    Serial.print(cell.getForceN(), 5);
    Serial.print(',');
    Serial.print(cell.getTorque(ROD_LENGTH_M), 6);
    Serial.print(',');
    Serial.println(get_velocity_rpm(), 2);
}

void runSweep()
{
    // tare with the rod touching the load cell and the motor off
    foc.setTargetVoltage(0.0f);
    Serial.println(F("#Taring..."));
    cell.tareNoDelay();
    while (!cell.isTareDone())
    {
        foc.update();
        cell.update();

        bool start_requested = false;
        if (handleSerial(start_requested))
        {
            foc.setTargetVoltage(0.0f);
            Serial.println(F("#ABORTED"));
            return;
        }
    }
    Serial.println(F("#Tare complete. Starting sweep."));

    Serial.println(F("point,voltage_V,mass_g,force_N,torque_Nm,speed_rpm"));

    for (int i = 0; i <= DATA_POINTS; i++)
    {
        float voltage = MOTOR_VOLTAGE_LIMIT * i / DATA_POINTS;
        foc.setTargetVoltage(VOLTAGE_SIGN * voltage);

        Serial.print(F("#Step "));
        Serial.print(i);
        Serial.print('/');
        Serial.print(DATA_POINTS);
        Serial.print(F(": "));
        Serial.print(VOLTAGE_SIGN * voltage, 3);
        Serial.println(F(" V"));

        if (!runFor((unsigned long)(STAB_TIME_S * 1000.0f)))
        {
            foc.setTargetVoltage(0.0f);
            Serial.println(F("#ABORTED"));
            return;
        }

        if (printing)
        {
            printRow(i, VOLTAGE_SIGN * voltage);
        }
    }

    foc.setTargetVoltage(0.0f);
    sweep_done_once = true;
    Serial.println(F("#DONE"));
    Serial.println(F("#Send 's' to repeat."));
}

void setup()
{
    Serial.begin(115200);
    Serial.println();
    Serial.println(F("#Torque constant measurement"));

    set_velocity_ema_filter(true, 0.05);

    // load cell: start, stabilize and tare
    if (!cell.begin(DEFAULT_CAL_FACTOR))
    {
        while (1)
            ;
    }
    if (LOADCELL_REVERSE)
    {
        cell.setReverseOutput();
    }
    cell.loadCalFactor(EEPROM_CAL_ADDRESS);

    Serial.print(F("#Load cell calibration factor: "));
    Serial.println(cell.getCalFactor(), 4);

    // motor: with stored alignment values the shaft does not move; otherwise keep the
    // load cell out of reach of the rod during the alignment
    if (!foc.begin(MOTOR_SUPPLY_VOLTAGE, MOTOR_VOLTAGE_LIMIT, SENSOR_ALIGN_VOLTAGE,
                   STORED_ZERO_ELECTRIC_ANGLE, STORED_SENSOR_DIRECTION))
    {
        while (1)
            ;
    }

    Serial.println("Sweeping voltage in the range: 0.0 to " + String(MOTOR_VOLTAGE_LIMIT) + " V.");
    printHelp();
}

void loop()
{
    static unsigned long last_prompt = 0;

    // motor at 0 V, load cell running
    foc.update();
    cell.update();

    // until the first sweep, repeat the prompt so a late listener does not miss it
    if (!sweep_done_once && millis() - last_prompt >= 2000)
    {
        last_prompt = millis();
        Serial.println(F("#READY: put the rod against the load cell (no preload), then send 's'"));
    }

    bool start_requested = false;
    handleSerial(start_requested);
    if (start_requested)
    {
        runSweep();
    }
}
