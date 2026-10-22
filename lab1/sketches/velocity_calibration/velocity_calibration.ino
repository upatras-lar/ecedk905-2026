// Measures the relation between the voltage command and the steady-state speed
// of the FOC motor, with the shaft FREE (no rod, no load).
//
// In voltage mode the speed settles on a straight line:
//     speed = (|V| - V_f) / Ke
// Ke is the back-EMF constant (V*s/rad) and V_f the voltage that only overcomes
// friction and cogging (dead zone). The voltage is increased in DATA_POINTS equal
// steps from 0 V to +MOTOR_VOLTAGE_LIMIT, then from 0 V to -MOTOR_VOLTAGE_LIMIT, so
// both directions can be compared (asymmetry).
//
// At every step: wait STAB_TIME_S for the speed to settle, then average the
// filtered speed during MEAS_TIME_S.
//
// Output (serial, 115200 baud): CSV lines
//     point,voltage_V,speed_rad_s,speed_rpm,std_rad_s
// std_rad_s is the variation of the speed inside the averaging window (ripple/noise).
// All other messages start with '#'. Use velocity_fit.py to capture the
// data, plot it and fit Ke and V_f.
//
// Procedure:
//   1. REMOVE the rod / lever arm: the shaft must spin freely.
//   2. When '#READY' appears, send 's' (the python script does this when you press Enter).
//   3. Send 'x' at any time to abort (0 V).

#include <FOC_lar.h>

// Motor definitions

#define MOTOR_POLE_PAIRS NOT_SET   // measured with find_pole_pairs_number
#define MOTOR_SUPPLY_VOLTAGE 12.0f // motor supply
#define MOTOR_VOLTAGE_LIMIT 3.0f   // last voltage of each sweep (+ and -)
#define SENSOR_ALIGN_VOLTAGE 1.0f  // voltage used for FOC sensor alignment

// Sweep definitions

#define DATA_POINTS 50    // equal voltage increases per direction
#define STAB_TIME_S 3.0f  // wait after every increase, for the speed to settle (s)
#define MEAS_TIME_S 1.0f  // averaging window after the wait (s)
#define COAST_TIME_S 3.0f // 0 V pause between the + and - sweeps (s)

#if DATA_POINTS < 1
#error "DATA_POINTS must be at least 1"
#endif

#define RAD_S_TO_RPM 9.5492966f // 60 / (2*PI)

// SimpleFOCMini v1.0 + Arduino UNO: pole pairs, IN1, IN2, IN3, EN
FOC_lar foc(MOTOR_POLE_PAIRS, 11, 10, 9, 8);

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

// Runs the motor loop for a given time.
// Returns false if 'x' was received (abort).

bool runFor(unsigned long duration_ms)
{
    unsigned long start = millis();

    while (millis() - start < duration_ms)
    {
        foc.update();

        bool start_requested = false;
        if (handleSerial(start_requested))
        {
            return false;
        }
    }
    return true;
}

// Runs the motor loop and averages the filtered speed (rad/s) over duration_ms
// (Welford's algorithm, stable in 32 bit floats).
// Returns false if 'x' was received (abort).

bool measureSpeed(unsigned long duration_ms, float &mean, float &std_dev)
{
    unsigned long start = millis();
    float m = 0.0f;
    float m2 = 0.0f;
    unsigned long n = 0;

    while (millis() - start < duration_ms)
    {
        foc.update();

        float x = (float)get_velocity_rad_sec();
        n++;
        float delta = x - m;
        m += delta / n;
        m2 += delta * (x - m);

        bool start_requested = false;
        if (handleSerial(start_requested))
        {
            return false;
        }
    }

    mean = m;
    std_dev = (n > 1) ? sqrt(m2 / (n - 1)) : 0.0f;
    return true;
}

void printRow(int point, float voltage, float speed_rad_s, float std_rad_s)
{
    Serial.print(point);
    Serial.print(',');
    Serial.print(voltage, 3);
    Serial.print(',');
    Serial.print(speed_rad_s, 3);
    Serial.print(',');
    Serial.print(speed_rad_s * RAD_S_TO_RPM, 2);
    Serial.print(',');
    Serial.println(std_rad_s, 3);
}

void abortSweep()
{
    foc.setTargetVoltage(0.0f);
    Serial.println(F("#ABORTED"));
}

void runSweep()
{
    Serial.println(F("#Starting sweep."));
    Serial.println(F("point,voltage_V,speed_rad_s,speed_rpm,std_rad_s"));

    int point = 0;

    for (int sign = 1; sign >= -1; sign -= 2)
    {
        if (sign == -1)
        {
            // let the motor stop before reversing
            foc.setTargetVoltage(0.0f);
            Serial.println(F("#Coasting to 0 V before the negative sweep..."));
            if (!runFor((unsigned long)(COAST_TIME_S * 1000.0f)))
            {
                abortSweep();
                return;
            }
        }

        // 0 V is measured once, in the positive sweep
        for (int i = (sign == 1 ? 0 : 1); i <= DATA_POINTS; i++)
        {
            float voltage = sign * MOTOR_VOLTAGE_LIMIT * i / DATA_POINTS;
            foc.setTargetVoltage(voltage);

            Serial.print(F("#Step "));
            Serial.print(i);
            Serial.print('/');
            Serial.print(DATA_POINTS);
            Serial.print(F(": "));
            Serial.print(voltage, 3);
            Serial.println(F(" V"));

            if (!runFor((unsigned long)(STAB_TIME_S * 1000.0f)))
            {
                abortSweep();
                return;
            }

            float mean, std_dev;
            if (!measureSpeed((unsigned long)(MEAS_TIME_S * 1000.0f), mean, std_dev))
            {
                abortSweep();
                return;
            }

            if (printing)
            {
                printRow(point, voltage, mean, std_dev);
            }
            point++;
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
    Serial.println(F("#Velocity constant measurement"));

    set_velocity_ema_filter(true, 0.05);

    // motor: the shaft must be free during the alignment
    if (!foc.begin(MOTOR_SUPPLY_VOLTAGE, MOTOR_VOLTAGE_LIMIT, SENSOR_ALIGN_VOLTAGE))
    {
        while (1)
            ;
    }

    printHelp();
}

void loop()
{
    static unsigned long last_prompt = 0;

    // motor at 0 V
    foc.update();

    // until the first sweep, repeat the prompt so a late listener does not miss it
    if (!sweep_done_once && millis() - last_prompt >= 2000)
    {
        last_prompt = millis();
        Serial.println(F("#READY: remove the rod (shaft must spin freely), then send 's'"));
    }

    bool start_requested = false;
    handleSerial(start_requested);
    if (start_requested)
    {
        runSweep();
    }
}
