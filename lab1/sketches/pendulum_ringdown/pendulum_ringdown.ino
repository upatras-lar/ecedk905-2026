// Pendulum ring-down test: identifies the friction of the shaft.
//
// The pendulum swings freely and the angle is logged while the oscillation decays. The decay gives
// the viscous friction b (the envelope falls as exp(-b*t/(2*J))) and the Coulomb friction (linear
// decay), and the period gives G/J. pendulum_ringdown_fit.py fits the nonlinear pendulum model to the
// log:    J*theta'' = -b*theta' - G*sin(theta) - tau_c*sign(theta')
//
// BRAKED_MODE 0 (default): the driver is DISABLED (enable pin low): the phases float, there is no
//   electrical braking, and the decay shows the MECHANICAL friction only.
//   If your driver has no working enable pin, disconnect the three motor phase wires instead.
// BRAKED_MODE 1: the driver is enabled with 0 V (all phases at the same duty): the motor brakes
//   electrically, with a damping of about k*Ke. Running both modes and comparing the two fits gives the
//   electrical damping, a check of your torque and back-EMF constants.
//
// Procedure:
//   1. Let the pendulum hang at rest, then send 's' (the python script does it when you press Enter).
//      The sketch averages the angle for ZERO_TIME_S: do not touch the pendulum.
//   2. At the message '#GO', lift the pendulum to about 60-90 degrees, HOLD IT STILL for about a
//      second, then RELEASE it without pushing. The log runs for RUN_TIME_S.
//   3. Run pendulum_ringdown_fit.py on the log.
//
// Log (CSV): t_s,theta_rad      (theta = 0 at the hanging rest position). Lines starting with '#' are messages.
// Send 'x' to stop.

#include <AS5600_lar.h>

#define BRAKED_MODE 0 // 0: driver disabled (free), 1: driver enabled at 0 V (electrical braking)

#define DRIVER_EN_PIN 8  // driver enable pin
#define LOG_RATE_HZ 200  // samples per second
#define ZERO_TIME_S 1.0f // averaging time of the rest angle
#define RUN_TIME_S 30.0f // logging time after '#GO'

#if BRAKED_MODE
#include <FOC_lar.h>

#define MOTOR_POLE_PAIRS NOT_SET
#define MOTOR_SUPPLY_VOLTAGE 12.0f
#define MOTOR_VOLTAGE_LIMIT 3.0f

// SimpleFOCMini v1.0 + Arduino UNO: pole pairs, IN1, IN2, IN3, EN
FOC_lar foc(MOTOR_POLE_PAIRS, 11, 10, 9, DRIVER_EN_PIN);
#endif

enum State
{
    IDLE,
    ZEROING,
    LOGGING
};

State state = IDLE;
bool ran_once = false;
bool printing = true;
double zero_angle = 0.0;
double zero_sum = 0.0;
unsigned long zero_count = 0;
unsigned long t_state_us = 0;
unsigned long t_last_log_us = 0;
unsigned long t_prompt_ms = 0;

void printHelp()
{
    Serial.println(F("***"));
    Serial.println(F("s : start a run (zero, then log)"));
    Serial.println(F("x : stop the run"));
    Serial.println(F("p : pause / resume the CSV log"));
    Serial.println(F("h : this help"));
    Serial.println(F("***"));
}

void setup()
{
    Serial.begin(115200);
    Serial.println();
    Serial.println(BRAKED_MODE ? F("#Pendulum ring-down: driver enabled at 0 V (electrical braking)")
                               : F("#Pendulum ring-down: driver disabled (free)"));

#if BRAKED_MODE
    // sensor, driver and motor init without the alignment (the shaft does not move)
    if (!foc.beginHardware(MOTOR_SUPPLY_VOLTAGE, MOTOR_VOLTAGE_LIMIT, 1.0f))
    {
        while (1)
            ;
    }
    foc.motor.enable();
    foc.motor.setPhaseVoltage(0.0f, 0.0f, 0.0f); // all phases at the same duty: no net voltage
#else
    pinMode(DRIVER_EN_PIN, OUTPUT);
    digitalWrite(DRIVER_EN_PIN, LOW); // phases floating
    if (!AS5600_lar_begin())
    {
        Serial.println(F("#ERROR: could not read the AS5600, check the wiring."));
        while (1)
            ;
    }
#endif

    printHelp();
}

void loop()
{
    update_sensor(); // as often as possible: keeps the angle and the turns up to date

    while (Serial.available())
    {
        char c = (char)Serial.read();

        if (c == 'x' && state != IDLE)
        {
            state = IDLE;
            Serial.println(F("#ABORTED"));
        }
        else if (c == 's' && state == IDLE)
        {
            state = ZEROING;
            zero_sum = 0.0;
            zero_count = 0;
            t_state_us = micros();
            ran_once = true;
            Serial.println(F("#Zeroing: do not touch the pendulum..."));
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

    if (state == IDLE)
    {
        // repeat the prompt until the first run
        if (!ran_once && millis() - t_prompt_ms >= 2000)
        {
            t_prompt_ms = millis();
            Serial.println(F("#READY: pendulum hanging at rest, then send 's'"));
        }
        return;
    }

    unsigned long now = micros();

    if (state == ZEROING)
    {
        zero_sum += get_position();
        zero_count++;

        if (now - t_state_us >= (unsigned long)(ZERO_TIME_S * 1e6f))
        {
            zero_angle = zero_sum / zero_count;
            state = LOGGING;
            t_state_us = now;
            t_last_log_us = now;
            Serial.println(F("#GO: lift the pendulum, hold it still for a second, then release it"));
            Serial.println(F("t_s,theta_rad"));
        }
        return;
    }

    // LOGGING
    if (now - t_state_us >= (unsigned long)(RUN_TIME_S * 1e6f))
    {
        state = IDLE;
        Serial.println(F("#DONE"));
        Serial.println(F("#Send 's' to repeat."));
        return;
    }

    if (now - t_last_log_us >= (unsigned long)(1e6f / LOG_RATE_HZ))
    {
        t_last_log_us += (unsigned long)(1e6f / LOG_RATE_HZ);

        if (printing)
        {
            Serial.print((now - t_state_us) * 1e-6f, 4);
            Serial.print(',');
            Serial.println((float)(get_position() - zero_angle), 5);
        }
    }
}
