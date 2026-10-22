#ifndef FOC_LAR_H
#define FOC_LAR_H

#include <Arduino.h>
#include <SimpleFOC.h>
#include <AS5600_lar.h>

// FOC_lar
//
// Convenience layer on top of SimpleFOC + AS5600_lar for a
// BLDC motor driven by a 3-PWM driver (e.g. SimpleFOCMini) and
// an AS5600 magnetic sensor on I2C.
//
// It provides:
//   - AS5600LarSensor : SimpleFOC Sensor backed by the AS5600_lar library
//   - FOC_lar         : owns motor + driver + sensor, does the whole
//                       init / alignment sequence, the FOC loop,
//                       serial voltage commands, torque command,
//                       pole pairs estimation and status reporting.
//
// Typical use:
//
//   FOC_lar foc(7, 11, 10, 9, 8);   // pole pairs, IN1, IN2, IN3, EN
//
//   void setup() {
//       Serial.begin(115200);
//       if (!foc.begin(12.0f, 2.0f, 1.0f)) while (1);
//   }
//   void loop() {
//       foc.update();
//       foc.receiveSerialCommand();
//   }

// SimpleFOC sensor backed by AS5600_lar

class AS5600LarSensor : public Sensor
{
public:
    void init()
    {
        Sensor::init();
    }

    unsigned int getRawAngle()
    {
        return (unsigned int)get_raw_angle();
    }

protected:
    float getSensorAngle() override
    {
        // Keep the last valid angle if the read fails. A sentinel (e.g. -1)
        // would look like a large jump to SimpleFOC and inject a velocity
        // spike into the torque command.
        update_sensor();
        return (float)get_mechanical_angle();
    }
};

// Pole pairs estimation (FOC_lar::findPolePairs)
//
// Tunable constants (compile-time): edit the default values below in this file,
// or pass them as -D build flags. A #define in the sketch has no effect, because
// the sweep code is compiled separately in FOC_lar.cpp.

#ifndef FOC_LAR_PP_SWEEP_STEPS
#define FOC_LAR_PP_SWEEP_STEPS 500 // steps per electrical revolution
#endif

#ifndef FOC_LAR_PP_STEP_DELAY_MS
#define FOC_LAR_PP_STEP_DELAY_MS 2 // delay between steps (ms)
#endif

#ifndef FOC_LAR_PP_MIN_MOVEMENT_RAD
#define FOC_LAR_PP_MIN_MOVEMENT_RAD 0.01f // less than this = motor did not move
#endif

#ifndef FOC_LAR_PP_MAX_POLE_PAIRS
#define FOC_LAR_PP_MAX_POLE_PAIRS 30 // larger results are rejected
#endif

enum PolePairsStatus
{
    POLEPAIRS_OK = 0,
    POLEPAIRS_ERR_NO_MOVEMENT, // motor did not move: increase the voltage
    POLEPAIRS_ERR_OUT_OF_RANGE // estimate <= 0 or > FOC_LAR_PP_MAX_POLE_PAIRS
};

struct PolePairsResult
{
    PolePairsStatus status;
    int pole_pairs;      // rounded estimate
    int poles;           // 2 * pole_pairs
    float estimate;      // raw (non-rounded) estimate, ideally close to an integer
    float movement_rad;  // mechanical movement for one electrical revolution
    Direction direction; // detected sensor direction
};

// Motor + driver + sensor wrapper

class FOC_lar
{
public:
    // pole_pairs : motor pole pairs (use findPolePairs() to find it if unknown)
    // pin_a/b/c  : driver PWM pins (IN1, IN2, IN3)
    // pin_en     : driver enable pin
    FOC_lar(int pole_pairs, int pin_a, int pin_b, int pin_c, int pin_en);

    // SimpleFOC objects are public so they can be used directly,
    // e.g. foc.motor.zero_electric_angle
    BLDCMotor motor;
    BLDCDriver3PWM driver;
    AS5600LarSensor sensor;

    // Current target (V), set by receiveSerialCommand() or setTargetVoltage()
    float target_voltage;

    // Full initialization: beginHardware() + alignFOC()
    bool begin(float supply_voltage,
               float voltage_limit,
               float align_voltage,
               Print &log = Serial,
               unsigned long i2c_clock = AS5600_LAR_DEFAULT_I2C_CLOCK);

    // Same as begin(), but with the alignment values of a previous run, so the
    // alignment does not move the shaft at all (see alignFOC(zero, direction)).
    bool begin(float supply_voltage,
               float voltage_limit,
               float align_voltage,
               float zero_electric_angle,
               Direction sensor_direction,
               Print &log = Serial,
               unsigned long i2c_clock = AS5600_LAR_DEFAULT_I2C_CLOCK);

    // Step 1: sensor + driver + motor init (no FOC alignment yet).
    // After this you may change motor.pole_pairs / motor.sensor_direction
    // (this is what findPolePairs() does) before calling alignFOC().
    bool beginHardware(float supply_voltage,
                       float voltage_limit,
                       float align_voltage,
                       Print &log = Serial,
                       unsigned long i2c_clock = AS5600_LAR_DEFAULT_I2C_CLOCK);

    // Step 2: SimpleFOC alignment (initFOC), reporting zero and velocity reference reset
    bool alignFOC(Print &log = Serial);

    // Step 2 with stored alignment values: the shaft does NOT move.
    //
    // Use the values printed by printAlignment() / printSummary() after a normal
    // alignment (done once, with the shaft free). The AS5600 is an absolute sensor,
    // so they stay valid across power cycles while the magnet / sensor mounting,
    // the motor phase wiring and the pole pairs do not change.
    //
    // A value left unset (zero_electric_angle = NOT_SET, sensor_direction =
    // Direction::UNKNOWN) is measured as usual, so both can be left unset to get
    // the normal alignment. A direction already set in motor.sensor_direction
    // (e.g. by findPolePairs()) is kept.
    bool alignFOC(float zero_electric_angle,
                  Direction sensor_direction,
                  Print &log = Serial);

    // Estimates the number of pole pairs of the motor.
    //
    // The electrical angle is swept through exactly ONE electrical revolution
    // (forward, then backward) and the mechanical movement of the shaft is
    // measured with the sensor:
    //
    //     pole pairs = 2*PI / mechanical movement (rad)
    //
    // The sensor direction is detected as a by-product.
    //
    // Call it after beginHardware() and BEFORE alignFOC(). The shaft must be free.
    //
    // voltage : phase voltage of the sweep (V). Start with 1.0; increase if the
    //           rotor does not follow the field.
    // result  : optional, receives the details
    // log     : optional, e.g. &Serial, for progress messages
    //
    // Returns the pole pairs (> 0) or 0 on error. On success motor.pole_pairs and
    // motor.sensor_direction are updated, so alignFOC() can be called right after.
    // On failure the motor settings are left unchanged.
    int findPolePairs(float voltage = 1.0f,
                      PolePairsResult *result = nullptr,
                      Print *log = nullptr);

    // Call as often as possible: motor.loopFOC() + motor.move(target_voltage)
    void update(void);

    // Target voltage (constrained to +/- voltage_limit)
    void setTargetVoltage(float voltage);
    float getTargetVoltage(void);

    // Torque command (N*m), converted to voltage with a calibration constant.
    //
    // The constant (N*m per volt) must be measured, e.g. with a lever arm
    // pushing on a load cell. It is valid near standstill / low speed:
    // at higher speed the back-EMF reduces the real torque for the same voltage.
    void setTorqueConstant(float newton_meter_per_volt);
    float getTorqueConstant(void);

    // Sets the target voltage that corresponds to torque_nm (constrained to
    // +/- voltage_limit). Returns false if the torque constant is not set,
    // in which case nothing is changed.
    bool setTargetTorque(float torque_nm);

    // Back-EMF constant Ke (V*s/rad), from velocity_constant_calibration (use the mean Ke,
    // or the low-speed Ke). Default 0 = no compensation.
    //
    // In voltage mode the real torque is  torque = k * (V - Ke * speed): the back-EMF eats
    // part of the voltage. With the speed known, setTargetTorque(torque, speed) applies
    //     V = torque / k + Ke * speed
    // so the shaft gets the requested torque (and no extra electrical damping).
    void setBackEmfConstant(float volt_second_per_rad);
    float getBackEmfConstant(void);

    // Torque command with back-EMF compensation. speed_rad_s is the shaft speed in the
    // direction of positive voltage. Returns false if the torque constant is not set.
    bool setTargetTorque(float torque_nm, float speed_rad_s);

    // Torque corresponding to the current target voltage (0 if not calibrated)
    float getTargetTorque(void);

    // Largest torque reachable with the voltage limit (0 if not calibrated)
    float getMaxTorque(void);

    // Non-blocking serial parser. Reads a line containing a number and
    // sets the target voltage. Also handles:
    //   p : pause/resume the periodic print (see isPrinting())
    //   h : print the command list for this sketch
    // Returns true when a new target voltage was received.
    bool receiveSerialCommand(Stream &serial = Serial);

    // Printing state toggled by the 'p' command (starts enabled)
    bool isPrinting(void) const;
    void setPrinting(bool enabled);

    // Prints the commands handled by receiveSerialCommand(), with the
    // voltage range of this instance.
    void printHelp(Print &out = Serial);

    // Position relative to the zero taken after alignment (rad)
    double getReportedPosition(void);
    void resetReportedZero(void);

    // One line: get_sensor_status_string() (relative to the reporting zero) + target voltage
    String getStatusString(void);

    // Print helpers (the sensor ones use the AS5600_lar print functions)
    void printSensorInfo(Print &out); // raw angle + angle + magnet status
    void printSummary(Print &out);    // pole pairs, direction, zero angle, EMA + printAlignment()

    // Prints the alignment results as two #define lines, ready to be pasted into
    // a sketch and passed to begin() / alignFOC() to skip the alignment.
    void printAlignment(Print &out);

private:
    float _voltage_limit;
    double _position_zero;
    float _torque_per_volt;
    float _back_emf;
    bool _printing;
    String _rx_buffer;
};

#endif
