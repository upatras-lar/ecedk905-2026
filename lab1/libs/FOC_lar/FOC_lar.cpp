#include "FOC_lar.h"

// Constructor

FOC_lar::FOC_lar(int pole_pairs, int pin_a, int pin_b, int pin_c, int pin_en)
    : motor(pole_pairs),
      driver(pin_a, pin_b, pin_c, pin_en),
      target_voltage(0.0f),
      _voltage_limit(0.0f),
      _position_zero(0.0),
      _torque_per_volt(0.0f),
      _back_emf(0.0f),
      _printing(true)
{
}


// Sensor info: angle and magnet status (see AS5600_lar)

void FOC_lar::printSensorInfo(Print &out)
{
    print_sensor_info(out);
}


// Step 1: sensor + driver + motor init

bool FOC_lar::beginHardware(float supply_voltage,
                            float voltage_limit,
                            float align_voltage,
                            Print &log,
                            unsigned long i2c_clock)
{
    _voltage_limit = voltage_limit;

    // AS5600
    if (!AS5600_lar_begin(i2c_clock))
    {
        log.println(F("ERROR: Could not read AS5600!"));
        return false;
    }

    sensor.init();
    motor.linkSensor(&sensor);
    printSensorInfo(log);

    // driver
    driver.voltage_power_supply = supply_voltage;
    if (!driver.init())
    {
        log.println(F("ERROR: Driver initialization failed."));
        return false;
    }
    motor.linkDriver(&driver);

    // torque controller based on voltage, no current sensor required
    motor.torque_controller = TorqueControlType::voltage;
    motor.controller = MotionControlType::torque;

    // voltage limits
    motor.updateVoltageLimit(voltage_limit);
    motor.voltage_sensor_align = align_voltage;

    // motor
    if (!motor.init())
    {
        log.println(F("ERROR: Motor initialization failed."));
        return false;
    }

    return true;
}


// Step 2: FOC alignment

bool FOC_lar::alignFOC(Print &log)
{
    return alignFOC(NOT_SET, Direction::UNKNOWN, log);
}

bool FOC_lar::alignFOC(float zero_electric_angle, Direction sensor_direction, Print &log)
{
    // stored values: SimpleFOC skips the part of the alignment that is already known
    if (_isset(zero_electric_angle))
    {
        motor.zero_electric_angle = zero_electric_angle;
    }
    if (sensor_direction != Direction::UNKNOWN)
    {
        motor.sensor_direction = sensor_direction;
    }

    // simpleFOC debug output (not available when SIMPLEFOC_DISABLE_DEBUG is defined)
#ifndef SIMPLEFOC_DISABLE_DEBUG
    SimpleFOCDebug::enable(&log);
#endif

    if (_isset(motor.zero_electric_angle) && motor.sensor_direction != Direction::UNKNOWN)
    {
        log.println(F("Stored alignment used."));
    }
    else
    {
        log.println(F("Starting FOC alignment..."));
    }

    if (!motor.initFOC())
    {
        log.println(F("ERROR: initFOC() failed."));
        return false;
    }

    // update our sensor once after alignment
    if (!update_sensor())
    {
        log.println(F("ERROR: AS5600 read failed after alignment."));
        return false;
    }

    resetReportedZero();                    // establish reporting zero position
    AS5600_lar_reset_velocity_reference();  // reset velocity calculation origin
    target_voltage = 0.0f;                  // zero torque

    return true;
}


// Full init

bool FOC_lar::begin(float supply_voltage,
                    float voltage_limit,
                    float align_voltage,
                    Print &log,
                    unsigned long i2c_clock)
{
    if (!beginHardware(supply_voltage, voltage_limit, align_voltage, log, i2c_clock))
    {
        return false;
    }

    return alignFOC(log);
}


bool FOC_lar::begin(float supply_voltage,
                    float voltage_limit,
                    float align_voltage,
                    float zero_electric_angle,
                    Direction sensor_direction,
                    Print &log,
                    unsigned long i2c_clock)
{
    if (!beginHardware(supply_voltage, voltage_limit, align_voltage, log, i2c_clock))
    {
        return false;
    }

    return alignFOC(zero_electric_angle, sensor_direction, log);
}


// Pole pairs estimation

// Sweep the electrical angle over one full electrical revolution
// (3PI/2 -> 3PI/2 + 2PI when forward, and back when reverse).

static void sweep_electrical_revolution(BLDCMotor &motor,
                                        Sensor &sensor,
                                        float voltage,
                                        bool forward)
{
    for (int i = 0; i <= FOC_LAR_PP_SWEEP_STEPS; i++)
    {
        int k = forward ? i : (FOC_LAR_PP_SWEEP_STEPS - i);

        float angle = _3PI_2 + _2PI * k / (float)FOC_LAR_PP_SWEEP_STEPS;

        motor.setPhaseVoltage(voltage, 0, angle);
        sensor.update();

        _delay(FOC_LAR_PP_STEP_DELAY_MS);
    }
}

int FOC_lar::findPolePairs(float voltage, PolePairsResult *result, Print *log)
{
    PolePairsResult r;
    r.status = POLEPAIRS_ERR_NO_MOVEMENT;
    r.pole_pairs = 0;
    r.poles = 0;
    r.estimate = 0.0f;
    r.movement_rad = 0.0f;
    r.direction = Direction::UNKNOWN;

    if (log)
    {
        log->println();
        log->println(F("Starting FOC-style pole-pair measurement."));
        log->println(F("Motor will move."));
        log->println();
    }

    // forward sweep
    sweep_electrical_revolution(motor, sensor, voltage, true);

    sensor.update();
    float mid_angle = sensor.getAngle();

    if (log)
    {
        log->print(F("Mechanical angle after forward sweep: "));
        log->print(mid_angle * 180.0f / _PI, 3);
        log->println(F(" deg"));
    }

    // backward sweep
    sweep_electrical_revolution(motor, sensor, voltage, false);

    sensor.update();
    float end_angle = sensor.getAngle();

    if (log)
    {
        log->print(F("Mechanical angle after reverse sweep: "));
        log->print(end_angle * 180.0f / _PI, 3);
        log->println(F(" deg"));
    }

    // release motor
    motor.setPhaseVoltage(0, 0, 0);
    _delay(300);

    // mechanical movement corresponding to exactly one electrical revolution
    float moved = fabs(mid_angle - end_angle);
    r.movement_rad = moved;

    if (moved < FOC_LAR_PP_MIN_MOVEMENT_RAD)
    {
        r.status = POLEPAIRS_ERR_NO_MOVEMENT;

        if (log)
        {
            log->println();
            log->println(F("ERROR: Motor did not move enough. Increase the sweep voltage."));
        }

        if (result) *result = r;
        return 0;
    }

    // sensor direction (same logic used inside SimpleFOC)
    r.direction = (mid_angle < end_angle) ? Direction::CCW : Direction::CW;

    // pole pairs = 2PI / mechanical movement
    r.estimate = _2PI / moved;
    int pp = (int)round(r.estimate);

    if (log)
    {
        log->println();
        log->print(F("Sensor direction: "));
        log->println(r.direction == Direction::CW ? F("CW") : F("CCW"));
        log->print(F("Mechanical movement = "));
        log->print(moved * 180.0f / _PI, 4);
        log->println(F(" deg"));
        log->print(F("PP estimate = "));
        log->println(r.estimate, 4);
        log->print(F("Estimated PP : "));
        log->println(pp);
    }

    if (pp <= 0 || pp > FOC_LAR_PP_MAX_POLE_PAIRS)
    {
        r.status = POLEPAIRS_ERR_OUT_OF_RANGE;

        if (log)
        {
            log->println(F("ERROR: Invalid PP estimate."));
        }

        if (result) *result = r;
        return 0;
    }

    r.status = POLEPAIRS_OK;
    r.pole_pairs = pp;
    r.poles = 2 * pp;

    // configure the motor with the measured values
    motor.pole_pairs = pp;
    motor.sensor_direction = r.direction;

    if (log)
    {
        log->print(F("PP accepted. Motor pole pairs = "));
        log->println(motor.pole_pairs);
    }

    if (result) *result = r;
    return pp;
}


// FOC loop
// motor.loopFOC()
//     -> sensor.update() -> getSensorAngle() -> update_sensor() -> AS5600
// so the AS5600_lar library state is updated here.

void FOC_lar::update(void)
{
    motor.loopFOC();
    motor.move(target_voltage);
}


// Target voltage

void FOC_lar::setTargetVoltage(float voltage)
{
    target_voltage = constrain(voltage, -_voltage_limit, _voltage_limit);
}

float FOC_lar::getTargetVoltage(void)
{
    return target_voltage;
}


// Torque command: torque = _torque_per_volt * voltage

void FOC_lar::setTorqueConstant(float newton_meter_per_volt)
{
    _torque_per_volt = newton_meter_per_volt;
}

float FOC_lar::getTorqueConstant(void)
{
    return _torque_per_volt;
}

bool FOC_lar::setTargetTorque(float torque_nm)
{
    if (_torque_per_volt <= 0.0f)
    {
        return false;   // not calibrated
    }

    setTargetVoltage(torque_nm / _torque_per_volt);
    return true;
}

void FOC_lar::setBackEmfConstant(float volt_second_per_rad)
{
    _back_emf = volt_second_per_rad;
}

float FOC_lar::getBackEmfConstant(void)
{
    return _back_emf;
}

bool FOC_lar::setTargetTorque(float torque_nm, float speed_rad_s)
{
    if (_torque_per_volt <= 0.0f)
    {
        return false;   // not calibrated
    }

    setTargetVoltage(torque_nm / _torque_per_volt + _back_emf * speed_rad_s);
    return true;
}

float FOC_lar::getTargetTorque(void)
{
    return target_voltage * _torque_per_volt;
}

float FOC_lar::getMaxTorque(void)
{
    return _voltage_limit * _torque_per_volt;
}


// Serial input: one number per line

static const unsigned int FOC_LAR_RX_MAX = 24;

bool FOC_lar::receiveSerialCommand(Stream &serial)
{
    bool received = false;

    while (serial.available())
    {
        char c = (char)serial.read();

        // Single-key commands are only accepted at the start of a line, so a
        // number such as "1.5" is never split.
        if (_rx_buffer.length() == 0 && (c == 'p' || c == 'h'))
        {
            if (c == 'p')
            {
                _printing = !_printing;
                serial.println(_printing ? F("#printing resumed") : F("#printing paused"));
            }
            else
            {
                printHelp(serial);
            }
            continue;
        }

        if (c == '\n' || c == '\r')
        {
            if (_rx_buffer.length() > 0)
            {
                setTargetVoltage(_rx_buffer.toFloat());

                serial.println("Target voltage = " + String(target_voltage, 3));

                _rx_buffer = "";
                received = true;
            }
        }
        else if (_rx_buffer.length() < FOC_LAR_RX_MAX)
        {
            _rx_buffer += c;
        }
    }

    return received;
}


// Printing state and help

bool FOC_lar::isPrinting(void) const
{
    return _printing;
}

void FOC_lar::setPrinting(bool enabled)
{
    _printing = enabled;
}

void FOC_lar::printHelp(Print &out)
{
    out.println(F("***"));
    out.print(F("number : set target voltage (0.0 - "));
    out.print(_voltage_limit, 1);
    out.println(F(" V)"));
    out.println(F("p : pause / resume printing"));
    out.println(F("h : this help"));
    out.println(F("***"));
}


// Position relative to our zero

double FOC_lar::getReportedPosition(void)
{
    return get_position() - _position_zero;
}

void FOC_lar::resetReportedZero(void)
{
    _position_zero = get_position();
}


// Status line

String FOC_lar::getStatusString(void)
{
    String output = get_sensor_status_string(_position_zero);
    output.reserve(output.length() + 16);
    output += F(" | V=");
    output += fixed_float(target_voltage, 5, 2);
    return output;
}


// Summary after init

void FOC_lar::printSummary(Print &out)
{
    out.println(F("Motor ready."));
    out.print(F("Motor pole pairs = "));
    out.println(motor.pole_pairs);

    print_velocity_ema_settings(out);

    printAlignment(out);
}


// Alignment values, ready to paste

void FOC_lar::printAlignment(Print &out)
{
    out.println(F("Alignment values to paste in the sketch:"));

    out.print(F("#define STORED_ZERO_ELECTRIC_ANGLE "));
    out.print(motor.zero_electric_angle, 5);
    out.println('f');

    out.print(F("#define STORED_SENSOR_DIRECTION Direction::"));
    out.println(motor.sensor_direction == Direction::CW ? F("CW") : F("CCW"));
}
