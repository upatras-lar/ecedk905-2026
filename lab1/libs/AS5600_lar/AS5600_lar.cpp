#include "AS5600_lar.h"
#include <Wire.h>

// Global variables

// position/angle state
static unsigned int cur_raw_angle = 0;
static double mech_angle_rad = 0.0;
static signed long int full_rot = 0;
static bool sensor_init = false;

// velocity state
static double prev_angle_rad = 0.0;
static unsigned long int prev_vel_time_us = 0;
static double vel_raw_rad_sec = 0.0;
static double vel_raw_rpm = 0.0;
static double vel_rad_sec = 0.0;
static double vel_rpm = 0.0;

// velocity EMA filter state
static bool velocity_ema_enabled = false;
static double velocity_ema_alpha = 1.0;
static bool velocity_ema_initialized = false;

static const unsigned long int VELOCITY_INTERVAL_US = (unsigned long int)(1e6 / AS5600_LAR_VEL_UPDATE_FREQ);

// Generic AS5600 I2C register read

bool read_AS5600_reg(unsigned char start_reg, unsigned char *buffer, unsigned char bytes_num)
{
    Wire.beginTransmission(AS5600_LAR_CHIP_ADDR);
    Wire.write(start_reg);

    // keep the I2C communication open (false = send repeated START instead of STOP)
    unsigned char error = Wire.endTransmission(false);

    if (error != 0)
    {
        return false;
    }

    // request the register data
    unsigned char bytes_received =
        Wire.requestFrom((unsigned char)AS5600_LAR_CHIP_ADDR, bytes_num);

    if (bytes_received != bytes_num)
    {
        return false;
    }

    for (unsigned char i = 0; i < bytes_num; i++)
    {
        if (!Wire.available())
        {
            return false;
        }

        buffer[i] = Wire.read();
    }

    return true;
}

// Read a 12-bit AS5600 register

bool read_AS5600_12bit_reg(unsigned char start_reg, unsigned int &value)
{
    unsigned char data[2];

    if (!read_AS5600_reg(start_reg, data, 2))
    {
        return false;
    }

    // combine the two bytes into one 16-bit value
    value = ((unsigned int)data[0] << 8) | (unsigned int)data[1];
    value &= 0x0FFF; // AS5600 uses only bits 11:0

    return true;
}

// Write a 12-bit AS5600 configuration register
// Used for:
//   ZPOS
//   MPOS
//   MANG (if added later)

bool write_AS5600_12bit_reg(unsigned char start_reg, unsigned int value)
{
    // make sure only the lower 12 bits are written
    value &= 0x0FFF;

    Wire.beginTransmission(AS5600_LAR_CHIP_ADDR);
    Wire.write(start_reg);

    Wire.write((unsigned char)((value >> 8) & 0x0F)); // upper 4 bits are used
    Wire.write((unsigned char)(value & 0xFF));        // lower 8 bits

    // close the I2C communication
    unsigned char error = Wire.endTransmission();

    if (error != 0)
    {
        return false;
    }

    // datasheet requires time for the new configuration to take effect.
    delay(1);

    return true;
}

// Set ZPOS

bool set_ZPOS(unsigned int position)
{
    return write_AS5600_12bit_reg(AS5600_LAR_ZPOS_STARTREG_ADDR, position);
}

// Set MPOS

bool set_MPOS(unsigned int position)
{
    return write_AS5600_12bit_reg(AS5600_LAR_MPOS_STARTREG_ADDR, position);
}

// Read RAW ANGLE (12-bit):
//   0x0C = bits 11:8
//   0x0D = bits 7:0

bool read_raw_angle(unsigned int &raw_angle)
{
    return read_AS5600_12bit_reg(AS5600_LAR_RAW_ANGLE_STARTREG_ADDR, raw_angle);
}

// Read SCALED ANGLE (different from RAW ANGLE)
// SCALED ANGLE is affected by the AS5600 configuration, such as ZPOS / MPOS / MANG

bool read_scaled_angle(unsigned int &angle)
{
    return read_AS5600_12bit_reg(AS5600_LAR_ANGLE_STARTREG_ADDR, angle);
}

// Convert raw AS5600 count to radians

double raw2rad(unsigned int raw_angle)
{
    return (double)raw_angle * AS5600_LAR_RAD_PER_COUNT;
}

// Convert raw AS5600 count to degrees

double raw2deg(unsigned int raw_angle)
{
    return (double)raw_angle * AS5600_LAR_DEG_PER_COUNT;
}

// Update continuous position
// This is the Wire equivalent of:
//     AS5600.update()
//     AS5600.getMechanicalAngle()
//     AS5600.getFullRotations()
//     AS5600.getAngle()
// The AS5600 itself only gives us a value from 0..4095,
// so we need to maintain the number of complete rotations
// in software.

bool update_sensor(void)
{
    unsigned int raw_angle;

    if (!read_raw_angle(raw_angle))
    {
        return false;
    }

    cur_raw_angle = raw_angle;

    double cur_angle_rad = raw2rad(raw_angle);

    // first valid measurement, establish the initial reference
    if (!sensor_init)
    {
        mech_angle_rad = cur_angle_rad;
        full_rot = 0;
        sensor_init = true;

        prev_angle_rad = get_position();
        prev_vel_time_us = micros();
        vel_rad_sec = 0.0;
        vel_rpm = 0.0;

        velocity_ema_initialized = false;

        return true;
    }

    // calculate the change since the previous sample (here, mech_angle_rad still contains the previous mechanical angle)
    double delta_angle = cur_angle_rad - mech_angle_rad;

    // Detect crossing of the 0 / 360-degree boundary.
    // Example:
    //   359 deg -> 1 deg
    // delta = -358 deg
    // This is actually +2 degrees and one completed rotation.
    // Conversely:
    //   1 deg -> 359 deg
    // delta = +358 deg
    // This represents -2 degrees and one reverse rotation.
    if (delta_angle > AS5600_LAR_ROTATION_WRAP_THRESHOLD)
    {
        full_rot--; // crossed 0 in the negative direction
    }
    else if (delta_angle < -AS5600_LAR_ROTATION_WRAP_THRESHOLD)
    {
        full_rot++; // crossed 0 in the positive direction
    }

    // update the angle state
    mech_angle_rad = cur_angle_rad;

    // velocity calculation
    unsigned long int now = micros();
    unsigned long int elapsed = now - prev_vel_time_us;

    if (elapsed >= VELOCITY_INTERVAL_US)
    {
        double cur_position = get_position();
        double delta_position = cur_position - prev_angle_rad;
        double dt = (double)elapsed * 1.0e-6;

        if (dt > 0.0)
        {
            vel_raw_rad_sec = delta_position / dt;
            vel_raw_rpm = vel_raw_rad_sec * 60.0 / AS5600_LAR_TWO_PI;

            if (velocity_ema_enabled)
            {
                if (!velocity_ema_initialized)
                {
                    vel_rad_sec = vel_raw_rad_sec;
                    velocity_ema_initialized = true;
                }
                else
                {
                    vel_rad_sec = velocity_ema_alpha * vel_raw_rad_sec + (1.0 - velocity_ema_alpha) * vel_rad_sec;
                }
            }
            else
            {
                vel_rad_sec = vel_raw_rad_sec;
            }

            vel_rpm = vel_rad_sec * 60.0 / AS5600_LAR_TWO_PI;
        }

        prev_angle_rad = cur_position;
        prev_vel_time_us = now;
    }

    return true;
}

// Get mechanical angle, in the range (0, 2PI) rad

double get_mechanical_angle(void)
{
    return mech_angle_rad;
}

// Get number of complete rotations

signed long int get_full_rotations(void)
{
    return full_rot;
}

// Get continuous shaft position. Equivalent to: AS5600.getFullRotations() * TWO_PI + AS5600.getMechanicalAngle()

double get_position(void)
{
    return ((double)full_rot * AS5600_LAR_TWO_PI) + mech_angle_rad;
}

// Get the latest raw sensor value

unsigned int get_raw_angle(void)
{
    return cur_raw_angle;
}

// Get calculated raw velocity in rad/s

double get_velocity_raw_rad_sec(void)
{
    return vel_raw_rad_sec;
}

// Get calculated raw velocity in RPM

double get_velocity_raw_rpm(void)
{
    return vel_raw_rpm;
}

// Get calculated velocity in rad/s

double get_velocity_rad_sec(void)
{
    return vel_rad_sec;
}

// Get calculated velocity in RPM

double get_velocity_rpm(void)
{
    return vel_rpm;
}

// Read AS5600 STATUS register
// STATUS bits:
// bit 5 = MD  Magnet detected
// bit 4 = ML  Magnet too weak
// bit 3 = MH  Magnet too strong
// byte format:
// x_x_MD_ML_MH_x_x_x

bool read_sensor_status(unsigned char &status)
{
    unsigned char data;

    if (!read_AS5600_reg(AS5600_LAR_STATUS_REG_ADDR, &data, 1))
    {
        return false;
    }

    status = data;

    return true;
}

// Initialize the AS5600 I2C interface and sensor state

bool AS5600_lar_begin(unsigned long int i2c_clock)
{
    Wire.begin();
    Wire.setClock(i2c_clock);

    sensor_init = false;

    cur_raw_angle = 0;
    mech_angle_rad = 0.0;
    full_rot = 0;
    prev_angle_rad = 0.0;
    prev_vel_time_us = 0;

    vel_raw_rad_sec = 0.0;
    vel_raw_rpm = 0.0;
    vel_rad_sec = 0.0;
    vel_rpm = 0.0;
    velocity_ema_initialized = false;

    return update_sensor();
}

// Reset velocity calculation origin

void AS5600_lar_reset_velocity_reference(void)
{
    prev_angle_rad = get_position();
    prev_vel_time_us = micros();

    vel_raw_rad_sec = 0.0;
    vel_raw_rpm = 0.0;
    vel_rad_sec = 0.0;
    vel_rpm = 0.0;
    velocity_ema_initialized = false;
}

// Configure velocity EMA filter
//
// enabled = true  -> filtered velocity is returned
// enabled = false -> raw velocity is returned
//
// alpha must satisfy:
//     0 < alpha <= 1
//
// alpha = 1.0 -> no filtering
// smaller alpha -> stronger filtering

bool set_velocity_ema_filter(bool enabled, double alpha)
{
    if (alpha <= 0.0 || alpha > 1.0)
    {
        return false;
    }

    velocity_ema_enabled = enabled;
    velocity_ema_alpha = alpha;

    // start a new EMA sequence the next time velocity is calculated.
    velocity_ema_initialized = false;

    return true;
}

// Get whether velocity EMA filtering is enabled

bool get_velocity_ema_filter_enabled(void)
{
    return velocity_ema_enabled;
}

// Get the current EMA alpha coefficient

double get_velocity_ema_filter_alpha(void)
{
    return velocity_ema_alpha;
}

// Print helpers

String fixed_uint(unsigned int value, unsigned int width)
{
    String s = String(value);
    String result;
    result.reserve(width);

    for (unsigned int i = s.length(); i < width; i++)
    {
        result += ' ';
    }

    result += s;
    return result;
}

String fixed_float(double value, unsigned int width, unsigned int decimals)
{
    char buffer[20];
    dtostrf(value, width, decimals, buffer);
    return String(buffer);
}

bool print_magnet_status(Print &out)
{
    unsigned char status;

    if (!read_sensor_status(status))
    {
        out.println(F("ERROR: could not read the AS5600 STATUS register."));
        return false;
    }

    out.print(F("STATUS = 0x"));
    out.println(status, HEX);

    if (status & 0x20)
    {
        out.println(F("Magnet detected."));
    }
    else
    {
        out.println(F("WARNING: Magnet NOT detected."));
    }
    if (status & 0x10)
    {
        out.println(F("WARNING: Magnet too weak."));
    }
    if (status & 0x08)
    {
        out.println(F("WARNING: Magnet too strong."));
    }

    return true;
}

void print_sensor_info(Print &out)
{
    out.print(F("RAW ANGLE = "));
    out.println(get_raw_angle());

    out.print(F("Angle = "));
    out.print(raw2deg(get_raw_angle()), 3);
    out.println(F(" deg"));

    print_magnet_status(out);
}

void print_velocity_ema_settings(Print &out)
{
    out.print(F("Velocity EMA = "));
    out.println(get_velocity_ema_filter_enabled() ? F("ON") : F("OFF"));

    out.print(F("Velocity EMA alpha = "));
    out.println(get_velocity_ema_filter_alpha(), 3);
}

String get_sensor_status_string(double position_zero)
{
    double position = get_position() - position_zero;

    String output;
    output.reserve(160);
    output += F("RAW=");
    output += fixed_uint(get_raw_angle(), 4);
    output += F(" | Mech=");
    output += fixed_float(get_mechanical_angle() * (180.0 / PI), 7, 2);
    output += F(" deg | Position=");
    output += fixed_float(position, 9, 4);
    output += F(" rad | Turns=");
    output += fixed_float(position / AS5600_LAR_TWO_PI, 7, 3);
    output += F(" | VelRaw=");
    output += fixed_float(get_velocity_raw_rad_sec(), 8, 3);
    output += F(" rad/s | RPMRaw=");
    output += fixed_float(get_velocity_raw_rpm(), 8, 2);
    output += F(" | VelFilt=");
    output += fixed_float(get_velocity_rad_sec(), 8, 3);
    output += F(" rad/s | RPMFilt=");
    output += fixed_float(get_velocity_rpm(), 8, 2);
    return output;
}

void print_sensor_status(Print &out, double position_zero)
{
    out.println(get_sensor_status_string(position_zero));
}
