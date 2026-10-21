#ifndef AS5600_LAR_H
#define AS5600_LAR_H

#include <Arduino.h>

// AS5600 configuration

// chip_address - I2C chip address of the magnetic sensor
// bit_resolution - resolution of the sensor (number of bits of the sensor internal counter register)
// angle_register_msb - I2C register number containing the MSB part of the angle value
// bits_used_msb - number of used bits in the I2C MSB register
//
// I2C registers are Byte (8-bit) registers.
// The register 0x0E contains the 4 MSB bits of the angle value,
// and the remaining 8 (12-4) LSB bits are contained in the
// next 0x0F register.
//
// 0x0E and 0x0F are the registers to be read to get the
// scaled angle measurement.
//
// We can also use the RAW ANGLE registers 0x0C and 0x0D,
// which carry the RAW ANGLE high byte and low byte respectively.

// Definitions

#define AS5600_LAR_CHIP_ADDR 0x36
#define AS5600_LAR_RAW_ANGLE_STARTREG_ADDR 0x0C  // RAW ANGLE MSB, 0x0D = LSB
#define AS5600_LAR_ANGLE_STARTREG_ADDR 0x0E      // SCALED ANGLE MSB, 0x0F = LSB
#define AS5600_LAR_ZPOS_STARTREG_ADDR 0x01       // ZPOS MSB, 0x02 = LSB
#define AS5600_LAR_MPOS_STARTREG_ADDR 0x03       // MPOS MSB, 0x04 = LSB
#define AS5600_LAR_STATUS_REG_ADDR 0x0B          // STATUS REGISTER carrying the byte x_x_MD_ML_MH_x_x_x
#define AS5600_LAR_CPR 4096                      // counts per revolution
#define AS5600_LAR_DEFAULT_I2C_CLOCK 400000UL    // recommended clock frequency is 400 kHz, which is commonly called Fast-mode I2C
#define AS5600_LAR_VEL_UPDATE_FREQ 100.0         // the update frequency for the velocity in Hz
#define AS5600_LAR_TWO_PI 6.28318530717958647692 // 2.0*pi

// SimpleFOC uses 0.8 * 2PI as the threshold for detecting that the sensor crossed the 0/360 degree boundary.
#define AS5600_LAR_ROTATION_WRAP_THRESHOLD (0.8 * AS5600_LAR_TWO_PI)
#define AS5600_LAR_RAD_PER_COUNT (AS5600_LAR_TWO_PI / AS5600_LAR_CPR)
#define AS5600_LAR_DEG_PER_COUNT (360.0 / AS5600_LAR_CPR)

// Initialization

bool AS5600_lar_begin(unsigned long int i2c_clock = AS5600_LAR_DEFAULT_I2C_CLOCK);
void AS5600_lar_reset_velocity_reference(void);

// Velocity EMA filter
// enabled = true -> use filtered velocity
// enabled = false -> use raw velocity
// alpha must satisfy:
//     0 < alpha <= 1
// alpha = 1.0 -> no filtering
// smaller alpha -> stronger filtering

bool set_velocity_ema_filter(bool enabled, double alpha);
bool get_velocity_ema_filter_enabled(void);
double get_velocity_ema_filter_alpha(void);

// Functions declarations

bool read_AS5600_reg(unsigned char, unsigned char *, unsigned char);
bool read_AS5600_12bit_reg(unsigned char, unsigned int &);
bool write_AS5600_12bit_reg(unsigned char, unsigned int);
bool read_raw_angle(unsigned int &);
bool read_scaled_angle(unsigned int &);
bool set_ZPOS(unsigned int);
bool set_MPOS(unsigned int);
double raw2rad(unsigned int);
double raw2deg(unsigned int);
bool update_sensor(void);
double get_mechanical_angle(void);
signed long int get_full_rotations(void);
double get_position(void);
unsigned int get_raw_angle(void);
double get_velocity_raw_rad_sec(void);
double get_velocity_raw_rpm(void);
double get_velocity_rad_sec(void);
double get_velocity_rpm(void);
bool read_sensor_status(unsigned char &);

// Print helpers
//
// Ready-made printing of the sensor data on any Print object
// (Serial, a software serial, an LCD...).

// Fixed-width strings, right-aligned and padded with spaces
String fixed_uint(unsigned int value, unsigned int width);
String fixed_float(double value, unsigned int width, unsigned int decimals);

// Reads the STATUS register and prints it with the magnet warnings
// (detected / too weak / too strong). Returns false on I2C error.
bool print_magnet_status(Print &out);

// Prints the last raw angle, the angle in degrees and the magnet status
void print_sensor_info(Print &out);

// Prints the state of the velocity EMA filter (ON/OFF and alpha)
void print_velocity_ema_settings(Print &out);

// One line with raw angle, mechanical angle, position, turns and raw/filtered velocities.
// position_zero (rad) is subtracted from the position, so a reporting zero can be used.
String get_sensor_status_string(double position_zero = 0.0);

// Prints get_sensor_status_string() followed by a newline
void print_sensor_status(Print &out, double position_zero = 0.0);

#endif