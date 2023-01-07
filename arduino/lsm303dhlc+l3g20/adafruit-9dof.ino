/*************************************NOTES*****************************************
 *
 * For reference see Adafruit official libraries: 
 * https://github.com/adafruit/Adafruit_9DOF/blob/master/Adafruit_9DOF.cpp
 * https://github.com/adafruit/Adafruit_LSM303DLHC
 * https://github.com/adafruit/Adafruit_L3GD20_U
 * https://github.com/adafruit/Adafruit_Sensor
 *
 * Datasheets: 
 * https://cdn-shop.adafruit.com/datasheets/LSM303DLHC.PDF 
 * https://cdn-shop.adafruit.com/datasheets/L3GD20.pdf
 *
 ***********************************************************************************/


#include <Wire.h>
#include <math.h>
#include "filter.h"

#define PRINT_ORIENTATION
//#define PRINT_DATA_ACCEL
//#define PRINT_DATA_MAG
//#define PRINT_DATA_GYRO
//#define CALIB
//#define USE_EULERS

#define GRAVITY_EARTH 9.80665f		// earth gravity in m/s^2
#define GAUSS_TO_MICROTESLA 100		// gaus to micro-tesla multiplier
#define DPS_TO_RADS 0.017453293f	// degrees/s to rad/s multiplier
#define RAD_TO_DGS 180 / M_PI		// rad/s to degrees/s multiplier

#define LSM303_ADDRESS_ACCEL 0x19
#define LSM303_REGISTER_CTRL_REG1_A 0x20	// control register power
//#define LSM303_REGISTER_CTRL_REG4_A 0x23	// optional enable High Resolution
#define LSM303_REGISTER_OUT_X_L_A 0x28 | 0x80 // MSB set to 1 to enable auto-increment

#define LSM303_ADDRESS_MAG 0x1E
#define LSM303_REGISTER_MR_M 0x02 
#define LSM303_REGISTER_CRA_M 0x00
#define LSM303_REGISTER_CRB_M 0x01
#define LSM303_REGISTER_OUT_X_H_M 0x03 // auto-increment enabled by default

#define L3GD20H_ADDRESS 0x69
#define L3GD20H_REGISTER_CTRL_REG1 0x20
//#define L3GD20H_REGISTER_CTRL_REG4 0x23 
#define L3GD20H_REGISTER_OUT_X_L 0x28 | 0x80 // MSB set to 1 to enable auto-increment


#define LINEAR_ACCELERATION_SENSITIVITY 0.001f // +/-2G range
#define GYRO_SENSITIVITY 0.00875f			   // 245 dps
#define MAGNETIC_GAIN_XY 1100.0f			   // +/-1.3G range XY
#define MAGNETIC_GAIN_Z 980.0f				   // +/-1.3G range Z

static int xlo, xhi, ylo, yhi, zlo, zhi;
static int raw_x, raw_y, raw_z;
static float ax, ay, az;
static float gx, gy, gz;
static float mx, my, mz;
static float bx, by, bz;

static float offset_ax, offset_ay, offset_az;
static float offset_gx, offset_gy, offset_gz;
static float offset_mx, offset_my, offset_mz;
static float scale_mx, scale_my, scale_mz;

static float accel_roll, accel_pitch, mag_heading;
static float gyro_angle_x, gyro_angle_y, gyro_angle_z;
static float roll, pitch, yaw;

static float dt;
static unsigned long time_start, time_end;

// can be used to store max and min values for the magnetometer calibration
struct mag {
	float max_x = 1; 
	float min_x = 1;
	float max_y = 1;
	float min_y = 1;
	float max_z = 1;
	float min_z = 1;
};

static quat orientation;

static mag m;

void setup()
{
	Wire.begin();
	Serial.begin(115200);
	delay(100);

	if (!start_sensor(LSM303_ADDRESS_ACCEL)) {
		Serial.print("Accelometer failed to start.\n");
		while (1);
	}

	if (!start_sensor(LSM303_ADDRESS_MAG)) {
		Serial.print("Magnetometer failed to start.\n");
		while (1);
	}

	if (!start_sensor(L3GD20H_ADDRESS)) {
		Serial.print("Gyroscope failed to start.\n");
		while (1);
	}

	calib(); 
}

void loop()
{
	time_start = millis();

	read_acc();
	read_mag();
	read_gyro();

#ifdef CALIB
	Serial.print("Raw:");
#endif

#ifdef PRINT_DATA_ACCEL
#undef PRINT_ORIENTATION
	Serial.print(ax);
	Serial.print(",");
	Serial.print(ay);
	Serial.print(",");
	Serial.println(az);
#endif

#ifdef PRINT_DATA_MAG
#undef PRINT_ORIENTATION
	Serial.print(mx);
	Serial.print(",");
	Serial.print(my);
	Serial.print(",");
	Serial.println(mz);
#endif

#ifdef PRINT_DATA_GYRO
#undef PRINT_ORIENTATION
	Serial.print(gyro_angle_x);
	Serial.print(",");
	Serial.print(gyro_angle_y);
	Serial.print(",");
	Serial.println(gyro_angle_z);
#endif

#if defined(PRINT_ORIENTATION) && defined(USE_EULERS)
	get_roll();
	get_pitch();
	get_yaw();
	Serial.print(roll);
	Serial.print(" ");
	Serial.print(pitch);
	Serial.print(" ");
	Serial.println(yaw);
#elif defined(PRINT_ORIENTATION)
	get_orientation();
	Serial.print(orientation.w);
	Serial.print(" ");
	Serial.print(orientation.x);
	Serial.print(" ");
	Serial.print(orientation.y);
	Serial.print(" ");
	Serial.println(orientation.z);
#endif

// wait for visualizer software
#ifdef USE_EULERS
	delay(50);
#else
	delay(15);
#endif
	time_end = millis();

	dt = (float)(time_end - time_start) * 10E-4;
}

bool start_sensor(byte sensor_i2c_address)
{
	byte reg_value;

	switch (sensor_i2c_address) {
	case LSM303_ADDRESS_ACCEL:
		// set normal mode 100 hz
		write_byte(sensor_i2c_address, LSM303_REGISTER_CTRL_REG1_A,
			   0x57);

		// check if powered up
		reg_value = read_byte(sensor_i2c_address,
				      LSM303_REGISTER_CTRL_REG1_A);
		if (reg_value != 0x57)
			return false;

		break;
	case LSM303_ADDRESS_MAG:
		// set continous mode
		write_byte(sensor_i2c_address, LSM303_REGISTER_MR_M, 0x00);

		// check if powered up
		reg_value =
			read_byte(sensor_i2c_address, LSM303_REGISTER_CRA_M);
		if (reg_value != 0x10)
			return false;
		
		// set range to +/- 1.3 gauss
		// write_byte(sensor_i2c_address, LSM303_REGISTER_CRB_M, 0x20);

		break;
	case L3GD20H_ADDRESS:
		// set normal mode 95 hz
		write_byte(sensor_i2c_address, L3GD20H_REGISTER_CTRL_REG1,
			   0x0F);

		// check if powered up
		reg_value = read_byte(sensor_i2c_address,
				      L3GD20H_REGISTER_CTRL_REG1);
		if (reg_value != 0x0F)
			return false;
	}
	return true;
}

// read sensor output
void read_sensor(byte sensor_i2c_address, byte output_register)
{
	Wire.beginTransmission(sensor_i2c_address);
	Wire.write(output_register);
	Wire.endTransmission();

	Wire.requestFrom(sensor_i2c_address, (uint8_t)6);

	while (Wire.available() < 6)
		;

	switch (sensor_i2c_address) {
	case LSM303_ADDRESS_ACCEL:
		xlo = Wire.read();
		xhi = Wire.read();
		ylo = Wire.read();
		yhi = Wire.read();
		zlo = Wire.read();
		zhi = Wire.read();
		raw_x = (int16_t)(xlo | (xhi << 8)) >> 4;
		raw_y = (int16_t)(ylo | (yhi << 8)) >> 4;
		raw_z = (int16_t)(zlo | (zhi << 8)) >> 4;

		// data in g's
		ax = (float)raw_x * LINEAR_ACCELERATION_SENSITIVITY;
		ay = (float)raw_y * LINEAR_ACCELERATION_SENSITIVITY;
		az = (float)raw_z * LINEAR_ACCELERATION_SENSITIVITY;

		// zero level bias correction
		ax -= offset_ax;
		ay -= offset_ay;
		az -= offset_az;

		// ax *= GRAVITY_EARTH;
		// ay *= GRAVITY_EARTH;
		// az *= GRAVITY_EARTH;
		break;
	case LSM303_ADDRESS_MAG:
		xhi = Wire.read();
		xlo = Wire.read();
		zhi = Wire.read();
		zlo = Wire.read();
		yhi = Wire.read();
		ylo = Wire.read();
		raw_x = (int16_t)(xlo | (xhi << 8));
		raw_y = (int16_t)(ylo | (yhi << 8));
		raw_z = (int16_t)(zlo | (zhi << 8));

		// data in Gauss
		mx = (float)raw_x / MAGNETIC_GAIN_XY;
		my = (float)raw_y / MAGNETIC_GAIN_XY;
		mz = (float)raw_z / MAGNETIC_GAIN_Z;

		// mx *= GAUSS_TO_MICROTESLA;
		// my *= GAUSS_TO_MICROTESLA;
		// mz *= GAUSS_TO_MICROTESLA;
		break;
	case L3GD20H_ADDRESS:
		xlo = Wire.read();
		xhi = Wire.read();
		ylo = Wire.read();
		yhi = Wire.read();
		zlo = Wire.read();
		zhi = Wire.read();
		raw_x = (int16_t)(xlo | (xhi << 8));
		raw_y = (int16_t)(ylo | (yhi << 8));
		raw_z = (int16_t)(zlo | (zhi << 8));

		// angular velocities deg/s
		gx = (float)raw_x * GYRO_SENSITIVITY;
		gy = (float)raw_y * GYRO_SENSITIVITY;
		gz = (float)raw_z * GYRO_SENSITIVITY;

#ifdef USE_EULERS
		gyro_angle_x += gx * dt;
		gyro_angle_y += gy * dt;
		gyro_angle_z += gz * dt;
#else
		gx *= DPS_TO_RADS;
		gy *= DPS_TO_RADS;
		gz *= DPS_TO_RADS;
#endif
		// zero level bias correction
		gx -= offset_gx;
		gy -= offset_gy;
		gz -= offset_gz;
		break;
	}
}

void read_acc()
{
	read_sensor(LSM303_ADDRESS_ACCEL, LSM303_REGISTER_OUT_X_L_A);
}

void read_mag()
{
	read_sensor(LSM303_ADDRESS_MAG, LSM303_REGISTER_OUT_X_H_M);
}

void read_gyro()
{
	read_sensor(L3GD20H_ADDRESS, L3GD20H_REGISTER_OUT_X_L);
}

      
// resources for implementing the complementary filter:
// https://web.archive.org/web/20091121085323/http://www.mikroquad.com/bin/view/Research/ComplementaryFilter

void get_roll()
{
	accel_roll = atan2(ay, sqrt(ax * ax + az * az));
	gyro_angle_x = 0.96 * gyro_angle_x + 0.04 * (accel_roll * RAD_TO_DGS);
	roll = gyro_angle_x;
}

void get_pitch()
{
	accel_pitch = atan2(-1.0f * ax, sqrt(ay * ay + az * az));
	gyro_angle_y = 0.96 * gyro_angle_y + 0.04 * (accel_pitch * RAD_TO_DGS);
	pitch = gyro_angle_y;
}

// no yaw drift correction implemented
void get_yaw()
{
	yaw = gyro_angle_z;
}

void get_orientation()
{
	orientation = update_filter(ax, ay, az, gx, gy, gz, mx, my, mz, dt);
}

void tilt_compensation(float roll, float pitch)
{
	//TBD
}

// used to calculate zero level offsets
void calib()
{
	//=================
	// ACCELEROMETER
	//=================

	int n = 100;
	static float sumx, sumy, sumz;

	for (int i = 1; i <= n; i++) {
		read_acc();
		sumx += ax;
		sumy += ay;
		sumz += az;

		if (i % 10 == 0) {
			Serial.println(".");
		}
	}

	offset_ax = (float)sumx / n;
	offset_ay = (float)sumy / n;
	offset_az = 1 - ((float)sumz / n);

	//=================
	// GYROSCOPE
	//=================

	sumx = 0;
	sumy = 0;
	sumz = 0;

	for (int i = 1; i <= n; i++) {
		read_gyro();
		sumx += gx;
		sumy += gy;
		sumz += gz;

		if (i % 10 == 0) {
			Serial.println(".");
		}
	}

	offset_gx = (float)sumx / n;
	offset_gy = (float)sumy / n;
	offset_gz = (float)sumz / n;

	//=================
	// MAGNETOMETER
	//=================
	
	// not implemented
	get_hard_iron(&m);
	get_soft_iron(&m);
}

void get_hard_iron(const struct mag *m)
{
	offset_mx = (m->max_x + m->min_x) * 0.5f;
	offset_my = (m->max_y + m->min_y) * 0.5f;
	offset_mz = (m->max_z + m->min_z) * 0.5f;
}

void get_soft_iron(const struct mag *m)
{
	float avg_delta_x = (m->max_x - m->min_x) * 0.5f;
	float avg_delta_y = (m->max_y - m->min_y) * 0.5f;
	float avg_delta_z = (m->max_z - m->min_z) * 0.5f;

	float avg_delta = (avg_delta_x + avg_delta_y + avg_delta_z) / 3;

	scale_mx = avg_delta / avg_delta_x;
	scale_my = avg_delta / avg_delta_y;
	scale_mz = avg_delta / avg_delta_z;
}

void write_byte(byte sensor_i2c_address, byte reg, byte value)
{
	Wire.beginTransmission(sensor_i2c_address);
	Wire.write(reg);
	Wire.write(value);
	Wire.endTransmission();
}

byte read_byte(byte sensor_i2c_address, byte reg)
{
	byte value;

	Wire.beginTransmission(sensor_i2c_address);
	Wire.write(reg);
	Wire.endTransmission();

	Wire.requestFrom(sensor_i2c_address, (uint8_t)1);

	while (Wire.available() < 1);

	value = Wire.read();

	return value;
}
