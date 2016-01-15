#ifndef LIS3MDL_h
#define LIS3MDL_h

#include <Arduino.h> // for byte data type

class LIS3MDL
{
  public:
    template <typename T> struct vector
    {
      T x, y, z;
    };

    enum deviceType { device_MDL, device_auto };
    enum sa1State { sa1_low, sa1_high, sa1_auto };

    // register addresses
    enum regAddr
    {
	  WHO_AM_I      = 0x0F,

	  CTRL_REG1      = 0x20,
	  CTRL_REG2      = 0x21,
	  CTRL_REG3      = 0x22,
	  CTRL_REG4      = 0x23,
	  CTRL_REG5      = 0x24,

	  STATUS_REG    = 0x27,
	  OUT_X_L      = 0x28,
	  OUT_X_H      = 0x29,
	  OUT_Y_L      = 0x2A,
	  OUT_Y_H      = 0x2B,
	  OUT_Z_L      = 0x2C,
	  OUT_Z_H      = 0x2D,
	  TEMP_OUT_L    = 0x2E,
	  TEMP_OUT_H    = 0x2F,
	  INT_CFG      = 0x30,
	  INT_SRC      = 0x31,
	  INT_THIS_L   = 0x32,
	  INT_THIS_H   = 0x33,
    };

    vector<int16_t> m; // magnetometer readings
	vector<int16_t> m_max; // maximum magnetometer values, used for calibration
    vector<int16_t> m_min; // minimum magnetometer values, used for calibration

    byte last_status; // status of last I2C transmission

    LIS3MDL(void);

    bool init(deviceType device = device_auto, sa1State sa1 = sa1_auto);
    deviceType getDeviceType(void) { return _device; }

    void enableDefault(void);

    void writeReg(byte reg, byte value);
    byte readReg(byte reg);

    void read(void);

    void setTimeout(unsigned int timeout);
    unsigned int getTimeout(void);
    bool timeoutOccurred(void);
	
	float heading(void);
    template <typename T> float heading(vector<T> from);

    // vector functions
    template <typename Ta, typename Tb, typename To> static void vector_cross(const vector<Ta> *a, const vector<Tb> *b, vector<To> *out);
    template <typename Ta, typename Tb> static float vector_dot(const vector<Ta> *a, const vector<Tb> *b);
    static void vector_normalize(vector<float> *a);

  private:
      deviceType _device; // chip type
      byte address;

      unsigned int io_timeout;
      bool did_timeout;

      int testReg(byte address, regAddr reg);
};

template <typename Ta, typename Tb, typename To> void LIS3MDL::vector_cross(const vector<Ta> *a, const vector<Tb> *b, vector<To> *out)
{
  out->x = (a->y * b->z) - (a->z * b->y);
  out->y = (a->z * b->x) - (a->x * b->z);
  out->z = (a->x * b->y) - (a->y * b->x);
}

template <typename Ta, typename Tb> float LIS3MDL::vector_dot(const vector<Ta> *a, const vector<Tb> *b)
{
  return (a->x * b->x) + (a->y * b->y) + (a->z * b->z);
}

#endif


