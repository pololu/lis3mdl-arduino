#include <LIS3MDL.h>
#include <Wire.h>
#include <math.h>

// Defines ////////////////////////////////////////////////////////////////

// The Arduino two-wire interface uses a 7-bit number for the address,
// and sets the last bit correctly based on reads and writes
#define SA1_HIGH_ADDRESS      0b0011110 
#define SA1_LOW_ADDRESS       0b0011100 

#define TEST_REG_ERROR -1

#define WHO_ID     0x3D

// Constructors ////////////////////////////////////////////////////////////////

LIS3MDL::LIS3MDL(void)
{
    /*
  These values lead to an assumed magnetometer bias of 0.
  Use the Calibrate example program to determine appropriate values
  for your particular unit. The Heading example demonstrates how to
  adjust these values in your own sketch.
  */
  m_min = (LIS3MDL::vector<int16_t>){-32767, -32767, -32767};
  m_max = (LIS3MDL::vector<int16_t>){+32767, +32767, +32767};
  
  _device = device_auto;

  io_timeout = 0;  // 0 = no timeout
  did_timeout = false;
}

// Public Methods //////////////////////////////////////////////////////////////

// Did a timeout occur in read() since the last call to timeoutOccurred()?
bool LIS3MDL::timeoutOccurred()
{
  bool tmp = did_timeout;
  did_timeout = false;
  return tmp;
}

void LIS3MDL::setTimeout(unsigned int timeout)
{
  io_timeout = timeout;
}

unsigned int LIS3MDL::getTimeout()
{
  return io_timeout;
}

bool LIS3MDL::init(deviceType device, sa1State sa1)
{
  int id;
  
  // perform auto-detection unless device type and SA1 state were both specified
  if (device == device_auto || sa1 == sa1_auto)
  {
    // check for L3GD20H, D20 if device is unidentified or was specified to be one of these types
    if (device == device_auto)
    {
      // check SA1 high address unless SA1 was specified to be low
      if (sa1 != sa1_low && (id = testReg(SA1_HIGH_ADDRESS, WHO_AM_I)) != TEST_REG_ERROR)
      {    
        sa1 = sa1_high;
        if (device == device_auto)
        {
          device = device_MDL;
        }
      }
      // check SA1 low address unless SA1 was specified to be high
      else if (sa1 != sa1_high && (id = testReg(SA1_LOW_ADDRESS, WHO_AM_I)) != TEST_REG_ERROR)
      {   
        sa1 = sa1_low;
        if (device == device_auto)
        {
          device = device_MDL;
        }
      }
    }
    
    // make sure device and SA1 were successfully detected; otherwise, indicate failure
    if (device == device_auto || sa1 == sa1_auto)
    {
      return false;
    }
  }
  
  _device = device;

  // set device address
  address = (sa1 == sa1_high) ? SA1_HIGH_ADDRESS : SA1_LOW_ADDRESS;
  
  return true;
}

/*
Enables the LIS3MDL's Magnetometer. Also:
- Enables temp sensor and sets mag to ultra-high-performance mode for all axes
- Selects 80 Hz ODR (output data rate).
- Sets mag full scale (gain) to ±12 gauss
- Select continuous conversion mode
Note that this function will also reset other settings controlled by
the registers it writes to.
*/
void LIS3MDL::enableDefault(void)
{
  // 0xFC = 0b11111100
  // Enables temp sensor and sets mag to ultra-high-performance mode for X and Y
  // Selects 80 Hz ODR (output data rate)
  writeReg(CTRL_REG1, 0xFC);
  
  // 0x40 = 0b01000000
  // Sets mag full scale (gain) to ±12 gauss
  writeReg(CTRL_REG2, 0x40);
  
  // 0x00 = 0b00000000
  // Select continuous conversion mode
  writeReg(CTRL_REG3, 0x00);
  
  // 0x0C = 0b00001100
  // Sets mag to ultra-high-performance mode for Z
  writeReg(CTRL_REG4, 0x0C);
}

// Writes a mag register
void LIS3MDL::writeReg(byte reg, byte value)
{
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(value);
  last_status = Wire.endTransmission();
}

// Reads a mag register
byte LIS3MDL::readReg(byte reg)
{
  byte value;

  Wire.beginTransmission(address);
  Wire.write(reg);
  last_status = Wire.endTransmission();
  Wire.requestFrom(address, (byte)1);
  value = Wire.read();
  Wire.endTransmission();

  return value;
}

// Reads the 3 mag channels and stores them in vector m
void LIS3MDL::read()
{
  Wire.beginTransmission(address);
  Wire.write(OUT_X_L | 0x80);
  Wire.endTransmission();
  Wire.requestFrom(address, (byte)6);
  
  unsigned int millis_start = millis();
  while (Wire.available() < 6)
  {
    if (io_timeout > 0 && ((unsigned int)millis() - millis_start) > io_timeout)
    {
      did_timeout = true;
      return;
    }
  }

  uint8_t xlm = Wire.read();
  uint8_t xhm = Wire.read();
  uint8_t ylm = Wire.read();
  uint8_t yhm = Wire.read();
  uint8_t zlm = Wire.read();
  uint8_t zhm = Wire.read();

  // combine high and low bytes
  m.x = (int16_t)(xhm << 8 | xlm);
  m.y = (int16_t)(yhm << 8 | ylm);
  m.z = (int16_t)(zhm << 8 | zlm);
}

/*
Returns the angular difference in the horizontal plane between a
default vector and north, in degrees.

The default vector here is chosen to point along the surface of the
PCB, in the direction of the top of the text on the silkscreen.
This is the +X axis on the Pololu LSM303D carrier and the -Y axis on
the Pololu LSM303DLHC, LSM303DLM, and LSM303DLH carriers.
*/
float LIS3MDL::heading(void)
{
  if (_device == device_MDL)
  {
    return heading((vector<int>){1, 0, 0});
  }
  else
  {
    return heading((vector<int>){0, -1, 0});
  }
}

void LIS3MDL::vector_normalize(vector<float> *a)
{
  float mag = sqrt(vector_dot(a, a));
  a->x /= mag;
  a->y /= mag;
  a->z /= mag;
}

// Private Methods //////////////////////////////////////////////////////////////

int LIS3MDL::testReg(byte address, regAddr reg)
{
  Wire.beginTransmission(address);
  Wire.write((byte)reg);
  if (Wire.endTransmission() != 0)
  {
    return TEST_REG_ERROR;
  }

  Wire.requestFrom(address, (byte)1);
  if (Wire.available())
  {
    return Wire.read();
  }
  else
  {
    return TEST_REG_ERROR;
  }
}
