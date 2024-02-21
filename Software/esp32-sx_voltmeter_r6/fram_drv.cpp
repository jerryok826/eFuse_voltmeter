#include <Wire.h>
//#include <Adafruit_GFX.h>
#include <Arduino.h>

// indent -gnu -br -cli2 -lp -nut -l100 fram_drv.cpp
// https://nightshade.net/knowledge-base/electronics-tutorials/arduino/intro-to-arduino-i2c-serial-communication-links/
// https://cdn-shop.adafruit.com/product-files/1895/MB85RC256V-DS501-00017-3v0-E.pdf
// https://github.com/sosandroid/FRAM_MB85RC_I2C/blob/master/FRAM_MB85RC_I2C.cpp#L673

int fram__get_dev_id (void);
void i2c_scan (void);
int fram_read_bytes (uint16_t fram_adr, uint8_t * ptr, int len);
int fram_write_bytes (uint16_t fram_adr, uint8_t * ptr, int len);

void
i2c_scan (void)
{
  uint8_t error;

  Serial.begin (115200);
  Serial.printf ("%s(%d)\n", __func__, __LINE__);

  /// Fram test  
  Wire.begin ();

  Serial.printf ("I2C Scan\n");

  uint8_t address;
  int nDevices;
  Serial.printf ("Scanning...\n");
  nDevices = 0;
  for (address = 1; address < 127; address++) {
    Wire.beginTransmission (address);
    error = Wire.endTransmission ();
    if (error == 0) {
      Serial.print ("I2C device found at address 0x");
      if (address < 16) {
        Serial.print ("0");
      }
      Serial.println (address, HEX);
      nDevices++;
    }
    else if (error == 4) {
      Serial.print ("Unknow error at address 0x");
      if (address < 16) {
        Serial.print ("0");
      }
      Serial.println (address, HEX);
    }
  }
  if (nDevices == 0) {
    Serial.println ("No I2C devices found\n");
  }
  else {
    Serial.println ("done\n");
  }
}

#define FRAME_MASTER_CODE (0xF8)
#define FRAME_I2C_ADR (0x50)
int
fram_get_dev_id (void)
{
  uint8_t error;
  uint8_t rx[4];
  int manufacture_id = 0;
  int product_id = 0;

  Serial.printf ("%s(%d)\n", __func__, __LINE__);

  Wire.begin ();

  Wire.beginTransmission (0xF8 >> 1);   // Should use MASTER_CODE
  Wire.write (FRAME_I2C_ADR << 1);      // Write the register address that we want to target
  error = Wire.endTransmission (0);

  Wire.requestFrom (FRAME_MASTER_CODE >> 1, 3);       // Read 3 byte from slave address

  for (int i = 0; i < 4; i++) {
    rx[i] = Wire.read ();       // Read bytes from registers
  }
  Wire.endTransmission ();

  manufacture_id = rx[1] >> 4;
  product_id = ((rx[1] & 0x0f) << 8) | rx[2];
  Serial.printf ("Fram ID Bytes: 0x%02X, 0x%02X, 0x%02X\n", rx[0], rx[1], rx[2]);
  Serial.printf ("manufacture_id: 0x%04X, product_id: 0x%02X\n", manufacture_id, product_id);

  Serial.printf ("%s(%d)\n", __func__, __LINE__);
  if ((manufacture_id == 0x0A) && (product_id == 0x358)) {
    return 0;
  }
  else {
    return -1;
  }
}

int
fram_read_bytes (uint16_t fram_adr, uint8_t * ptr, int len)
{
  unsigned long start_time = micros();
  unsigned long stop_time;

  Wire.begin ();
  //Wire.setClock(800000);

  Wire.beginTransmission (FRAME_I2C_ADR);       // Begin communication with slave address 85 (0x55) 
  Wire.write (fram_adr >> 8);
  Wire.write (fram_adr & 0xFF); // Write the value that we want in register 12 
  Wire.endTransmission (0);     // Send the restart condition

  Wire.requestFrom (FRAME_I2C_ADR, len);        //
  for (int i = 0; i < len; i++) {
    ptr[i] = Wire.read ();      // Read bytes from registers
  }
  stop_time = micros() - start_time;

  Serial.printf ("%s(%d) Adr 0x%04X: 0x%02X 0x%02X 0x%02X 0x%02X, %ld us\n", __func__, __LINE__, fram_adr,
                 ptr[0], ptr[1], ptr[2], ptr[3], stop_time);
  return 0;
}

int
fram_write_bytes (uint16_t fram_adr, uint8_t * ptr, int len)
{
//  Serial.printf ("%s(%d) Adr 0x%04X: 0x%02X 0x%02X 0x%02X 0x%02X\n", __func__, __LINE__, fram_adr,
//                 ptr[0], ptr[1], ptr[2], ptr[3]);
  unsigned long start_time = micros();
  unsigned long stop_time;

  Wire.begin ();
  //Wire.setClock(800000);

  Wire.beginTransmission (FRAME_I2C_ADR);       // Begin communication with slave address 85 (0x55) 
  Wire.write (fram_adr >> 8);
  Wire.write (fram_adr & 0xFF); // Write the value that we want in register 12 
  for (int i = 0; i < len; i++) {
    Wire.write (ptr[i]);        // Read bytes from registers
  }
  Wire.endTransmission ();      // Send the stop condition and end the transaction
  ptr[0]++;
  stop_time = micros() - start_time;
  Serial.printf ("%s(%d) Adr 0x%04X: 0x%02X 0x%02X 0x%02X 0x%02X, %ld us\n", __func__, __LINE__, fram_adr,
                 ptr[0], ptr[1], ptr[2], ptr[3], stop_time);

  uint8_t rd_back[10];
  fram_read_bytes (fram_adr, rd_back, len);
  if (ptr[0] != rd_back[0]) {
      Serial.printf ("%s(%d) ReadBack err 0x%04X: 0x%02X 0x%02X\n", __func__, __LINE__, fram_adr, ptr[0], rd_back[0]);
  }
  return 0;
}

#if 0
void
reg_dump (int reg)
{
  int reg_value = 0x73;

  reg_value = ina228.reg_read (reg);    // jok
  Serial.printf ("INA228 reg[%02X] 0x%04X\n", reg, reg_value);
}
#endif

#define INA228_I2C_ADR (0x40)
int
ina228_reg_read(int reg)
{
  uint8_t data[4];
  uint16_t reg_value=0;

  Wire.begin ();
//  Wire.setClock(100000);

  Wire.beginTransmission (INA228_I2C_ADR);  // Begin communication with slave address (0x40) 
  Wire.write (reg);
  Wire.endTransmission (0);     // Send the restart condition

  Wire.requestFrom (INA228_I2C_ADR, 3);        //
  data[0] = Wire.read ();      // Read bytes from registers
  data[1] = Wire.read ();      // Read bytes from registers

  reg_value = (data[0]<<8) | data[1];
//  Serial.printf ("INA228 reg[%02X] 0x%04X\n", reg, reg_value);
  return reg_value;
}

int
ina228_reg_write(int reg, uint16_t reg_value)
{
  Serial.printf ("INA228 reg[%02X] 0x%04X\n", reg, reg_value);

  Wire.begin ();

  Wire.beginTransmission (INA228_I2C_ADR);       // Begin communication with slave address 85 (0x55) 
  Wire.write (reg); // send reg adr
  Wire.write (reg_value>>8); // Write the value that we want in register 12 
  Wire.write (reg_value & 0xFF); // Write the value that we want in register 12 
  Wire.endTransmission ();      // Send the stop condition and end the transaction

  return 0;
}
