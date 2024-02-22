#include <SPI.h>
#include <Wire.h>
#include <Toggle.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>    // Hardware-specific library for ST7789
#include <Adafruit_ADS1X15.h>
#include "Adafruit_INA228.h"

#define ESP32 ///
//#define ENCODER_DO_NOT_USE_INTERRUPTS
#define ENCODER_USE_INTERRUPTS
#include <Encoder.h>  // https://github.com/PaulStoffregen/Encoder/
// ~/Arduino/libraries/Adafruit-GFX-Library/Fonts/
#include <Fonts/FreeSans12pt7b.h>
#include <Fonts/FreeMono18pt7b.h>
#include <Fonts/FreeMonoBold18pt7b.h>

// https://github.com/adafruit/Adafruit_INA228

// indent -gnu -br -cli2 -lp -nut -l100 esp32-s3_voltmeter_r4.ino

Adafruit_ST7789 display = Adafruit_ST7789 (TFT_CS, TFT_DC, TFT_RST);
GFXcanvas16 canvas (240, 135);

int fram_get_dev_id (void);
void i2c_scan (void);
int fram_read_bytes (uint16_t fram_adr, uint8_t * ptr, int len);
int fram_write_bytes (uint16_t fram_adr, uint8_t * ptr, int len);
int ina228_reg_read (int reg);
int ina228_reg_write(int reg, uint16_t reg_value);

Adafruit_INA228 ina228 = Adafruit_INA228 ();

const float sense_resistor = 0.015;
const float fet_on_resistance = 0.025;

// GPIO outputs
const int OC_led = 12;          // 7;       // GPIO7
const int OV_led = 11;          //8;        // GPIO
const int OUTPUT_led = 10;      // 10;      // GPIO
const int BUZZER = 15;          // 6;       // GPIO
const int FETDRV = 14;          // GPIO
const int INA228_ALR = 13;      // GPIO13  INA228_ALR

// GPIO inputs
const int OP_SW = 9;            // 18;       // GPIO18 ??
const int OV_ENC_SW = 16;       //12; // 25; // GPIO25 ??
const int OC_ENC_SW = 8;        //12; // 25; // GPIO25 ??

const int OV_ENC_A = 18; 
const int OV_ENC_B = 17;  
const int OC_ENC_A = 6; 
const int OC_ENC_B = 5; 

int ina228_alr_int_flag=0; // int flag

static int output_state = 0;
static int fault_state = 0;
static float voltage_limit = 0.0;
static float current_limit = 0.0;
static float oc_pot_setting, ov_pot_setting;
static int ov_fault = 0;
static int oc_fault = 0;

static float max_voltage_limit = 20.0;
static float max_current_limit = 10.0;  // normal value of 4A
static float fault_voltage = 0.0;
static float fault_current = 0.0;

long Position_Diff = 0.0;
static int dirty_ov_setting = 0;
static int dirty_oc_setting = 0;

#define DEFAULT_OV_LIMIT  (13.5)
#define DEFAULT_OC_LIMIT  (2.0)

// Change these two numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
//   avoid using pins with LEDs attached

Encoder OC_Enc(OC_ENC_A,OC_ENC_B); // 5,6); // 5, 6);
Encoder OV_Enc(OV_ENC_B,OV_ENC_A); // 17,18); // 5, 6);

static signed long old_OV_Position = 0;
static signed long new_OV_Position = 0;

static long old_OC_Position = 0;
static long new_OC_Position = 0;

#define OV_FAULT_BIT  (0x1 << 4)
#define OC_FAULT_BIT  (0x1 << 6)

void led_test (int state);
void alert_bell (int state);
void fet_drv_state (int state);
void output_led_set (int state);
void ov_led_set (int state);
void oc_led_set (int state);
void buzzer_set (int state);

Toggle button (OP_SW);

void
reg_dump (int reg)
{
  int reg_value = 0x73;

  reg_value = ina228_reg_read (reg); 
//  reg_value = ina228.reg_read (reg);  
  Serial.printf ("INA228 reg[%02X] 0x%04X\n", reg, reg_value);
}

void
output_led_set (int state)
{
  pinMode (OUTPUT_led, OUTPUT);
  if (state == 0) {
    digitalWrite (OUTPUT_led, HIGH);  // LED off
  }
  else {
    digitalWrite (OUTPUT_led, LOW); // LED on
  }
}

void
ov_led_set (int state)
{
  pinMode (OV_led, OUTPUT);
  if (state == 0) {
    digitalWrite (OV_led, HIGH);  // LED off
    ov_fault = 0;
  }
  else {
    digitalWrite (OV_led, LOW); // LED on
    ov_fault = 1;
  }
}

void
oc_led_set (int state)
{
  pinMode (OC_led, OUTPUT);
  if (state == 0) {
    digitalWrite (OC_led, HIGH); // LED off
    oc_fault = 0;
  }
  else {
    digitalWrite (OC_led, LOW); // LED on
    oc_fault = 1;
  }
}

void
buzzer_set (int state)
{
  pinMode (BUZZER, OUTPUT);
  if (state == 0) {
    digitalWrite (BUZZER, LOW); // off
  }
  else {
    digitalWrite (BUZZER, HIGH); // on
  }
}

void
voltage_limit_set (float voltage)
{
  int reg_value = 0;

  // set voltage limit
  reg_value = (int) (voltage / 0.003125);
//  ina228.reg_write (0x0e, reg_value);
  ina228_reg_write(0x0e, reg_value);

  Serial.printf ("%s(%d) V Limit set: %03.3fV, reg[%d]: 0x%04X\n", __func__,__LINE__,voltage, 0x0e, reg_value);

  // set enable V fault
  reg_value = ina228_reg_read (0x0B); 
//  reg_value = ina228.reg_read (0x0B);   // jok
  reg_value &= (1 << 6); // Clear all but OC flag
  reg_value |= 1 << 4;          // bus over voltage
  reg_value |= 1 << 15;         // Set Alert latch bit
//  ina228.reg_write (0x0b, reg_value);
  ina228_reg_write(0x0b, reg_value);

  Serial.printf ("%s(%d) over V alert enabled: %03.3fV, reg[%d]: 0x%04X\n", __func__,__LINE__,voltage, 0x0B, reg_value);
}

void
current_limit_set (float current)
{
  int reg_value = 0;
  float shunt_voltage;

  // set current limit
  shunt_voltage = current * sense_resistor;
//  ina228.reg_write (0x0c, (float) (shunt_voltage / 0.000005));
  ina228_reg_write (0x0c, (float) (shunt_voltage / 0.000005));
  Serial.printf ("%s(%d) I Limit set: %03.3fV, reg[%d]: 0x%04X\n", __func__,__LINE__,current, 0x0c, reg_value);

  // set enable V fault
  reg_value = ina228_reg_read (0x0B); 
//  reg_value = ina228.reg_read (0x0B);   // jok
  reg_value &= (1 << 4); // Clear all but OV flag
  reg_value |= 1 << 6;          // bus over current
  reg_value |= 1 << 15;         // Set Alert latch bit
//  ina228.reg_write (0x0B, reg_value);
  ina228_reg_write(0x0b, reg_value);

  Serial.printf ("%s(%d) I Limit set: %03.3fA, reg[%d]: 0x%04X\n", __func__,__LINE__, current, 0x0c, reg_value);
}

void
output_pin_test (int pin_to_test)
{
  int toggle_bit = 0;
  pinMode (pin_to_test, OUTPUT);

  while (1) {
    if (toggle_bit) {
      digitalWrite (pin_to_test, HIGH);
      toggle_bit = !toggle_bit;
      delay (1);
    }
    else {
      digitalWrite (pin_to_test, LOW);
      toggle_bit = !toggle_bit;
      delay (1);
    }
  }
}

// switch test,  AL_RST_SW
//    alarm_reset_sw_state(void)
void
sw_test (int pin_nub)           // AL_RST_SW
{
  pinMode (pin_nub, INPUT_PULLUP);
  while (1) {
    if (digitalRead (pin_nub) == 0) {
      oc_led_set (0);
      ov_led_set (1);
    }
    else {
      oc_led_set (1);
      ov_led_set (0);
    }
    delay (10);
  }
}

void
alert_bell (int state)
{
  if (state) {
    buzzer_set (1);
  }
  else {
    buzzer_set (0);
  }
}

void
fet_drv_state (int state)
{
  pinMode (FETDRV, OUTPUT);
  if (state) {
    output_state = 1;
    output_led_set (1);
    digitalWrite (FETDRV, HIGH); // on
  }
  else {
    output_state = 0;
    output_led_set (0);
    digitalWrite (FETDRV, LOW); // off
  }
}

void
led_test (int state)
{
  if (state) { // on
    output_led_set (1); 
    oc_led_set (1);
    ov_led_set (1);
  }
  else {
    output_led_set (0); 
    oc_led_set (0);
    ov_led_set (0);
  }
}

void
read_voltage_current (float *voltage, float *current)
{
  float fet_sense_r_V_drop = 0.0;

  *voltage = ina228.readBusVoltage () / 1000.0;
  *current = ina228.readCurrent () / 1000.0;    // sensor current reading.
  *current -= *voltage / 1000;  // adjust out 1K load resistor current.

  fet_sense_r_V_drop = (*current) * (sense_resistor + fet_on_resistance); // ~0.035 ohms
  *voltage -= fet_sense_r_V_drop;
}

// return true if reset sw pressed
int
alarm_reset_sw_state (void)
{
  if ((digitalRead (OV_ENC_SW) == 0) || (digitalRead (OC_ENC_SW) == 0)) { 
    return 1;
  }
  else {
    return 0;
  }
}

void
display_limits (void)
{
  canvas.fillScreen (ST77XX_BLACK);     // SH110X_BLACK);
  canvas.setFont (&FreeSans12pt7b);     // FreeMono18pt7b
  canvas.setCursor (0, 0);
  canvas.setTextColor (ST77XX_WHITE);   // ST77XX_RED ST77XX_YELLOW ST77XX_GREEN ST77XX_BLUE ST77XX_WHITE
  canvas.printf ("\n           V&C Limits\n");
  canvas.setTextColor (ST77XX_GREEN);
  canvas.printf (" %19.2fV\n", voltage_limit);
  canvas.setTextColor (ST77XX_YELLOW);
  canvas.printf (" %19.2fA\n", current_limit);
  display.drawRGBBitmap (0, 0, canvas.getBuffer (), 240, 135);
}

void
display_readings (void)
{
  float voltage = 0.0;
  float current = 0.0;
  float temperature = 0.0;
  static float last_current = 0.0;

  read_voltage_current (&voltage, &current);
  if (fault_state) {
//    voltage = fault_voltage;  // I don't think this is needed
    current = fault_current;    // Show fault current value before output this abled.
  }
  temperature = ina228.readDieTemp ();

  canvas.setFont (&FreeMonoBold18pt7b); // FreeSans12pt7b); // FreeMono18pt7b
  canvas.fillScreen (ST77XX_BLACK);     // SH110X_BLACK);
  canvas.setCursor (0, 0);
  canvas.setTextColor (ST77XX_GREEN);
  if (ov_fault) {
    canvas.setTextColor (ST77XX_RED);
  }
  // screen is 4 rows by 18 chars
  canvas.printf ("\n%6.2fV OUT\n", voltage);
  // show current
  if (current < 0.0) {
    current = 0.0;
  }
  canvas.setTextColor (ST77XX_GREEN);
  if (oc_fault) {
    canvas.setTextColor (ST77XX_RED);
  }
  if (output_state) {
    canvas.printf ("%6.2fA ON\n", current);
  } else {
    canvas.printf ("%6.2fA OFF\n", current);
  }
  canvas.setTextColor (ST77XX_GREEN);

  canvas.setTextColor (ST77XX_YELLOW); // output_state = 1;
  canvas.setFont (&FreeSans12pt7b);
  canvas.printf (" %12.2fV Limits\n", voltage_limit);
  canvas.printf (" %14.2fA  %02.1fC\n", current_limit,temperature );

//  canvas.printf ("012345678901234567\n");
  display.drawRGBBitmap (0, 0, canvas.getBuffer (), 240, 135);
}

void
display_update (void)
{
  float voltage = 0.0;
  float current = 0.0;
  float temperature = 0.0;
  static float last_current = 0.0;

  read_voltage_current (&voltage, &current);
  temperature = ina228.readDieTemp ();

  if (alarm_reset_sw_state ()) {        // check reset button
    display_limits ();
  }
  else {
    display_readings ();
  }
//  Serial.printf(" %3.3fV,  %3.3fA, %3.1f C\n",voltage,current,temperature);
}

// #define DEFAULT_OV_LIMIT  (13.5)
// #define DEFAULT_OC_LIMIT  (2.0)
float getVoltageSetting()
{
  unsigned long start_time = micros();
  uint32_t voltage=0;
  float voltage_limit;

  fram_read_bytes (0, (uint8_t *)&voltage, sizeof(voltage));
  voltage_limit = ((float)voltage) /  1000.0;
  if (voltage_limit > DEFAULT_OV_LIMIT) {
      Serial.printf("%s(%d) Out of range Voltage limit from FRAM: %3.3f, %ld us\n", __func__,__LINE__,voltage_limit, micros() - start_time);
      voltage_limit = DEFAULT_OV_LIMIT;
      saveVoltageSetting(voltage_limit);
  } 
  Serial.printf("%s(%d) Voltage limit from FRAM: %3.3f, %ld us\n", __func__,__LINE__,voltage_limit, micros() - start_time);
  return voltage_limit;
}

void saveVoltageSetting(float voltage_limit)
{
//  unsigned long start_time = micros();
  uint32_t voltage=0;

  voltage = voltage_limit * 1000.0;
  fram_write_bytes (0, (uint8_t *)&voltage, sizeof(voltage));

//  Serial.printf("%s(%d) Voltage_limit set value: %3.3f, %ld us\n", __func__,__LINE__,voltage_limit, micros() - start_time);
}

float getCurrentSetting()
{
//  unsigned long start_time = micros();
  uint32_t current;
  float current_limit;

  fram_read_bytes (4, (uint8_t *)&current, sizeof(current));
  current_limit = ((float)current) /  1000.0;

//  Serial.printf("%s(%d) current_limit get value: %3.3f, %ld us\n", __func__,__LINE__,current_limit, micros() - start_time);
  return current_limit;
}

void saveCurrentSetting(float current_limit)
{
//  unsigned long start_time = micros();
  uint32_t current=0;

  current = current_limit * 1000.0;
  fram_write_bytes (4, (uint8_t *)&current, sizeof(current));

//  Serial.printf("%s(%d) current_limit set value: %3.3f, %ld us\n", __func__,__LINE__,current_limit, micros() - start_time);
}

int isr_cntr=0;

void IRAM_ATTR isr() {
//    ina228_reg_read (0x0B); // read alert reg to clearit
    ina228_alr_int_flag=1;
    isr_cntr++;
}

void
setup ()
{
  Serial.begin (115200);

  output_led_set (0); 
  ov_led_set (0); 
  oc_led_set (0); 
  buzzer_set (0);
  fet_drv_state (0);

  delay(1000);
  Serial.printf("=====================================================\n");
  Serial.printf("=====================================================\n");
  Serial.printf("eFuse_voltmeter Init, Built: %s %s\n",__DATE__,__TIME__);

  // Dont know way this is required, Without it OV enc does not work. GPIO18
  //pinMode (OV_ENC_A, INPUT);  

  button.begin (OP_SW);
  button.setToggleState (0);    // set initial state 0 or 1
  button.setToggleTrigger (0);  // set trigger onPress: 0, or onRelease: 1

  // turn backlite on later
  pinMode (TFT_BACKLITE, OUTPUT);
  digitalWrite (TFT_BACKLITE, LOW); // Keep backlight off for now

  // turn on the TFT / I2C power supply
  pinMode (TFT_I2C_POWER, OUTPUT);
  digitalWrite (TFT_I2C_POWER, HIGH);

  pinMode (OP_SW, INPUT_PULLUP);
  pinMode (OV_ENC_SW, INPUT_PULLUP); // AL_RST_SW, INPUT);
  pinMode (OC_ENC_SW, INPUT_PULLUP); // AL_RST_SW, INPUT);

  // Set encoder inputs
#if 0
//  pinMode (OV_ENC_B, INPUT_PULLUP); // Does not cause ENC problems if commented out
  pinMode (OV_ENC_A, INPUT); //18, INPUT); // Voltage ENC, Not sure if or why we need this

//  pinMode (OC_ENC_B, INPUT_PULLUP); // Does not cause ENC problems if commented out
//  pinMode (OC_ENC_A, INPUT_PULLUP); // Does not cause ENC problems if commented out
#endif
  
//  i2c_scan (); // debug routines
//  fram_get_dev_id ();

  // Turn LEED and Buzzer on to sound reboot
  oc_led_set (1); // Turn on fault Leds, boot up test
  ov_led_set (1); 
  output_led_set (1); 
  buzzer_set (0); // Keep buzzer off

// If fault switch is press during boot, than preset fault limts
  if (alarm_reset_sw_state ()) {
      saveVoltageSetting(DEFAULT_OV_LIMIT);
      saveCurrentSetting(DEFAULT_OC_LIMIT);
      Serial.printf("Fault SW pressed, Used preset V & I limits.\n");
  }
  Serial.println ("Init I2C Chips");

#if 0
  Serial.println("Basic Encoder Test:");
  while(1) {
    // over voltage encoder test
    new_OV_Position = OV_Enc.read();
    if (new_OV_Position != old_OV_Position) {
       old_OV_Position = new_OV_Position;
       Serial.printf("OV_Enc: %d\n",new_OV_Position);
    }

    // over current encoder test
    new_OC_Position = OC_Enc.read();
    if (new_OC_Position != old_OC_Position) {
       old_OC_Position = new_OC_Position;
       Serial.printf("OC_Enc: %d\n",new_OC_Position);
    }
  }
#endif

  // Dump some INA228 ID regs, for debug
  ina228_reg_read (0x01);  // ADC config reg, 0xFB668
  ina228_reg_read (0x3E);  // manufacturing ID reg, 0x5449
  ina228_reg_read (0x3F);  // Devive ID reg, 0x2281

  if (!ina228.begin ()) {
    Serial.println ("Couldn't find INA228 chip");
  } else {
    Serial.println ("Found INA228 chip");
  }

  // INA228 board config
  ina228.setShunt (sense_resistor, 10.0);       // updated to match MY board
  ina228.setAveragingCount (INA228_COUNT_16);
  uint16_t counts[] = { 1, 4, 16, 64, 128, 256, 512, 1024 };
  //Serial.print ("Averaging counts: ");
  //Serial.println (counts[ina228.getAveragingCount ()]);

  // set the time over which to measure the current and bus voltage
  ina228.setVoltageConversionTime (INA228_TIME_150_us);
  ina228.setCurrentConversionTime (INA228_TIME_280_us);

  // default polarity for the alert is low on ready, but
  // it can be inverted!
  ina228.setAlertPolarity (INA228_ALERT_POLARITY_INVERTED); // Active low open-collector

  display.init (135, 240);      // Init ST7789 240x135
  display.setRotation (3);
  canvas.setFont (&FreeSans12pt7b);
  canvas.setTextColor (ST77XX_WHITE);
  canvas.fillScreen (ST77XX_BLACK);
  canvas.setCursor (0, 0); // was 0,25
//    canvas.setTextSize (0);      // 2);
//  canvas.setFont (&FreeSans12pt7b);
  canvas.setFont (&FreeMonoBold18pt7b); // FreeSans12pt7b); // FreeMono18pt7b

  canvas.setTextColor (ST77XX_WHITE);
  canvas.printf ("\n   eFuse\n");
  canvas.printf (" Voltmeter");
  display.drawRGBBitmap (0, 0, canvas.getBuffer (), 240, 135);
  digitalWrite (TFT_BACKLITE, HIGH);    // turn backlight on
  Serial.printf("Display Init done\n");

  delay (1000);

  alert_bell (1);
  delay (50);
  alert_bell (0);
  led_test (0);

  fault_state = 0;
  fet_drv_state (0);
  ov_led_set (0);
  oc_led_set (0);

  // reload old V & C limits
//#define DEFAULT_OV_LIMIT  (13.5)
//#define DEFAULT_OC_LIMIT  (2.0)
// static float max_voltage_limit = 20.0;
// static float max_current_limit = 10.0;  // normal value of 4A

  voltage_limit = getVoltageSetting(); // from FRAM
  if (voltage_limit > max_voltage_limit) { // 20.0
      Serial.printf("Out of range Volt limit: %3.2fV, Used default value\n", voltage_limit);
      voltage_limit = DEFAULT_OV_LIMIT;
  }
  current_limit = getCurrentSetting();
  if (current_limit > max_current_limit) {  // 10.0
      Serial.printf("Out of range current limit: %3.2fA, Used default value\n", current_limit);
      current_limit = DEFAULT_OC_LIMIT;    
  }
  
  Serial.printf("Using saved Voltage limit: %3.2fV\n", voltage_limit);
  Serial.printf("Using saved Current limit: %3.2fA\n", current_limit );

  voltage_limit_set (voltage_limit);
  current_limit_set (current_limit);

  Serial.printf("Setup done, jump to Loop\n");

  // enable fault interrupt
  ina228_alr_int_flag=0;
  pinMode(INA228_ALR, INPUT_PULLUP);
  attachInterrupt(INA228_ALR, isr, FALLING);
}

void
loop ()
{
  static int display_update_time = 0;
  static int pot_update_time = 0;
  static int fault_update_time = 0;
  static int settings_update_time = 0;
  static bool op_sw_state = 0;
  int alert_status = 0;

  if ((millis () - settings_update_time) > 1000) {
    settings_update_time = millis ();

    if (dirty_ov_setting) {
      voltage_limit_set (voltage_limit);
       saveVoltageSetting(voltage_limit);
       dirty_ov_setting = 0;
       Serial.printf ("%s(%d) Voltage Limit Up date: voltage: %03.3fV, current: %03.3fA\n", __func__, __LINE__,
                   voltage_limit, current_limit);
    }
    if (dirty_oc_setting) {
       saveCurrentSetting(current_limit);
       current_limit_set (current_limit);
       dirty_oc_setting = 0;
       Serial.printf ("%s(%d) Current Limit Up date: voltage: %03.3fV, current: %03.3fA\n", __func__, __LINE__,
                   voltage_limit, current_limit);
    }
  }

  if ((millis () - display_update_time) > 100) {
    display_update_time = millis ();
    display_update ();
  }

  // update fault pot setting
  if ((millis () - pot_update_time) > 200) {
    pot_update_time = millis ();

    // Check voltage setting change
    float voltage_diff=0.0;
    new_OV_Position = OV_Enc.read();
    if (new_OV_Position != old_OV_Position) {
      Position_Diff = new_OV_Position - old_OV_Position;
      voltage_diff = ((float)Position_Diff) * -0.05;
      voltage_limit = voltage_limit - voltage_diff;
      Serial.printf ("%s(%d) OV %d %d %d\n", __func__, __LINE__,new_OV_Position,old_OV_Position ,Position_Diff);
      old_OV_Position = new_OV_Position;
//      saveVoltageSetting(voltage_limit);
//      voltage_limit_set (voltage_limit);  // Not sure this is required
      dirty_ov_setting = 1;
    }

    // Check Current setting change
    float current_diff=0.0;
    new_OC_Position = OC_Enc.read();
    if (new_OC_Position != old_OC_Position) {
      Position_Diff = new_OC_Position - old_OC_Position;
      current_diff = ((float)Position_Diff) * -0.05;
      current_limit = current_limit - current_diff;
      Serial.printf ("%s(%d) OC %d %d %d\n", __func__, __LINE__,new_OC_Position,old_OC_Position ,Position_Diff);
      old_OC_Position = new_OC_Position;
//      saveCurrentSetting(current_limit);
//      current_limit_set (current_limit); // Not sure this is required
      dirty_oc_setting = 1;
    }
  }

  button.poll ();             // Check output button state
  op_sw_state = button.toggle ();

  if (((millis () - fault_update_time) > 10) || (ina228_alr_int_flag)) {
    fault_update_time = millis ();
    alert_status = ina228_reg_read(0x0B);
    // check OC & OV faults
    if (ina228_alr_int_flag) {
       ina228_alr_int_flag = 0;
//      Serial.printf ("%s(%d) ina228_alr_int_flag set 0x%04X\n", __func__, __LINE__,alert_status);
    }

    if (alert_status & (OV_FAULT_BIT | OC_FAULT_BIT)) {
      read_voltage_current (&fault_voltage, &fault_current);  // capture current readings
      fault_state = 1;
      ov_led_set (alert_status & OV_FAULT_BIT);
      oc_led_set (alert_status & OC_FAULT_BIT);
      fet_drv_state (0);
      button.setToggleState (0);        // reset button toggle state
      alert_bell (1);
      if (alert_status & OV_FAULT_BIT) {
#if 0
         Serial.printf("%s(%d) Voltage Fault: %03.3fV, limit: %03.3fV, status reg: 0x%04X\n", 
                                __func__, __LINE__, fault_voltage, voltage_limit, alert_status);
#endif   
      } 
      if (alert_status & OC_FAULT_BIT) {
         Serial.printf("%s(%d) Current Fault: %03.3fA, limit: %03.3fA, status reg: 0x%04X\n", 
                                __func__, __LINE__, fault_current, current_limit, alert_status);
     } 
    }
    else if (fault_state == 0) {
      ov_led_set (0);
      oc_led_set (0);
    }

    if ((digitalRead (OV_ENC_SW) == 0) || (digitalRead (OC_ENC_SW) == 0)) { //AL_RST_SW) == 0) {
      fault_state = 0;
      alert_bell (0);
      fet_drv_state (0);
      button.setToggleState (0);        // reset button toggle state
    }

    if (op_sw_state) {
      fet_drv_state (1);
    }
    else {
      fet_drv_state (0);
    }
  }

  yield ();
}
