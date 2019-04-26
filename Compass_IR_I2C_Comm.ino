
#define SDA_PORT PORTC
#define SDA_PIN 0 // = A0
#define SCL_PORT PORTC
#define SCL_PIN 1 // = A1

#include <SoftI2CMaster.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_TSL2591.h"

const int compass_i2c_addr = 0x28;
const int OPR_register_addr = 0x3D;
const int compass_operation_mode = 0x08;    //Compass mode(Gyro+Mag)
const int compass_dataout_register = 0x1A;  //Euler Angle

Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591);

void setup(void) {
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);
  digitalWrite(A4, LOW); //disable internal pullup resistor
  digitalWrite(A5, LOW); //disable internal pullup resistor
  Serial.begin(57600);
  i2c_initiation();
  i2c_set_mode();
  IR_init();
}

double heading = 0.0;

struct IRPACKAGE{
  uint16_t ir;
  double ratio;
};

void loop(void) {
  static IRPACKAGE irpackage;
  heading = i2c_read_compass();
  irpackage = IR_read();
}

void i2c_initiation() {
  if (!i2c_init()) // Initialize everything and check for bus lockup
    Serial.println("I2C init failed");
  return;
}

void i2c_set_mode() {
  if (!i2c_start((compass_i2c_addr << 1) | I2C_WRITE)) { // start transfer
    Serial.println("I2C device busy");
    return;
  }
  i2c_write(OPR_register_addr);
  i2c_write(compass_operation_mode);
  i2c_stop();
  return;
}

double i2c_read_compass() {
  static bool firstread = true;
  static double head_ref = 0.0;
  static double heading_rawdata = 0.0;
  static double N = 0.0;
  static double heading = 0.0;

  if (!i2c_start((compass_i2c_addr << 1) | I2C_WRITE)) { // start transfer
    Serial.println("I2C device busy");
    return;
  }
  i2c_write(compass_dataout_register);
  i2c_rep_start((compass_i2c_addr << 1) | I2C_READ);
  heading_rawdata = i2c_read(false);
  N = i2c_read(true);
  i2c_stop();

  if (firstread) {
    head_ref = heading_rawdata + N * 255.0;
    firstread = false;
  }
  heading = (heading_rawdata + N * 255.0 - head_ref) * 0.0625;
  if (heading >= 360.0) {
    int n = heading / 360.001;
    heading -= (double)n * 360.0;
  }
  else if (heading < 0.0) {
    int n = -heading / 360.001;
    heading += 360.0 * (double)(n + 1);
  }
  Serial.print(heading);
  Serial.print("  ");
  return heading;
}

void IR_init() {
  Serial.println("Starting TSL2591 Navigation module!");
  if (tsl.begin()) {
    Serial.println("Found TSL2591 sensor");
  }
  else {
    Serial.println("Not enough sensor found ... check your wiring?");
    while (1);
  }
  configureSensor();
  return;
}

void configureSensor(void)
{
  // You can change the gain on the fly, to adapt to brighter/dimmer light situations
  tsl.setGain(TSL2591_GAIN_LOW);    // 1x gain (bright light)
  // Changing the integration time gives you a longer time over which to sense light
  // longer timelines are slower, but are good in very low light situtations!
  tsl.setTiming(TSL2591_INTEGRATIONTIME_100MS);  // shortest integration time (bright light)

  /* Display the gain and integration time for reference sake */
  Serial.println("------------------------------------");
  Serial.print  ("Gain:         ");
  tsl2591Gain_t gain = tsl.getGain();
  switch (gain)
  {
    case TSL2591_GAIN_LOW:
      Serial.println("One Tsl 1x (Low)");
      break;
    case TSL2591_GAIN_MED:
      Serial.println("One Tsl 25x (Medium)");
      break;
    case TSL2591_GAIN_HIGH:
      Serial.println("One tsl 428x (High)");
      break;
    case TSL2591_GAIN_MAX:
      Serial.println("One tsl 9876x (Max)");
      break;
  }
  Serial.print  ("Timing:       ");
  Serial.print((tsl.getTiming() + 1) * 100, DEC);
  Serial.println(" ms");
  Serial.println("------------------------------------");
  Serial.println("");
  return;
}

struct IRPACKAGE IR_read(void)
{
  // More advanced data read example. Read 32 bits with top 16 bits IR, bottom 16 bits full spectrum
  // That way you can do whatever math and comparisons you want!
  IRPACKAGE irpackage;
  uint32_t lum = tsl.getFullLuminosity();
  uint16_t full, visible;

  irpackage.ir = lum >> 16;
  full = lum & 0xFFFF;
  visible = full - irpackage.ir;
  irpackage.ratio = (double)irpackage.ir / visible;
  Serial.print("[ "); Serial.print(millis()); Serial.print(" ms ] ");
  Serial.print("IR: "); Serial.print(irpackage.ir); Serial.print("  ");
  Serial.print("Full: "); Serial.print(full); Serial.print("  ");
  Serial.print("Visible: "); Serial.print(full - irpackage.ir); Serial.print("  ");
  Serial.print("Lux: "); Serial.print(tsl.calculateLux(full, irpackage.ir)); Serial.print("  ");
  Serial.print("IRFULLratio: "); Serial.print(irpackage.ratio); Serial.println("  ");
  return irpackage;
}

