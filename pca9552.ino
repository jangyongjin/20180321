/******************************************************************************
 *  i2c
 *  
 *  jangyongjin@gmail.com
 *  
 *  April 5, 2018
 *
 *  Prototype I2C interface to TI PCA9552
 *
 ******************************************************************************/

#include <Wire.h>

#define ADD       (0xC << 3 | 0x7)

#define FLAG_AI   0x10

#define PSC0      FLAG_AI | 0x02
#define PWM0      FLAG_AI | 0x03
#define PSC1      FLAG_AI | 0x04
#define PWM1      FLAG_AI | 0x05
#define LS0       FLAG_AI | 0x06
#define LS1       FLAG_AI | 0x07
#define LS2       FLAG_AI | 0x08
#define LS3       FLAG_AI | 0x09

#define LED_ON    0
#define LED_OFF   1
#define LED_PWM0  2
#define LED_PWM1  3

void setup() {  
  pinMode(7, OUTPUT);
  delay(100);

  digitalWrite(7, LOW);
  delay(500);

  digitalWrite(7, HIGH);
  
  Wire.begin();
  
  write_reg(ADD, PSC0, 43);
  write_reg(ADD, PWM0, 128);
  write_reg(ADD, PSC1, 10);
  write_reg(ADD, PWM1, 192);
  write_reg(ADD, LS0, B10101010);
  write_reg(ADD, LS1, B10101010);
  write_reg(ADD, LS2, B10101010);
  write_reg(ADD, LS3, B10101010);
}

void loop() {
  
}

void write_reg(int address, int cmd, int data) {
  //  Send output register address
  Wire.beginTransmission(address);
  Wire.write(cmd);
  
  //  Connect to device and send two bytes
  Wire.write(0xff & data);  //  low byte
  Wire.write(data >> 8);    //  high byte

  Wire.endTransmission();
}

