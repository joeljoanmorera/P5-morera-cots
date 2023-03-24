/***************************************************************************************************/
/* 
   This is an Arduino library for Aosong ASAIR AHT10, AHT15 Digital Humidity & Temperature Sensor

   written by : enjoyneering79
   sourse code: https://github.com/enjoyneering/

   This chip uses I2C bus to communicate, specials pins are required to interface
   Board:                                    SDA                    SCL                    Level
   Uno, Mini, Pro, ATmega168, ATmega328..... A4                     A5                     5v
   Mega2560................................. 20                     21                     5v
   Due, SAM3X8E............................. 20                     21                     3.3v
   Leonardo, Micro, ATmega32U4.............. 2                      3                      5v
   Digistump, Trinket, ATtiny85............. 0/physical pin no.5    2/physical pin no.7    5v
   Blue Pill, STM32F103xxxx boards.......... PB7                    PB6                    3.3v/5v
   ESP8266 ESP-01........................... GPIO0/D5               GPIO2/D3               3.3v/5v
   NodeMCU 1.0, WeMos D1 Mini............... GPIO4/D2               GPIO5/D1               3.3v/5v
   ESP32.................................... GPIO21/D21             GPIO22/D22             3.3v

   Frameworks & Libraries:
   ATtiny  Core          - https://github.com/SpenceKonde/ATTinyCore
   ESP32   Core          - https://github.com/espressif/arduino-esp32
   ESP8266 Core          - https://github.com/esp8266/Arduino
   STM32   Core          - https://github.com/stm32duino/Arduino_Core_STM32
                         - https://github.com/rogerclarkmelbourne/Arduino_STM32


   GNU GPL license, all text above must be included in any redistribution,
   see link for details  - https://www.gnu.org/licenses/licenses.html
*/
/***************************************************************************************************/
#include <AHT10.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define COLUMS           20   //LCD columns
#define ROWS             4    //LCD rows
#define LCD_SPACE_SYMBOL 0x20 //space symbol from LCD ROM, see p.9 of GDM2004D datasheet

LiquidCrystal_I2C lcd(PCF8574_ADDR_A21_A11_A01, 4, 5, 6, 16, 11, 12, 13, 14, POSITIVE);

uint8_t readStatus = 0;

AHT10 myAHT10(AHT10_ADDRESS_0X38);


void setup()
{
  Serial.begin(115200);
  Serial.println();
  
  while (myAHT10.begin() != true)
  {
    Serial.println(F("AHT10 not connected or fail to load calibration coefficient")); //(F()) save string to flash & keeps dynamic memory free
    delay(5000);
  }
  Serial.println(F("AHT10 OK"));

  Serial.begin(115200);

  while (lcd.begin(COLUMS, ROWS, LCD_5x8DOTS) != 1) //colums, rows, characters size
  {
    Serial.println(F("PCF8574 is not connected or lcd pins declaration is wrong. Only pins numbers: 4,5,6,16,11,12,13,14 are legal."));
    delay(5000);   
  }

  lcd.print(F("PCF8574 is OK..."));    //(F()) saves string to flash & keeps dynamic memory free
  delay(2000);

  lcd.clear();

  /* prints static text */
  lcd.setCursor(0, 0);                 //set 1-st colum & 2-nd row
  lcd.print(F("Processadors Digitals"));

  lcd.setCursor(0, 1);
  lcd.print(F("23/3/2023"));

  lcd.setCursor(0, 2);
  lcd.print(F("Temperatura: "));
  
  lcd.setCursor(0, 3);
  lcd.print(F("Humitat:  "));


}

//Wire.setClock(400000); //experimental I2C speed! 400KHz, default 100KHz



void loop()
{
  /* DEMO - 1, every temperature or humidity call will read 6 bytes over I2C, total 12 bytes */
  Serial.println(F("DEMO 1: read 12-bytes, show 255 if communication error is occurred"));
  Serial.print(F("Temperature: ")); Serial.print(myAHT10.readTemperature()); Serial.println(F(" +-0.3C")); //by default "AHT10_FORCE_READ_DATA"
  Serial.print(F("Humidity...: ")); Serial.print(myAHT10.readHumidity());    Serial.println(F(" +-2%"));   //by default "AHT10_FORCE_READ_DATA"
  lcd.setCursor(14, 2);
  lcd.print(myAHT10.readTemperature()); lcd.print(F("C"));
  lcd.setCursor(14,3);
  lcd.print(myAHT10.readHumidity()); lcd.print(F("%"));
  lcd.write(LCD_SPACE_SYMBOL);
  
  
  /* DEMO - 2, temperature call will read 6 bytes via I2C, humidity will use same 6 bytes */
  Serial.println(F("DEMO 2: read 6 byte, show 255 if communication error is occurred"));
  Serial.print(F("Temperature: ")); Serial.print(myAHT10.readTemperature(AHT10_FORCE_READ_DATA)); Serial.println(F(" +-0.3C"));
  Serial.print(F("Humidity...: ")); Serial.print(myAHT10.readHumidity(AHT10_USE_READ_DATA));      Serial.println(F(" +-2%"));


  /* DEMO - 3, same as demo2 but different call procedure */
  Serial.println(F("DEMO 3: read 6-bytes, show 255 if communication error is occurred"));

  readStatus = myAHT10.readRawData(); //read 6 bytes from AHT10 over I2C
  
  if (readStatus != AHT10_ERROR)
  {
    Serial.print(F("Temperature: ")); Serial.print(myAHT10.readTemperature(AHT10_USE_READ_DATA)); Serial.println(F(" +-0.3C"));
    Serial.print(F("Humidity...: ")); Serial.print(myAHT10.readHumidity(AHT10_USE_READ_DATA));    Serial.println(F(" +-2%"));
  }
  else
  {
    Serial.print(F("Failed to read - reset: ")); 
    Serial.println(myAHT10.softReset());         //reset 1-success, 0-failed
  }
lcd.write(LCD_SPACE_SYMBOL);
 
  delay(10000); //recomended polling frequency 8sec..30sec
}
