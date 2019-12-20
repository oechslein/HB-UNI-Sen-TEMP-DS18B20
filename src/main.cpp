//- -----------------------------------------------------------------------------------------------------------------------
// AskSin++
// 2016-10-31 papa Creative Commons - http://creativecommons.org/licenses/by-nc-sa/3.0/de/
// 2018-03-24 jp112sdl Creative Commons - http://creativecommons.org/licenses/by-nc-sa/3.0/de/
//- -----------------------------------------------------------------------------------------------------------------------

// define this to read the device id, serial and device type from bootloader section
// #define USE_OTA_BOOTLOADER

#define USE_LCD
#define LCD_ADDRESS 0x27

#define EI_NOTEXTERNAL
#include <EnableInterrupt.h>
#include <AskSinPP.h>
#include <LowPower.h>

#include <Register.h>
#include <MultiChannelDevice.h>

#ifdef USE_LCD
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(LCD_ADDRESS, 20, 4);
#endif

#include <OneWire.h>
#include <sensors/Ds18b20.h>
#define MAX_SENSORS       8

// Arduino Pro mini 8 Mhz
// Arduino pin for the config button
#define CONFIG_BUTTON_PIN 8
#define LED_PIN           4

#define LCD_BACKLIGHT_PIN 7 // USed to enable / disable LCD backlight
#define LCD_ON_PIN 6        // Enables LCD per interrupt
#define LCD_ON_VCC_PIN 5    // Always LOW used for LCD_ON_PIN

// number of available peers per channel
#define PEERS_PER_CHANNEL 6

//DS18B20 Sensors connected to pin
OneWire oneWire(3);

// all library classes are placed in the namespace 'as'
using namespace as;

// define all device properties
const struct DeviceInfo PROGMEM devinfo = {
  {0xf3, 0x01, 0x01},          // Device ID
  "UNITEMP001",               // Device Serial
  {0xF3, 0x01},              // Device Model
  0x10,                       // Firmware Version
  as::DeviceType::THSensor,   // Device Type
  {0x01, 0x01}               // Info Bytes
};

/**
   Configure the used hardware
*/
typedef AvrSPI<10, 11, 12, 13> SPIType;
typedef Radio<SPIType, 2> RadioType;
typedef StatusLed<LED_PIN> LedType;
typedef AskSin<LedType, BatterySensor, RadioType> BaseHal;
class Hal : public BaseHal {
  public:
    void init (const HMID& id) {
      BaseHal::init(id);

      battery.init(seconds2ticks(60UL * 60), sysclock); //battery measure once an hour
      battery.low(22);
      battery.critical(18);
    }

    bool runready () {
      return sysclock.runready() || BaseHal::runready();
    }
} hal;


DEFREGISTER(UReg0, MASTERID_REGS, DREG_LOWBATLIMIT, 0x21, 0x22)
class UList0 : public RegList0<UReg0> {
  public:
    UList0 (uint16_t addr) : RegList0<UReg0>(addr) {}
    bool Sendeintervall (uint16_t value) const {
      return this->writeRegister(0x21, (value >> 8) & 0xff) && this->writeRegister(0x22, value & 0xff);
    }
    uint16_t Sendeintervall () const {
      return (this->readRegister(0x21, 0) << 8) + this->readRegister(0x22, 0);
    }
    void defaults () {
      clear();
      lowBatLimit(22);
      Sendeintervall(180);
    }
};

DEFREGISTER(UReg1, 0x01, 0x02, 0x03, 0x04)
class UList1 : public RegList1<UReg1> {
  public:
    UList1 (uint16_t addr) : RegList1<UReg1>(addr) {}

    bool Offset (int32_t value) const {
      return
          this->writeRegister(0x01, (value >> 24) & 0xff) &&
          this->writeRegister(0x02, (value >> 16) & 0xff) &&
          this->writeRegister(0x03, (value >> 8) & 0xff) &&
          this->writeRegister(0x04, (value) & 0xff)
          ;
    }

    int32_t Offset () const {
      return
          ((int32_t)(this->readRegister(0x01, 0)) << 24) +
          ((int32_t)(this->readRegister(0x02, 0)) << 16) +
          ((int32_t)(this->readRegister(0x03, 0)) << 8) +
          ((int32_t)(this->readRegister(0x04, 0)))
          ;
    }
    void defaults () {
      clear();
      Offset(0);
    }
};

int32_t Offsets[MAX_SENSORS];
int16_t temperatures[MAX_SENSORS];
static const int16_t INVALID_TEMPERATURE = -555;


class WeatherEventMsg : public Message {
  public:
    void init(uint8_t msgcnt, Ds18b20* sensors, bool batlow, uint8_t channelFieldOffset) {
      Message::init(0x16, msgcnt, 0x53, (msgcnt % 20 == 1) ? (BIDI | WKMEUP) : BCAST, batlow ? 0x80 : 0x00, 0x41 + channelFieldOffset);
      int16_t t0 = sensors[0 + channelFieldOffset].temperature() + Offsets[0 + channelFieldOffset];
      int16_t t1 = sensors[1 + channelFieldOffset].temperature() + Offsets[1 + channelFieldOffset];
      int16_t t2 = sensors[2 + channelFieldOffset].temperature() + Offsets[2 + channelFieldOffset];
      int16_t t3 = sensors[3 + channelFieldOffset].temperature() + Offsets[3 + channelFieldOffset];

      pload[0] = (t0 >> 8) & 0xff;
      pload[1] = (t0) & 0xff;
      pload[2] = 0x42 + channelFieldOffset;
      pload[3] = (t1 >> 8) & 0xff;
      pload[4] = (t1) & 0xff;
      pload[5] = 0x43 + channelFieldOffset;
      pload[6] = (t2 >> 8) & 0xff;
      pload[7] = (t2) & 0xff;
      pload[8] = 0x44 + channelFieldOffset;
      pload[9] = (t3 >> 8) & 0xff;
      pload[10] = (t3) & 0xff;
    }
};

class WeatherChannel : public Channel<Hal, UList1, EmptyList, List4, PEERS_PER_CHANNEL, UList0> {
  public:
    WeatherChannel () : Channel() {}
    virtual ~WeatherChannel () {}

    void configChanged() {
      //DPRINT(F("(")); DDEC(number()); DPRINTLN(F(") Config changed List1"));
      DPRINT(F("OFFSET: ")); DDECLN(this->getList1().Offset());
      Offsets[number() - 1] = this->getList1().Offset();
    }

    uint8_t status () const {
      return 0;
    }

    uint8_t flags () const {
      return 0;
    }
};

class UType : public MultiChannelDevice<Hal, WeatherChannel, MAX_SENSORS, UList0> {

    class SensorArray : public Alarm {
        UType& dev;

      public:
        uint8_t       sensorcount;
        Ds18b20       sensors[MAX_SENSORS];
        SensorArray (UType& d) : Alarm(0), dev(d), sensorcount(0) {}

        virtual void trigger (__attribute__ ((unused)) AlarmClock& clock) {
          tick = delay();
          sysclock.add(*this);

          Ds18b20::measure(sensors, sensorcount);
          DPRINT(F("Temperaturen: | "));
          for (int i = 0; i < MAX_SENSORS; i++) {
            DDEC(sensors[i].temperature()); DPRINT(" | ");

            if ((i + 1) <= sensorcount) {
              temperatures[i] = sensors[i].temperature();
            } else {
              temperatures[i] = INVALID_TEMPERATURE;
            }
          }
          DPRINTLN("");
          WeatherEventMsg& msg = (WeatherEventMsg&)dev.message();
          //Aufteilung in 2 Messages, da sonst die max. BidCos Message Size (0x1a)? überschritten wird
          msg.init(dev.nextcount(), sensors, dev.battery().low(), 0);
          dev.send(msg, dev.getMasterID());
#if MAX_SENSORS > 4
          _delay_ms(250);
          msg.init(dev.nextcount(), sensors, dev.battery().low(), 4);
          dev.send(msg, dev.getMasterID());
#endif
        }

        uint32_t delay () {
          uint16_t _txMindelay = 180;
          _txMindelay = dev.getList0().Sendeintervall();
          if (_txMindelay == 0) _txMindelay = 180;
          return seconds2ticks(_txMindelay);
        }

    } sensarray;

  public:
    typedef MultiChannelDevice<Hal, WeatherChannel, MAX_SENSORS, UList0> TSDevice;
    UType(const DeviceInfo& info, uint16_t addr) : TSDevice(info, addr), sensarray(*this) {}
    virtual ~UType () {}

    virtual void configChanged () {
      TSDevice::configChanged();
      DPRINTLN("Config Changed List0");
      DPRINT("LOW BAT Limit: ");
      DDECLN(this->getList0().lowBatLimit());
      this->battery().low(this->getList0().lowBatLimit());
      DPRINT("Sendeintervall: "); DDECLN(this->getList0().Sendeintervall());
    }

    void init (Hal& hal) {
      TSDevice::init(hal);
      sensarray.sensorcount = Ds18b20::init(oneWire, sensarray.sensors, MAX_SENSORS);
      DPRINT("Found "); DDEC(sensarray.sensorcount); DPRINTLN(" DS18B20 Sensors");
#ifdef USE_LCD
      //lcd.setCursor(2, 3);
      //lcd.print("Found Sensors: " + String(sensarray.sensorcount));
#endif
      sensarray.set(seconds2ticks(5));
      sysclock.add(sensarray);
    }
};

UType sdev(devinfo, 0x20);
ConfigButton<UType> cfgBtn(sdev);

void i2c_scanner() {
  Wire.begin();

  byte error, address;
  int nDevices;
 
  Serial.println("Scanning...");
 
  nDevices = 0;
  for(address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
 
    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");
 
      nDevices++;
    }
    else if (error==4)
    {
      Serial.print("Unknown error at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
}

volatile static bool backlight_on = false;
volatile static unsigned long backlight_on_ticks = 0;
void lcd_on_button_pressed() {
  Serial.println("lcd_on_button_pressed pressed\n");
  backlight_on = true;
  backlight_on_ticks = millis();
}

void setup () {
  DINIT(57600, ASKSIN_PLUS_PLUS_IDENTIFIER);
  memset(Offsets, 0, MAX_SENSORS);
  DDEVINFO(sdev);

  for (int i = 0; i < MAX_SENSORS; i++) {
    temperatures[i] = INVALID_TEMPERATURE;
  }

#ifdef USE_LCD
  pinMode(LCD_BACKLIGHT_PIN, OUTPUT);
  digitalWrite(LCD_BACKLIGHT_PIN, HIGH);
  pinMode(LCD_ON_VCC_PIN, OUTPUT);
  digitalWrite(LCD_ON_VCC_PIN, LOW);

  //i2c_scanner();
  lcd.init();
  lcd.backlight();
  lcd.setCursor(3, 0);
  lcd.print((char*)serial);
  HMID temp;
  sdev.getDeviceID(temp);
  lcd.setCursor(5, 1);
  lcd.print(temp, HEX);

  lcd_on_button_pressed();

  pinMode(LCD_ON_PIN, INPUT_PULLUP);
  enableInterrupt(LCD_ON_PIN, lcd_on_button_pressed, RISING);
#endif

  sdev.init(hal);
  buttonISR(cfgBtn, CONFIG_BUTTON_PIN);
  sdev.initDone();
}

#ifdef USE_LCD
void loop_lcd() {
  static const int unsigned PAGE_STAY_SECS = 4;
  static const int unsigned BACKLIGHT_STAY_ON_SECS = PAGE_STAY_SECS * 2 * 5;
  static unsigned long last_lcd_update = millis();  // while sleeping timer also stops
  static bool first_page = false;

  if (backlight_on
      && (// overflow sets backlight to false
          (millis() < backlight_on_ticks) 
           // or of BACKLIGHT_STAY_ON_SECS reached
           || (millis() > (backlight_on_ticks + BACKLIGHT_STAY_ON_SECS * 1000UL)))) {
    Serial.println("Switch off backlight");
    Serial.println(millis() < backlight_on_ticks);
    Serial.println(millis() > (backlight_on_ticks + BACKLIGHT_STAY_ON_SECS * 1000UL));
    backlight_on = false;
  }

  if (!backlight_on) {
    digitalWrite(LCD_BACKLIGHT_PIN, LOW);
    lcd.clear();
    first_page = false;
  } else {
    hal.activity.stayAwake(seconds2ticks(BACKLIGHT_STAY_ON_SECS));

    /*
    Serial.print("millis: "); Serial.println(millis());
    Serial.print("backlight_on_ticks: "); Serial.println(backlight_on_ticks);
    Serial.print("backlight_on_ticks + seconds2ticks(BACKLIGHT_STAY_ON_SECS): "); Serial.println(backlight_on_ticks + BACKLIGHT_STAY_ON_SECS * 1000);
    Serial.print("last_lcd_update: "); Serial.println(last_lcd_update);
    Serial.print("last_lcd_update + seconds2ticks(1): "); Serial.println(last_lcd_update + PAGE_STAY_SECS * 1000);
    */

    digitalWrite(LCD_BACKLIGHT_PIN, HIGH);
    bool all_temp_unset = true;
    for (int i = 0; i < MAX_SENSORS; i++) {
      if (temperatures[i] != INVALID_TEMPERATURE) all_temp_unset = false;
    }

    if (// overflow changes first_page
        (millis() < last_lcd_update) 
        // or if one second reached
        || (millis() > (last_lcd_update + PAGE_STAY_SECS * 1000))) {
        first_page = !first_page;
        last_lcd_update = millis();
    }

    if (!all_temp_unset) {
      int start_i = first_page ? 0 : 4;
      for (int i = start_i; i < min(start_i + 4, MAX_SENSORS); i++) {
        uint8_t x = (i % 2 == 0 ? 0 : 8);
        uint8_t y = (i - start_i) / 2;
        lcd.setCursor(x, y);

        String s_temp = " --.-";
        if (temperatures[i] != INVALID_TEMPERATURE) {
          s_temp = (String)round((float)(temperatures[i] + Offsets[i]) / 10.0);
          //s_temp = s_temp.substring(0, s_temp.length() - 1);
          if (temperatures[i] < 1000 && temperatures[i] >= 0) s_temp = " " + s_temp;
        }
        String disp_temp = String(i + 1) + ":" + s_temp + (char)223 + "C " + "  ";
        lcd.print(disp_temp);
      }
    }
  }
}
#endif  // ifdef USE_LCD

void loop() {

  bool worked = hal.runready();
  bool poll = sdev.pollRadio();

#ifdef USE_LCD
  loop_lcd();
#endif  

  if ( worked == false && poll == false ) {
    if ( hal.battery.critical() ) {
      hal.activity.sleepForever(hal);
    }
    hal.activity.savePower<Sleep<>>(hal);
  }
}

