//- -----------------------------------------------------------------------------------------------------------------------
// AskSin++
// 2016-10-31 papa Creative Commons - http://creativecommons.org/licenses/by-nc-sa/3.0/de/
// 2021-03-03 jp112sdl Creative Commons - http://creativecommons.org/licenses/by-nc-sa/3.0/de/
// HB-UNI-Sen-TDS - HomeMatic TDS Sensor
// based on HB-UNI-Sen-EC by jp112sdl
// DFRobot Gravity: Analog TDS Sensor / Meter (SEN0244)
//- -----------------------------------------------------------------------------------------------------------------------
// ci-test=yes board=328p aes=no

// ========================================================
// Hinweise zur GravityTDS-Library:
//
// 1) KALIBRIERUNG:
//    Die Library besitzt KEINE oeffentliche calibrate()-Methode.
//    Die interne ecCalibration() wird ausschliesslich ueber Serial-
//    Kommandos ("ENTER", "CAL:707", "EXIT") ausgeloest.
//    Im AskSinPP-Betrieb ohne freie Serial-Konsole kalibrieren wir
//    daher den kValue selbst und schreiben ihn direkt per EEPROM.
//
//    Formel (aus GravityTDS.cpp):
//      rawEC  = 133.42*U^3 - 255.86*U^2 + 857.39*U   [uS/cm, bei kValue=1]
//      rawEC25 = rawEC / (1 + 0.02*(T-25))            [Temperaturkompensation]
//      kValue = rawEC_solution / rawEC                [bei 707 ppm = 1413 uS/cm]
//    Gueltiger Bereich: kValue 0.25 .. 4.0
//
// 2) TEMPERATURSENSOR (DS18B20):
//    OPTIONAL - ohne Sensor verwendet die Library 25 °C als Default.
//    Fehler ohne Kompensation: ~2 % pro °C Abweichung von 25 °C.
//    Fuer Poolwasser (15-35 °C) empfohlen, aber nicht zwingend.
//    Wird kein DS18B20 gefunden, wird 25.0 °C als Festwert genutzt
//    und TEMPERATURE=-40.0 (Sonderwert) an die CCU uebertragen.
// ========================================================

#define SENSOR_ONLY
#define HIDE_IGNORE_MSG

#define EI_NOTEXTERNAL
#include <EnableInterrupt.h>
#include <AskSinPP.h>
#include <LowPower.h>
#include <Register.h>
#include <MultiChannelDevice.h>
#include <sensors/Ds18b20.h>
#include <EEPROM.h>
#include <GravityTDS.h>
#include <LiquidCrystal_I2C.h>

// ========================================================
// Pin-Belegung
// ========================================================
#define CONFIG_BUTTON_PIN    8
#define LED_PIN              4
#define DS18B20_PIN          5       // optional, 0 = nicht verwendet
#define TDS_SENSOR_PIN       A1

// CC1101: SCK=13, MISO=12, MOSI=11, CS=10, GDO0=2

// ========================================================
// Konfiguration
// ========================================================
#define LCD_ADDRESS          0x27
#define LCD_ROWS             2
#define LCD_COLUMNS          16

#define REF_VOLTAGE          5.0f    // AREF in Volt (3.3f fuer 3.3V-Boards!)
#define ADC_RANGE            1024.0f // 10-Bit ADC

// EEPROM-Adresse fuer kValue (gleich wie in GravityTDS-Library: 0x08)
#define KVALUE_EEPROM_ADDR   8

// Kalibrierstandard: 707 ppm = 1413 uS/cm @ 25 °C
#define CAL_STANDARD_PPM     707.0f
#define TDS_FACTOR           0.5f    // TDS = EC / 2  (aus GravityTDS.h)

// Plausibilitaet Kalibrierung: kValue muss in [0.25 .. 4.0] liegen
#define KVALUE_MIN           0.25f
#define KVALUE_MAX           4.0f

// Plausibilitaet Rohspannung: <5 mV = Sonde nicht eingetaucht
#define VOLTAGE_MIN_MV       5.0f

// Anzahl ADC-Samples fuer stabile Messung
#define ADC_SAMPLES          30

#define CALIBRATION_MODE_TIMEOUT  600   // Sekunden bis Kalibriermodus ablaeuft

#define PEERS_PER_CHANNEL    6

using namespace as;

// ========================================================
// DeviceInfo - Model 0xFC30
// ========================================================
const struct DeviceInfo PROGMEM devinfo = {
  { 0xFC, 0x30, 0x01 },   // Device ID
  "HBTDS00001",            // Device Serial (10-stellig, in CCU eindeutig!)
  { 0xFC, 0x30 },          // Device Model
  0x10,                    // Firmware Version
  0x53,                    // Device Type
  { 0x01, 0x00 }           // Info Bytes
};

typedef AskSin<StatusLed<LED_PIN>, NoBattery, Radio<AvrSPI<10, 11, 12, 13>, 2>> Hal;
Hal hal;

// ========================================================
// List0 - Geraete-Register
// ========================================================
DEFREGISTER(UReg0, MASTERID_REGS, 0x1f, 0x20, 0x21, DREG_BACKONTIME)
class UList0 : public RegList0<UReg0> {
public:
  UList0 (uint16_t addr) : RegList0<UReg0>(addr) {}

  bool Sendeintervall (uint8_t value) const { return this->writeRegister(0x21, value & 0xff); }
  uint8_t Sendeintervall () const { return this->readRegister(0x21, 0); }

  bool Messintervall (uint16_t value) const {
    return this->writeRegister(0x1f, (value >> 8) & 0xff) &&
           this->writeRegister(0x20, value & 0xff);
  }
  uint16_t Messintervall () const {
    return (this->readRegister(0x1f, 0) << 8) + this->readRegister(0x20, 0);
  }

  void defaults () {
    clear();
    lowBatLimit(22);
    backOnTime(60);
    Sendeintervall(18);
    Messintervall(10);
  }
};

// ========================================================
// List1 - Kanal-Register
// ========================================================
DEFREGISTER(UReg1, 0x05)
class UList1 : public RegList1<UReg1> {
public:
  UList1 (uint16_t addr) : RegList1<UReg1>(addr) {}

  bool TemperatureOffsetIndex (uint8_t value) const {
    return this->writeRegister(0x05, value & 0xff);
  }
  uint8_t TemperatureOffsetIndex () const { return this->readRegister(0x05, 0); }

  void defaults () { clear(); TemperatureOffsetIndex(7); }
};

// ========================================================
// EEPROM-Hilfsfunktionen fuer kValue (float, 4 Byte)
// ========================================================
void kValueWrite(float v) {
  byte *p = (byte*)&v;
  for (int i = 0; i < (int)sizeof(float); i++)
    EEPROM.write(KVALUE_EEPROM_ADDR + i, p[i]);
}

float kValueRead() {
  float v;
  byte *p = (byte*)&v;
  for (int i = 0; i < (int)sizeof(float); i++)
    p[i] = EEPROM.read(KVALUE_EEPROM_ADDR + i);
  // Uninitialisiertes EEPROM (0xFF 0xFF 0xFF 0xFF) = NaN / ungueltig
  if (EEPROM.read(KVALUE_EEPROM_ADDR)   == 0xFF &&
      EEPROM.read(KVALUE_EEPROM_ADDR+1) == 0xFF &&
      EEPROM.read(KVALUE_EEPROM_ADDR+2) == 0xFF &&
      EEPROM.read(KVALUE_EEPROM_ADDR+3) == 0xFF) {
    return 1.0f;
  }
  return v;
}

// ========================================================
// Rohe EC-Kurve der GravityTDS-Library (kValue=1)
// ecRaw = 133.42*U^3 - 255.86*U^2 + 857.39*U  [uS/cm]
// ========================================================
float calcRawEC(float voltageV) {
  return 133.42f * voltageV * voltageV * voltageV
       - 255.86f * voltageV * voltageV
       + 857.39f * voltageV;
}

// ========================================================
// Stabilen ADC-Mittelwert lesen (Median-Filter wie in Library)
// ========================================================
float readStableVoltage() {
  int buf[ADC_SAMPLES];
  for (int i = 0; i < ADC_SAMPLES; i++) {
    buf[i] = analogRead(TDS_SENSOR_PIN);
    delay(10);
  }
  // Bubblesort (kleine Array-Groesse)
  for (int i = 0; i < ADC_SAMPLES - 1; i++)
    for (int j = i + 1; j < ADC_SAMPLES; j++)
      if (buf[i] > buf[j]) { int t = buf[i]; buf[i] = buf[j]; buf[j] = t; }
  // Mittleres Drittel verwenden
  long sum = 0;
  for (int i = ADC_SAMPLES/3; i < ADC_SAMPLES*2/3; i++) sum += buf[i];
  float avg = (float)sum / (ADC_SAMPLES / 3);
  return avg / ADC_RANGE * REF_VOLTAGE;
}

// ========================================================
// LCD-Klasse
// ========================================================
class LcdType {
public:
  class BacklightAlarm : public Alarm {
    LcdType& lcdDev;
  public:
    BacklightAlarm (LcdType& l) : Alarm(0), lcdDev(l) {}
    virtual ~BacklightAlarm () {}
    void restartTimer(uint8_t sec) {
      sysclock.cancel(*this);
      set(seconds2ticks(sec));
      lcdDev.lcd.backlight();
      sysclock.add(*this);
    }
    virtual void trigger (__attribute__((unused)) AlarmClock& clock) {
      lcdDev.lcd.noBacklight();
    }
  } backlightalarm;

private:
  uint8_t backlightOnTime;
  byte degree[8] = { 0b00111, 0b00101, 0b00111, 0b00000,
                     0b00000, 0b00000, 0b00000, 0b00000 };

  String tempToStr(int16_t t) {
    String s = String((float)t / 10.0f, 1);
    return s;
  }

public:
  LiquidCrystal_I2C lcd;
  LcdType () : backlightalarm(*this), backlightOnTime(10),
               lcd(LCD_ADDRESS, LCD_COLUMNS, LCD_ROWS) {}
  virtual ~LcdType () {}

  void showMeasureValues(int16_t temperature, uint16_t tds10, float kVal) {
    lcd.clear();
    // Zeile 0: TDS-Wert
    uint16_t tdsInt = tds10 / 10;
    uint8_t  tdsDec = tds10 % 10;
    char buf[17];
    snprintf(buf, sizeof(buf), "TDS:%4u.%1u ppm  ", tdsInt, tdsDec);
    lcd.setCursor(0, 0);
    lcd.print(buf);
    // Zeile 1: Temperatur + kValue
    lcd.setCursor(0, 1);
    if (temperature == -400) {
      lcd.print("T: --.- C");
    } else {
      snprintf(buf, sizeof(buf), "T:%5.1f", (float)temperature / 10.0f);
      lcd.print(buf);
      lcd.write(byte(0));
      lcd.print("C");
    }
    // kValue rechts anzeigen (Hinweis ob kalibriert)
    snprintf(buf, sizeof(buf), " k%4.2f", kVal);
    lcd.setCursor(9, 1);
    lcd.print(buf);
  }

  void showCalibrationMenu(uint8_t step) {
    lcd.clear();
    switch (step) {
      case 0:
        lcd.setCursor(0, 0); lcd.print(F("TDS CALIBRATION"));
        lcd.setCursor(2, 1); lcd.print(F("Press button"));
        break;
      case 1:
        lcd.setCursor(0, 0); lcd.print(F("Put 707ppm sol."));
        lcd.setCursor(2, 1); lcd.print(F("Press button"));
        break;
      case 11:
        lcd.setCursor(0, 0); lcd.print(F("707ppm READ OK"));
        lcd.setCursor(2, 1); lcd.print(F("Saving kValue..."));
        _delay_ms(1000);
        break;
      case 99:
        lcd.setCursor(0, 0); lcd.print(F("Calibration"));
        lcd.setCursor(0, 1); lcd.print(F("Done. Saved."));
        _delay_ms(2000);
        break;
      case 200:
        lcd.setCursor(0, 0); lcd.print(F("ERR: no signal"));
        lcd.setCursor(0, 1); lcd.print(F("Check probe!"));
        _delay_ms(2000);
        break;
      case 201:
        lcd.setCursor(0, 0); lcd.print(F("ERR:value range"));
        lcd.setCursor(0, 1); lcd.print(F("Retry! Press btn"));
        _delay_ms(2000);
        break;
    }
  }

  void initLCD(uint8_t *serial) {
    Wire.begin();
    Wire.beginTransmission(LCD_ADDRESS);
    if (Wire.endTransmission() == 0) {
      lcd.init();
      lcd.createChar(0, degree);
      lcd.backlight();
      lcd.setCursor(0, 0);
      lcd.print(ASKSIN_PLUS_PLUS_IDENTIFIER);
      lcd.setCursor(3, 1);
      lcd.setContrast(200);
      lcd.print((char*)serial);
      if (backlightOnTime > 0) backlightalarm.restartTimer(backlightOnTime);
    } else {
      DPRINT("LCD not found at 0x"); DHEXLN((uint8_t)LCD_ADDRESS);
    }
  }

  void setBackLightOnTime(uint8_t t) {
    backlightOnTime = t;
    if (backlightOnTime == 0) lcd.backlight(); else lcd.noBacklight();
  }
};

LcdType lcd;

// ========================================================
// Message
// ========================================================
class MeasureEventMsg : public Message {
public:
  // Payload: 2 Byte Temperatur (signed, *10) + 2 Byte TDS (*10 ppm)
  // Gesamtlaenge: 0x0d = 13 Bytes
  void init(uint8_t msgcnt, int16_t temp, uint16_t tds10) {
    Message::init(0x0d, msgcnt, 0x53, BIDI | WKMEUP, (temp >> 8) & 0x7f, temp & 0xff);
    pload[0] = (tds10 >> 8) & 0xff;
    pload[1] = tds10 & 0xff;
  }
};

// ========================================================
// Messkanal
// ========================================================
class MeasureChannel
  : public Channel<Hal, UList1, EmptyList, List4, PEERS_PER_CHANNEL, UList0>,
    public Alarm
{
private:
  MeasureEventMsg  msg;
  OneWire          dsWire;
  Ds18b20          ds18b20[1];
  bool             ds18b20_present;
  GravityTDS       gravityTds;
  float            kValue;

  bool             tdsCalibrationMode;
  uint8_t          tdsCalibrationStep;
  int16_t          currentTemperature;   // *10, degC
  uint16_t         tds10;                // TDS * 10 in ppm

  uint16_t         measureCount;
  uint32_t         tds10_cumulated;
  int32_t          temperature_cumulated;

public:
  MeasureChannel ()
    : Channel(), Alarm(seconds2ticks(3)),
      dsWire(DS18B20_PIN), ds18b20_present(false),
      kValue(1.0f),
      tdsCalibrationMode(false), tdsCalibrationStep(0),
      currentTemperature(250), tds10(0),
      measureCount(0), tds10_cumulated(0), temperature_cumulated(0)
  {}
  virtual ~MeasureChannel () {}

  // ---- Temperatur lesen (DS18B20 + Offset, oder 25.0 °C Fallback) ----
  int16_t readTemperature() {
    if (!ds18b20_present) return 250;   // 25.0 °C
    Ds18b20::measure(ds18b20, 1);
    int16_t raw = ds18b20[0].temperature();
    // Offset: Index 7 = 0.0 K, je Schritt +0.5 K, Bereich -3.5..+3.5 K
    int16_t t = raw + (-35 + 5 * (int16_t)this->getList1().TemperatureOffsetIndex());
    DPRINT(F("Temperature : ")); DDECLN(t);
    return t;
  }

  // ---- TDS lesen: eigene Berechnung mit gespeichertem kValue --------
  // Wir verwenden die Library nur fuer ADC-Lesen/Mittelung,
  // berechnen aber mit dem manuell gesetzten kValue fuer korrekte
  // Ergebnisse auch ohne Serial-Kalibrierung.
  uint16_t readTDS() {
    // Temperatur setzen (fuer interne Kompensation in update())
    float tempC = (float)currentTemperature / 10.0f;
    gravityTds.setTemperature(tempC);
    gravityTds.update();
    float tdsVal = gravityTds.getTdsValue();   // ppm, mit gespeichertem kValue
    DPRINT(F("TDS (ppm)   : ")); DDECLN((int)tdsVal);
    return (uint16_t)(tdsVal * 10.0f + 0.5f);
  }

  // ---- Kalibrierung ----
  void disableCalibrationMode() {
    DPRINTLN(F("Exiting TDS Calibration Mode"));
    tdsCalibrationMode = false;
    sysclock.cancel(*this);
    tdsCalibrationStep = 0;
    this->changed(true);
    set(millis2ticks(1000));
    sysclock.add(*this);
  }

  void enableCalibrationMode() {
    DPRINTLN(F("Entering TDS Calibration Mode"));
    tdsCalibrationMode = true;
    sysclock.cancel(*this);
    this->changed(true);
    set(seconds2ticks(CALIBRATION_MODE_TIMEOUT));
    sysclock.add(*this);
    tdsCalibrationStep = 0;
    nextCalibrationStep();
  }

  void toggleTDSCalibrationMode() {
    tdsCalibrationMode = !tdsCalibrationMode;
    if (tdsCalibrationMode) enableCalibrationMode();
    else                    disableCalibrationMode();
  }

  bool getCalibrationMode() { return tdsCalibrationMode; }

  // ------------------------------------------------------------------
  // Kalibrierungs-Ablauf: Single-Point mit 707 ppm Standard-Loesung
  //
  //   kValue = rawEC_solution_25C / rawEC_measured_25C
  //
  //   rawEC_solution_25C = (CAL_STANDARD_PPM / TDS_FACTOR)       [uS/cm]
  //                      * (1 + 0.02 * (tempC - 25))             [Temp-Komp. rueckgaengig]
  //
  //   rawEC_measured_25C = calcRawEC(voltage)   [kValue=1, vor Temp-Komp.]
  //
  // Die Library kalibriert intern identisch (GravityTDS.cpp, case 2).
  // Wir schreiben kValue direkt per EEPROM, damit gravityTds.begin()
  // den Wert beim naechsten Start automatisch laedt.
  // ------------------------------------------------------------------
  void nextCalibrationStep() {
    if (!tdsCalibrationMode) { tdsCalibrationStep = 0; return; }

    DPRINT(F("TDS CALIB STEP ")); DDECLN(tdsCalibrationStep);

    switch (tdsCalibrationStep) {
      case 0:
        lcd.showCalibrationMenu(0);   // "TDS CALIBRATION"
        break;

      case 1:
        lcd.showCalibrationMenu(1);   // "Put 707ppm sol."
        break;

      case 2: {
        // Stabile Spannung messen
        float voltage = readStableVoltage();
        float tempC   = (float)readTemperature() / 10.0f;

        DPRINT(F("  CAL voltage (V) : ")); DDECLN((int)(voltage * 1000));
        DPRINT(F("  CAL tempC  (°C) : ")); DDECLN((int)tempC);

        // Kein Signal?
        if (voltage < (VOLTAGE_MIN_MV / 1000.0f)) {
          DPRINTLN(F("  CAL ERR: no signal"));
          lcd.showCalibrationMenu(200);
          tdsCalibrationStep = 1;
          return;
        }

        // Raw-EC bei kValue=1 berechnen
        float rawEC     = calcRawEC(voltage);
        // Temperaturkompensation rueckgaengig machen wie die Library
        float rawEC25   = rawEC / (1.0f + 0.02f * (tempC - 25.0f));

        // Ziel-EC aus der Standardloesung (707 ppm -> 1414 uS/cm)
        float targetEC25 = (CAL_STANDARD_PPM / TDS_FACTOR)
                         * (1.0f + 0.02f * (tempC - 25.0f));

        // kValue berechnen (gleiche Formel wie GravityTDS.cpp)
        float kNew = targetEC25 / rawEC25;

        DPRINT(F("  CAL rawEC25     : ")); DDECLN((int)rawEC25);
        DPRINT(F("  CAL targetEC25  : ")); DDECLN((int)targetEC25);
        DPRINT(F("  CAL kValue new  : ")); DDECLN((int)(kNew * 1000));

        // Plausibilitaet: kValue 0.25 .. 4.0 (wie Library)
        if (kNew < KVALUE_MIN || kNew > KVALUE_MAX) {
          DPRINT(F("  CAL ERR: kValue out of range (0.25..4.0): "));
          DDECLN((int)(kNew * 1000));
          lcd.showCalibrationMenu(201);
          tdsCalibrationStep = 1;
          return;
        }

        // kValue ins EEPROM schreiben (Adresse 8, identisch zur Library)
        kValueWrite(kNew);
        kValue = kNew;
        // Library-Instanz neu initialisieren damit sie den neuen kValue laedt
        gravityTds.begin();

        DPRINT(F("  CAL OK, kValue saved: ")); DDECLN((int)(kValue * 1000));
        lcd.showCalibrationMenu(11);   // "707ppm READ OK / Saving..."
        lcd.showCalibrationMenu(99);   // "Done."
        disableCalibrationMode();
        return;
      }
    }
    tdsCalibrationStep++;
  }

  // ---- Regel-Messung ----
  void run() {
    measureCount++;
    DPRINT(F("Messung #")); DDECLN(measureCount);
    set(seconds2ticks(max(5, (int)device().getList0().Messintervall())));

    currentTemperature = readTemperature();
    tds10              = readTDS();

    lcd.showMeasureValues(currentTemperature, tds10, kValue);

    tds10_cumulated       += tds10;
    temperature_cumulated += currentTemperature;

    if (measureCount >= device().getList0().Sendeintervall()) {
      // Temperatur: -40.0 (= -400 intern) als Sonderwert wenn kein DS18B20
      int16_t sendTemp = ds18b20_present
        ? (int16_t)(temperature_cumulated / measureCount)
        : (int16_t)-400;
      msg.init(device().nextcount(), sendTemp,
               (uint16_t)(tds10_cumulated / measureCount));
      device().broadcastEvent(msg);
      measureCount          = 0;
      tds10_cumulated       = 0;
      temperature_cumulated = 0;
    }
    sysclock.add(*this);
  }

  virtual void trigger (__attribute__((unused)) AlarmClock& clock) {
    if (!tdsCalibrationMode) run();
    else                     disableCalibrationMode();
  }

  void setup(Device<Hal, UList0>* dev, uint8_t number, uint16_t addr) {
    Channel::setup(dev, number, addr);

    // DS18B20 (optional)
    ds18b20_present = (Ds18b20::init(dsWire, ds18b20, 1) == 1);
    DPRINT(F("DS18B20: ")); DPRINTLN(ds18b20_present ? "OK" : "not found - using 25.0 C");

    // GravityTDS: laedt kValue automatisch aus EEPROM[8] in begin()
    gravityTds.setPin(TDS_SENSOR_PIN);
    gravityTds.setAref(REF_VOLTAGE);
    gravityTds.setAdcRange(ADC_RANGE);
    gravityTds.begin();
    // kValue direkt aus EEPROM lesen (getKvalue() liefert erst nach
    // update() den richtigen Wert, weil die Library ihn intern erst
    // beim ersten update() aus dem Konstruktor-Default ueberschreibt)
    kValue = kValueRead();
    DPRINT(F("TDS kValue loaded: ")); DDECLN((int)(kValue * 1000));

    sysclock.add(*this);
  }

  void configChanged() {
    DPRINT(F("*Temperature Offset : ")); DDECLN(this->getList1().TemperatureOffsetIndex());
  }

  uint8_t status () const { return 0; }
  uint8_t flags ()  const { return tdsCalibrationMode ? (0x01 << 1) : 0x00; }
};

// ========================================================
// Device-Typ
// ========================================================
class UType : public MultiChannelDevice<Hal, MeasureChannel, 1, UList0> {
public:
  typedef MultiChannelDevice<Hal, MeasureChannel, 1, UList0> TSDevice;
  UType(const DeviceInfo& info, uint16_t addr) : TSDevice(info, addr) {}
  virtual ~UType () {}

  virtual void configChanged () {
    TSDevice::configChanged();
    DPRINT(F("*Messintervall   : ")); DDECLN(this->getList0().Messintervall());
    DPRINT(F("*Sendeintervall  : ")); DDECLN(this->getList0().Sendeintervall());
    uint8_t bOn = this->getList0().backOnTime();
    DPRINT(F("*LCD Backlight   : ")); DDECLN(bOn);
    lcd.setBackLightOnTime(bOn);
  }
};

UType sdev(devinfo, 0x20);

// ========================================================
// Calib-Button  (identisch zu HB-UNI-Sen-EC)
//   Kurz druecken : Anlernen / Weiter im Kalibriermenü
//   Lang halten   : Kalibrierung starten/abbrechen
//   Sehr lang     : Factory Reset
// ========================================================
class CalibButton : public StateButton<HIGH, LOW, INPUT_PULLUP> {
  UType& device;
public:
  typedef StateButton<HIGH, LOW, INPUT_PULLUP> ButtonType;

  CalibButton (UType& dev, uint8_t longpresstime = 3) : device(dev) {
    this->setLongPressTime(seconds2ticks(longpresstime));
  }
  virtual ~CalibButton () {}

  virtual void state (uint8_t s) {
    uint8_t old = ButtonType::state();
    ButtonType::state(s);

    if (s == ButtonType::released) {
      if (device.channel(1).getCalibrationMode() == true)
        device.channel(1).nextCalibrationStep();
      else
        device.startPairing();
    }
    else if (s == ButtonType::pressed) {
      lcd.backlightalarm.restartTimer(sdev.getList0().backOnTime());
    }
    else if (s == ButtonType::longreleased) {
      device.channel(1).toggleTDSCalibrationMode();
    }
    else if (s == ButtonType::longpressed) {
      if (old == ButtonType::longpressed) {
        if (device.getList0().localResetDisable() == false)
          device.reset();
      } else {
        device.led().set(LedStates::key_long);
      }
    }
  }
};

CalibButton calibBtn(sdev);

// ========================================================
// setup + loop
// ========================================================
void setup () {
  DINIT(57600, ASKSIN_PLUS_PLUS_IDENTIFIER);
  sdev.init(hal);
  buttonISR(calibBtn, CONFIG_BUTTON_PIN);
  sdev.initDone();
  uint8_t serial[11];
  sdev.getDeviceSerial(serial);
  serial[10] = 0;
  lcd.initLCD(serial);
}

void loop() {
  bool worked = hal.runready();
  bool poll   = sdev.pollRadio();
  if (worked == false && poll == false) {
    hal.activity.savePower<Idle<false, true>>(hal);
  }
}
