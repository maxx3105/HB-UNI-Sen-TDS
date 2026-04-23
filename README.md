# HB-UNI-Sen-TDS

HomeMatic / AskSin++ kompatibler TDS-Sensor auf Basis des **DFRobot Gravity: Analog TDS Sensor / Meter (SEN0244)**.

Basiert auf [HB-UNI-Sen-EC](https://github.com/jp112sdl) von jp112sdl.

---

## Funktionen

- TDS-Messung (Total Dissolved Solids) in ppm mit einer Nachkommastelle
- Optionale Temperaturkompensation via DS18B20 (ohne Sensor: Fallback 25,0 °C)
- Kalibrierung über Taster mit 707 ppm / 1413 µS/cm Standardlösung (dieselbe wie beim EC-Sensor)
- kValue wird im EEPROM gespeichert und beim Start automatisch geladen
- LCD-Anzeige (I²C, 16×2) mit aktuellem TDS-Wert, Temperatur und kValue
- Sendeintervall und Messintervall über CCU WebUI konfigurierbar
- Device Model: **0xFC30**
- Benötigt: [JP-HB-Devices-addon](https://github.com/jp112sdl/JP-HB-Devices-addon)

---

## Hardware

| Bauteil | Pin |
|---|---|
| CC1101 SCK | D13 |
| CC1101 MISO | D12 |
| CC1101 MOSI | D11 |
| CC1101 CS | D10 |
| CC1101 GDO0 | D2 |
| Config-Taster | D8 |
| Status-LED | D4 |
| DS18B20 (optional) | D5 |
| TDS Sensor (SEN0244) | A1 |
| LCD I²C SDA | A4 |
| LCD I²C SCL | A5 |

> Bei 3,3V-Boards `REF_VOLTAGE` im Sketch auf `3.3f` ändern (Standard: `5.0f`).

---

## Benötigte Libraries

- [AskSinPP](https://github.com/pa-pa/AskSinPP)
- [EnableInterrupt](https://github.com/GreyGnome/EnableInterrupt)
- [GravityTDS](https://github.com/DFRobot/GravityTDS)
- [LiquidCrystal_I2C](https://github.com/johnrickman/LiquidCrystal_I2C)
- OneWire + DallasTemperature (für DS18B20)

---

## Taster-Bedienung

| Aktion | Funktion |
|---|---|
| Kurz drücken | Anlernen (außerhalb Kalibriermodus) |
| Kurz drücken | Nächster Kalibrierungsschritt (im Kalibriermodus) |
| Lang halten ≥ 3 s | Kalibriermodus starten / abbrechen |
| Sehr lang halten (doppelt) | Factory Reset |

---

## Kalibrierung

1. Lang drücken → LCD: **"TDS CALIBRATION"**
2. Kurz drücken → LCD: **"Put 707ppm sol."** → Sonde in 707 ppm / 1413 µS/cm Lösung tauchen
3. Kurz drücken → Messung, kValue-Berechnung, EEPROM-Speicherung → LCD: **"Done. Saved."**

Bei Fehler (kein Signal oder kValue außerhalb 0,25–4,0) erscheint eine Fehlermeldung und der Ablauf springt zu Schritt 1 zurück.

> Die 1413 µS/cm Kalibrierlösung des EC-Sensors kann direkt verwendet werden.

---

## CCU XML

Die Datei `hb-uni-sen-tds.xml` in das AddOn-Verzeichnis des JP-HB-Devices-Addon kopieren:

```
/firmware/rftypes/hb-uni-sen-tds.xml
```

### Datenpunkte (Kanal 1)

| Datenpunkt | Einheit | Bereich |
|---|---|---|
| `TEMPERATURE` | °C | −40,0 … 80,0 |
| `HB_TDS` | ppm | 0,0 … 9999,9 |
| `INFO_MSG` | — | NORMAL / CALIBRATION |

---

## Lizenz

Creative Commons BY-NC-SA 3.0 DE
