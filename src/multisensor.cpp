
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_AM2320.h>
#include <Adafruit_BME280.h>
#include <Adafruit_MLX90614.h>
//#include <Adafruit_TSL2591.h>
#include <SQM_TSL2591.h>
#include <Adafruit_VEML6075.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <LiquidCrystal.h>

// initialize the library by associating any needed LCD interface pin
// with the arduino pin number it is connected to
const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_MLX90614 mlx = Adafruit_MLX90614();
Adafruit_BME280 bme; // I2C
//Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591);
SQM_TSL2591 sqm        = SQM_TSL2591(2591);
Adafruit_AM2320 am2320 = Adafruit_AM2320();
Adafruit_VEML6075 uv   = Adafruit_VEML6075();

#define ONE_WIRE_BUS 10
#define TEMPERATURE_PRECISION 12

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature ds18B20(&oneWire);
DeviceAddress thermometer[4];
uint8_t numTherm = 0;

uint8_t state                = 0;
unsigned long previousMillis = 0;

//const tsl2591Gain_t gain[4] =  { TSL2591_GAIN_LOW, TSL2591_GAIN_MED, TSL2591_GAIN_HIGH, TSL2591_GAIN_MAX };
//char currentGain = 0;

void configureSensors(void)
{
    // Rain sensor
    pinMode(A0, INPUT);
    pinMode(7, INPUT);

    // Setup DS18B20
    numTherm = ds18B20.getDS18Count();
    if (numTherm > 4)
        numTherm = 4;

    if (numTherm == 0)
    {
        Serial.println("No thermometers found!");
    }

    for (uint8_t i = 0; i < numTherm; i++)
    {
        if (!ds18B20.getAddress(thermometer[i], i))
        {
            Serial.print("Unable to find address for Device ");
            Serial.println(i);
        }
        else
        {
            ds18B20.setResolution(thermometer[i], TEMPERATURE_PRECISION);
        }
    }

#if 0
  // Setup TSL2591
  // You can change the gain on the fly, to adapt to brighter/dimmer light situations
  //tsl.setGain(TSL2591_GAIN_LOW);    // 1x gain (bright light)
  //tsl.setGain(TSL2591_GAIN_MED);      // 25x gain
  //tsl.setGain(TSL2591_GAIN_HIGH);   // 428x gain
  tsl.setGain(gain[currentGain]);

  // Changing the integration time gives you a longer time over which to sense light
  // longer timelines are slower, but are good in very low light situtations!
  //tsl.setTiming(TSL2591_INTEGRATIONTIME_100MS);  // shortest integration time (bright light)
  // tsl.setTiming(TSL2591_INTEGRATIONTIME_200MS);
  // tsl.setTiming(TSL2591_INTEGRATIONTIME_300MS);
  // tsl.setTiming(TSL2591_INTEGRATIONTIME_400MS);
  // tsl.setTiming(TSL2591_INTEGRATIONTIME_500MS);
  tsl.setTiming(TSL2591_INTEGRATIONTIME_600MS);  // longest integration time (dim light)
#else
    sqm.config.gain = TSL2591_GAIN_LOW;
    sqm.config.time = TSL2591_INTEGRATIONTIME_200MS;
    sqm.configSensor();
    sqm.setCalibrationOffset(0.0);
    sqm.verbose = false;
#endif
    // Setup VEML6075
    // Set the integration constant
    uv.setIntegrationTime(VEML6075_100MS);
    uv.setHighDynamic(false);
    uv.setForcedMode(false);
    // Set the calibration coefficients
    uv.setCoefficients(2.22, 1.33,          // UVA_A and UVA_B coefficients
                       2.95, 1.74,          // UVB_C and UVB_D coefficients
                       0.001461, 0.002591); // UVA and UVB responses
}

#if 0
float tsl2591Read(void)
{
  uint16_t ir, full;

  // More advanced data read example. Read 32 bits with top 16 bits IR, bottom 16 bits full spectrum
  // That way you can do whatever math and comparisons you want!
  uint32_t lum = tsl.getFullLuminosity();
  ir = lum >> 16;
  full = lum & 0xFFFF;

  if(full == 0xffff && currentGain != 0)
  {
    // Lower gain
    currentGain--;
    tsl.setGain(gain[currentGain]);
  }

  if(full < 2000 && currentGain != 3)
  {
    currentGain++;
    tsl.setGain(gain[currentGain]);
  }
  return tsl.calculateLux(full, ir);
}
#endif

void setup()
{
    Serial.begin(9600);
    while (!Serial)
        ; // time to get serial running

    ds18B20.begin();
    mlx.begin();
    bme.begin();
    //  tsl.begin();
    sqm.begin();
    am2320.begin();
    uv.begin();
    configureSensors();

    lcd.begin(16, 2);
    lcd.print("Multisensor");

    // Contrast setting
    pinMode(9, OUTPUT);
    digitalWrite(9, LOW);

    ds18B20.requestTemperatures();
}

void loop()
{
    if (Serial.available())
    {
        Serial.read();
        Serial.println("{");
        float ambient, object, difference;
        ambient    = mlx.readAmbientTempC();
        object     = mlx.readObjectTempC();
        difference = ambient - object;
        Serial.print("\"mlx90614\": {\"ambient\":");
        Serial.print(ambient);
        Serial.print(", \"object\":");
        Serial.print(object);
        Serial.print(", \"difference\":");
        Serial.print(difference);
        Serial.println("},");

        Serial.print("\"bme280\": {\"temperature\":");
        Serial.print(bme.readTemperature());
        Serial.print(", \"pressure\":");
        Serial.print(bme.readPressure() / 100.0F);
        Serial.print(", \"humidity\":");
        Serial.print(bme.readHumidity());
        Serial.println("},");

        Serial.print("\"am2023b\": {\"temperature\":");
        Serial.print(am2320.readTemperature());
        Serial.print(", \"humidity\":");
        Serial.print(am2320.readHumidity());
        Serial.println("},");

        sqm.setTemperature(bme.readTemperature());
        sqm.takeReading();
        Serial.print("\"tsl2591\": {\"lux\":");
        Serial.print(sqm.calculateLux(sqm.full, sqm.ir));
        Serial.print(", \"mpsas\":");
        Serial.print(sqm.mpsas);
        Serial.print(", \"dmpsas\":");
        Serial.print(sqm.dmpsas);
        Serial.println("},");

        Serial.print("\"veml6075\": {\"uva\":");
        Serial.print(uv.readUVA());
        Serial.print(", \"uvb\":");
        Serial.print(uv.readUVB());
        Serial.print(", \"uvi\":");
        Serial.print(uv.readUVI());
        Serial.println("},");

        ds18B20.requestTemperatures();
        Serial.print("\"ds18b20\": {");
        for (int i = 0; i < numTherm; i++)
        {
            char str[24];
            char flt[16];

            float tempC = ds18B20.getTempC(thermometer[i]);
            dtostrf(tempC, 6, 4, flt);
            if (i != 0)
                Serial.print(", ");
            sprintf(str, "\"temp%d\":%s", i, flt);
            Serial.print(str);
        }
        Serial.println("},");

        Serial.print("\"rain\": {\"analog\":");
        Serial.print((1023 - analogRead(A0)) / 1023.0f);
        Serial.print(", \"digital\":");
        Serial.print(1 - digitalRead(7));
        Serial.println("}");

        Serial.println("}");
    }

    unsigned long currentMillis = millis();
    unsigned long interval      = 3000;
    if (currentMillis - previousMillis >= interval)
    {
        previousMillis = currentMillis;

        char value[17];
        lcd.clear();
        switch (state)
        {
            case 0:
            {
                lcd.print("DS18B20 #1:");
                float tempC = ds18B20.getTempC(thermometer[0]);
                dtostrf(tempC, 6, 4, value);
                break;
            }
            case 1:
            {
                lcd.print("DS18B20 #2:");
                float tempC = ds18B20.getTempC(thermometer[1]);
                dtostrf(tempC, 6, 4, value);
                break;
            }
            case 2:
            {
                lcd.print("AM2320B Temp:");
                float tempC = am2320.readTemperature();
                dtostrf(tempC, 6, 4, value);
                break;
            }
            case 3:
            {
                lcd.print("BME280 Temp:");
                float tempC = bme.readTemperature();
                dtostrf(tempC, 6, 4, value);
                break;
            }
            case 4:
            {
                lcd.print("MLX Ambient:");
                float tempC = mlx.readAmbientTempC();
                dtostrf(tempC, 6, 4, value);
                break;
            }
            case 5:
            {
                lcd.print("MLX Object:");
                float tempC = mlx.readObjectTempC();
                dtostrf(tempC, 6, 4, value);
                break;
            }
            case 6:
            {
                lcd.print("AM2320B Humidity:");
                float tempC = am2320.readHumidity();
                dtostrf(tempC, 6, 4, value);
                break;
            }
            case 7:
            {
                lcd.print("BM280 Humidity:");
                float tempC = bme.readHumidity();
                dtostrf(tempC, 6, 4, value);
                break;
            }
            case 8:
            {
                lcd.print("BM280 Pressure:");
                float tempC = bme.readPressure() / 100.0f;
                dtostrf(tempC, 6, 4, value);
                break;
            }
            case 9:
            {
                lcd.print("TSL2591 Lux:");
                float light = sqm.calculateLux(sqm.full, sqm.ir);
                dtostrf(light, 6, 8, value);
                break;
            }
            case 10:
            {
                lcd.print("TSL2591 MPSAS:");
                dtostrf(sqm.mpsas, 6, 8, value);
                break;
            }
            case 11:
            {
                lcd.print("TSL2591 DMPSAS:");
                dtostrf(sqm.dmpsas, 6, 8, value);
                break;
            }
            case 12:
            {
                lcd.print("VEML6075 UVA:");
                float uva = uv.readUVA();
                dtostrf(uva, 6, 4, value);
                break;
            }
            case 13:
            {
                lcd.print("VEML6075 UVB:");
                float uva = uv.readUVB();
                dtostrf(uva, 6, 4, value);
                break;
            }
            case 14:
            {
                lcd.print("VEML6075 UVI:");
                float uva = uv.readUVI();
                dtostrf(uva, 6, 4, value);
                break;
            }
        }
        lcd.setCursor(0, 1);
        lcd.print(value);

        state++;
        if (state > 14)
        {
            state = 0;
            ds18B20.requestTemperatures();
            sqm.setTemperature(bme.readTemperature());
            sqm.takeReading();
        }
    }
}
