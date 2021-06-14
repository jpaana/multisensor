#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_AM2320.h>
#include <Adafruit_BME280.h>
#include <Adafruit_MLX90614.h>
#include <SQM_TSL2591.h>
#include <Adafruit_VEML6075.h>
#include <OneWire.h>
#include <DallasTemperature.h>

Adafruit_MLX90614 mlx = Adafruit_MLX90614();
Adafruit_BME280 bme; // I2C
SQM_TSL2591 sqm        = SQM_TSL2591(2591);
Adafruit_AM2320 am2320 = Adafruit_AM2320();
Adafruit_VEML6075 uv   = Adafruit_VEML6075();

#define ONE_WIRE_BUS 10
#define TEMPERATURE_PRECISION 12

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature ds18B20(&oneWire);
DeviceAddress thermometer[4];
uint8_t numTherm = 0;

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
        Serial.println(F("No thermometers found!"));
    }

    for (uint8_t i = 0; i < numTherm; i++)
    {
        if (!ds18B20.getAddress(thermometer[i], i))
        {
            Serial.print(F("Unable to find address for Device "));
            Serial.println(i);
        }
        else
        {
            ds18B20.setResolution(thermometer[i], TEMPERATURE_PRECISION);
        }
    }

    sqm.config.gain = TSL2591_GAIN_LOW;
    sqm.config.time = TSL2591_INTEGRATIONTIME_200MS;
    sqm.configSensor();
    sqm.setCalibrationOffset(0.0);
    sqm.verbose = false;

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

void setup()
{
    Serial.begin(9600);
    while (!Serial)
        ; // time to get serial running

    ds18B20.begin();
    mlx.begin();
    bme.begin();
    sqm.begin();
    am2320.begin();
    uv.begin();
    configureSensors();
    ds18B20.requestTemperatures();
}

float correctSkyTemperature(float IR, float T)
{
//Cloudy sky is warmer that clear sky. Thus sky temperature meassure by IR sensor
//is a good indicator to estimate cloud cover. However IR really meassure the
//temperatura of all the air column above increassing with ambient temperature.
//So it is important include some correction factor:
//From AAG Cloudwatcher formula. Need to improve futher.
//http://www.aagware.eu/aag/cloudwatcher700/WebHelp/index.htm#page=Operational%20Aspects/23-TemperatureFactor-.htm
//Sky temp correction factor. Tsky=Tsky_meassure - Tcorrection
//Formula Tcorrection = (K1 / 100) * (Thr - K2 / 10) + (K3 / 100) * pow((exp (K4 / 1000* Thr)) , (K5 / 100));
#define  K1 33.f
#define  K2 0.f
#define  K3 4.f
#define  K4 100.f
#define  K5 100.f

  float Td = (K1 / 100.f) * (T - K2 / 10.f) + (K3 / 100.f) * powf((expf(K4 / 1000.f * T)) , (K5 / 100.f));
  float Tsky = IR - Td;
  return Tsky;
}

void loop()
{
    if (Serial.available())
    {
        Serial.read();
        Serial.println("{");
        float ambient, object, difference;
        ambient    = mlx.readAmbientTempC();
        object     = correctSkyTemperature(mlx.readObjectTempC(), ambient);
        difference = ambient - object;
        Serial.print(F("\"mlx90614\": {\"ambient\":"));
        Serial.print(ambient, 3);
        Serial.print(F(", \"object\":"));
        Serial.print(object, 3);
        Serial.print(F(", \"difference\":"));
        Serial.print(difference, 3);
        Serial.println(F("},"));

        Serial.print(F("\"bme280\": {\"temperature\":"));
        Serial.print(bme.readTemperature(), 3);
        Serial.print(F(", \"pressure\":"));
        Serial.print(bme.readPressure() / 100.0F);
        Serial.print(F(", \"humidity\":"));
        Serial.print(bme.readHumidity(), 3);
        Serial.println(F("},"));

        Serial.print(F("\"am2320b\": {\"temperature\":"));
        Serial.print(am2320.readTemperature(), 3);
        Serial.print(F(", \"humidity\":"));
        Serial.print(am2320.readHumidity(), 3);
        Serial.println(F("},"));

        sqm.setTemperature(bme.readTemperature());
        sqm.takeReading();
        Serial.print(F("\"tsl2591\": {\"lux\":"));
        Serial.print(sqm.calculateLux(sqm.full, sqm.ir), 6);
        Serial.print(F(", \"mpsas\":"));
        Serial.print(sqm.mpsas, 3);
        Serial.print(F(", \"dmpsas\":"));
        Serial.print(sqm.dmpsas, 3);
        Serial.println(F("},"));

        Serial.print(F("\"veml6075\": {\"uva\":"));
        Serial.print(uv.readUVA(), 6);
        Serial.print(F(", \"uvb\":"));
        Serial.print(uv.readUVB(), 6);
        Serial.print(F(", \"uvi\":"));
        Serial.print(uv.readUVI());
        Serial.println(F("},"));

        ds18B20.requestTemperatures();
        Serial.print(F("\"ds18b20\": {"));
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
        Serial.println(F("},"));

        Serial.print(F("\"rain\": {\"analog\":"));
        Serial.print((1023 - analogRead(A0)) / 1023.0f);
        Serial.print(F(", \"digital\":"));
        Serial.print(1 - digitalRead(7));
        Serial.println(F("}"));

        Serial.println(F("}"));
    }
}
