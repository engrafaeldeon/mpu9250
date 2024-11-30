#include <Arduino.h>
#include <Wire.h>
#include "i2c.h"
#include "i2c_BMP280.h"

//Endereço I2C da IMU
#define MPU 0x68

//constantes de conversão
#define A_R 16384.0 // 32768/2
#define G_R 131.0 // 32768/250

//conversão de radianos em graus 180/PI
#define RAD_A_DEG = 57.295779

//MPU-6050 da los valores en enteros de 16 bits
//Valores RAW
int16_t AcX, AcY, AcZ, GyX, GyY, GyZ;

//Angulos
float Acc[2];
float Gy[3];
float Angle[3];

String valores;

long tiempo_prev;
float dt;
float pascal;
float temperature;
static float meters;

BMP280 bmp280;

void setup()
{
  Serial.print("Probe BMP280: ");
  if (bmp280.initialize()) Serial.println("Sensor found");
  else
  {
    Serial.println("Sensor missing");
    while (1) {}
  }

  // onetime-measure:
  bmp280.setEnabled(0);
  bmp280.triggerMeasurement();

  Wire.begin(4); // D2(GPIO4)=SDA 
  Wire.begin(5); // D1(GPIO5)=SCL
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  Serial.begin(115200);
}

void loop()
{
  bmp280.awaitMeasurement();
  bmp280.getTemperature(temperature);
  bmp280.getPressure(pascal);
  bmp280.getAltitude(meters);
  bmp280.triggerMeasurement();

  //Leer los valores del Acelerometro de la IMU
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); //Pedir el registro 0x3B - corresponde al AcX
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,6,true);   //A partir del 0x3B, se piden 6 registros
  AcX=Wire.read()<<8|Wire.read(); //Cada valor ocupa 2 registros
  AcY=Wire.read()<<8|Wire.read();
  AcZ=Wire.read()<<8|Wire.read();

  //A partir de los valores del acelerometro, se calculan los angulos Y, X
  //respectivamente, con la formula de la tangente.
  Acc[1] = atan(-1*(AcX/A_R)/sqrt(pow((AcY/A_R),2) + pow((AcZ/A_R),2)))*RAD_TO_DEG;
  Acc[0] = atan((AcY/A_R)/sqrt(pow((AcX/A_R),2) + pow((AcZ/A_R),2)))*RAD_TO_DEG;

  //Leer los valores del Giroscopio
  Wire.beginTransmission(MPU);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,6,true);   //A partir del 0x43, se piden 6 registros
  GyX=Wire.read()<<8|Wire.read(); //Cada valor ocupa 2 registros
  GyY=Wire.read()<<8|Wire.read();
  GyZ=Wire.read()<<8|Wire.read();

  //Calculo del angulo del Giroscopio
  Gy[0] = GyX/G_R;
  Gy[1] = GyY/G_R;
  Gy[2] = GyZ/G_R;

  dt = (millis() - tiempo_prev) / 1000.0;
  tiempo_prev = millis();

  //Aplicar el Filtro Complementario
  Angle[0] = 0.98 *(Angle[0]+Gy[0]*dt) + 0.02*Acc[0];
  Angle[1] = 0.98 *(Angle[1]+Gy[1]*dt) + 0.02*Acc[1];

  //Integración respecto del tiempo paras calcular el YAW
  Angle[2] = Angle[2]+Gy[2]*dt;

  //Imprime os valores na interface serial no formato de captura da Extensão(serial-plotter) do VSCode, link da extensão: https://marketplace.visualstudio.com/items?itemName=badlogicgames.serial-plotter)

  Serial.print(">");
  Serial.print("angX:");
  Serial.print(String(map(Angle[0], -90, 90, 0, 1023) >> 2));
  Serial.print(",");

  Serial.print("angY:");
  Serial.print(String(map(Angle[1], -90, 90, 0, 1023) >> 2));
  Serial.print(",");

  Serial.print("angZ:");
  Serial.print(String(map(Angle[2], -90, 90, 0, 1023) >> 2));
  Serial.print(",");
  
  Serial.print("BMPTempCelsius:");
  Serial.print(String(temperature));
  Serial.print(",");

  Serial.print("BMPPressurePascal:");
  Serial.print(String(pascal));
  Serial.print(",");

  Serial.print("BMPAltitudeMetro:");
  Serial.print(String(meters));
  Serial.println();    

  delay(10);
}

