/*
  Waits for acceleration to go above a threshold, then sends IMU and environmental readings over debug and uart
*/

#include <Arduino_LSM9DS1.h>
#include <Arduino_LPS22HB.h>

const float accelerationThreshold = 2.5; // threshold of significant motion in g
char outBuf[2048];
float aX, aY, aZ, gX, gY, gZ, mX, mY, mZ;
float pressure, temperature, altitude;
bool motion = false;

void setup() 
{
  Serial.begin(9600);
  Serial1.begin(115200);
  while (!Serial);

  if (!IMU.begin()) 
  {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  if (!BARO.begin()) 
  {
    Serial.println("Failed to initialize pressure sensor!");
    while (1);
  }

  // print a csv-ish header
  Serial.println("aX,aY,aZ,gX,gY,gZ,mX,mY,mZ,p,T,a");
}

void sendReport()
{
  sprintf(outBuf, "{\"Accel:\"{\"x\":%.2f,\"y\":%.2f,\"z\":%.2f},\"Mag\":{\"x\":%.2f,\"y\":%.2f,\"z\":%.2f},\"Gyro\":{\"x\":%.2f\"y\":%.2f\":z\":%.2f},\"Pressure\":%.2f,\"Temp\":%.2f,\"Alt\":%.2f}",aX,aY,aZ,gX,gY,gZ,mX,mY,mZ,pressure,temperature,altitude);
  Serial1.println(outBuf);
}

void loop() 
{
  String inString; 

  if(Serial1.available() > 0)
  {
    inString = Serial1.readString();
    inString.trim();
    if (inString=="REPORT")
    {
      Serial.println("Report requested via UART");
      sendReport();
    }
  }
  
  if ((!motion)&&(IMU.accelerationAvailable()))
  {
    // read the acceleration data
    IMU.readAcceleration(aX, aY, aZ);

    // sum up the absolutes
    float aSum = fabs(aX) + fabs(aY) + fabs(aZ);

    // check if it's above the threshold
    if (aSum >= accelerationThreshold) 
    {
      motion = true;
    }
  }

  if(motion) 
  {
    // Check data available
    if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable() && IMU.magneticFieldAvailable()) 
    {
      pressure = BARO.readPressure();
      temperature = BARO.readTemperature();
      altitude = 44330 * ( 1 - pow(pressure/101.325, 1/5.255) );

      // read the acceleration, mag, and gyroscope data
      IMU.readAcceleration(aX, aY, aZ);
      IMU.readGyroscope(gX, gY, gZ);
      IMU.readMagneticField(mX, mY, mZ);

      // print the data locally
      sprintf(outBuf, "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",aX,aY,aZ,gX,gY,gZ,mX,mY,mZ,pressure,temperature,altitude);
      Serial.println(outBuf);
 
      sendReport();
      motion = false;
    }
  }
}