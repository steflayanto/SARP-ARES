//#include <SoftwareSerial.h> // only if using uno

#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_BME280.h>
#include <Adafruit_GPS.h>

Adafruit_MPU6050 mpu;
Adafruit_BME280 bme;
#define SEALEVELPRESSURE_HPA (1013.25)

#define GPS_TX 3
#define GPS_RX 2
//SoftwareSerial Serial1(GPS_TX, GPS_RX);
Adafruit_GPS GPS(&Serial1);

#define PMTK_SET_NMEA_UPDATE_1HZ  "$PMTK220,1000*1F"
#define PMTK_SET_NMEA_UPDATE_5HZ  "$PMTK220,200*2C"
#define PMTK_SET_NMEA_UPDATE_10HZ "$PMTK220,100*2F"

// turn on only the second sentence (GPRMC)
#define PMTK_SET_NMEA_OUTPUT_RMCONLY "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"
// turn on GPRMC and GGA
#define PMTK_SET_NMEA_OUTPUT_RMCGGA "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
// turn on ALL THE DATA
#define PMTK_SET_NMEA_OUTPUT_ALLDATA "$PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
// turn off output
#define PMTK_SET_NMEA_OUTPUT_OFF "$PMTK314,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"

#define PMTK_Q_RELEASE "$PMTK605*31"

int minTimestep = 20;   // ms
unsigned long loopNumber;
unsigned long lastAccelEventTime = 0;
unsigned long lastGyroEventTime = 0;

float accelBiases[3];
float gyroBiases[3];
float groundLevelPressure;

float position[3] = {0,0,0};
float velocity[3] = {0,0,0};
float acceleration[3] = {0,0,0};

float angularPosition[3];
float angularVelocity[3];

void setup(void) {
  Serial.begin(115200);
  
  while (!Serial)
    delay(10); // will pause Arduino until serial console opens

  Serial.println("Testing MPU6050 IMU, BME280 barometer, and Adafruit GPS");

  // Try to initialize
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    /*while (1) {
      delay(10);
    }*/
  }
  Serial.println("MPU6050 Found!");
  delay(10);
  
  if (!bme.begin()) {
    Serial.println("Failed to find BME280 chip");
    /*while (1) {
      delay(10);
    }*/
  }
  Serial.println("BME280 Found!");

  GPS.begin(9600);
  Serial.println("GPS Found!");
  delay(10);
  
  Serial1.println(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  Serial1.println(PMTK_SET_NMEA_UPDATE_1HZ);

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("IMU filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }  
  
  Serial.println("");
  
  setAllBiases();

  delay(50);
}

void loop() {
  
  getPosVelAcc();
  getAngPosVel();

  if (loopNumber % 10 == 0) {
    
    //printIMUReadings();
    //printBaroReadings();
    
    printPosVelAcc();
    printAngPosVel();
  }

  if (loopNumber % 10 == 0) {
    //printGPSReadings();
  }
  
  loopNumber++;
  delay(minTimestep);
}

void getPosVelAcc() {
  if (lastAccelEventTime != 0) {
    sensors_event_t a;
    mpu.getAccelEvent(&a);
    
    acceleration[0] = a.acceleration.x;// - accelBiases[0];
    acceleration[1] = a.acceleration.y;// - accelBiases[1];
    acceleration[2] = a.acceleration.z;// - accelBiases[2];

    float magnitude = 0;
    float biasedMagnitude = 0;
    
    /*Serial.print("Magnitude without bias: ");
    magnitude = (a.acceleration.x * a.acceleration.x) + (a.acceleration.y * a.acceleration.y) + (a.acceleration.z * a.acceleration.z);
    Serial.println(sqrt(magnitude));
    Serial.print("Magnitude with bias: ");
    biasedMagnitude = (acceleration[0] * acceleration[0]) + (acceleration[1] * acceleration[1]) + (acceleration[2] * acceleration[2]);
    Serial.println(sqrt(biasedMagnitude));*/
  
    float dt = (float)(millis()-lastAccelEventTime) / 1000;
    for (int d = 0; d < 3; d++) {
      position[d] += velocity[d] * dt + 0.5 * acceleration[d] * dt*dt;
      velocity[d] += acceleration[d] * dt;
    }
  }
  
  lastAccelEventTime = millis();
}

void getAngPosVel() {
  if (lastGyroEventTime != millis()) {
    sensors_event_t g;
    mpu.getGyroEvent(&g);
    
    float dt = (float)(millis()-lastGyroEventTime) / 1000;
    for (int d = 0; d < 3; d++) {
      angularPosition[d] += angularVelocity[d] * dt;
    }
  
    angularVelocity[0] = (g.gyro.x - gyroBiases[0]); // experimental scaling factor?
    angularVelocity[1] = (g.gyro.y - gyroBiases[1]);
    angularVelocity[2] = (g.gyro.z - gyroBiases[2]);
  }

  lastGyroEventTime = millis();
}

void getGPS() {
  
}

void setAllBiases() {
  
  /*Take a time average of all accel, gyro, and baro readings over 5 seconds*/
  int numSamples = (int)(1000 / minTimestep);

  float xAccelSum = 0;
  float yAccelSum = 0;
  float zAccelSum = 0;

  float xGyroSum = 0;
  float yGyroSum = 0;
  float zGyroSum = 0;

  float baroSum = 0;

  Serial.println("Finding Biases...");
  
  for (int t = 0; t < numSamples; t++) {
    /* Take new IMU reading */
    mpu.read();
        
    /* Get new sensor events with IMU readings */
    sensors_event_t a, g;
    mpu.getAccelEvent(&a);
    mpu.getGyroEvent(&g);
    
    xAccelSum += a.acceleration.x;
    yAccelSum += a.acceleration.y;
    zAccelSum += a.acceleration.z;

    xGyroSum += g.gyro.x;
    yGyroSum += g.gyro.y;
    zGyroSum += g.gyro.z;

    baroSum += bme.readPressure();

    delay (minTimestep);
  }
  
  accelBiases[0] = xAccelSum / numSamples;
  accelBiases[1] = yAccelSum / numSamples;
  accelBiases[2] = zAccelSum / numSamples;// - 9.8;
  
  gyroBiases[0] = xGyroSum / numSamples;
  gyroBiases[1] = yGyroSum / numSamples;
  gyroBiases[2] = zGyroSum / numSamples;

  groundLevelPressure = baroSum / numSamples;

  Serial.println("Done!");
}

void printPosVelAcc() {
  /*Serial.print("Position  x: ");
  Serial.print((position[0] < 0) ? "-" : " ");
  Serial.print(abs(position[0]));
  Serial.print(",\ty: ");
  Serial.print((position[1] < 0) ? "-" : " ");
  Serial.print(abs(position[1]));
  Serial.print(",\tz: ");
  Serial.print((position[2] < 0) ? "-" : " ");
  Serial.print(abs(position[2]));
  Serial.println(" m");*/

  Serial.print("Velocity  x: ");
  Serial.print((velocity[0] < 0) ? "-" : " ");
  Serial.print(abs(velocity[0]));
  Serial.print(",\ty: ");
  Serial.print((velocity[1] < 0) ? "-" : " ");
  Serial.print(abs(velocity[1]));
  Serial.print(",\tz: ");
  Serial.print((velocity[2] < 0) ? "-" : " ");
  Serial.print(abs(velocity[2]));
  Serial.println(" m/s");

  Serial.print("Acceleration  x: ");
  Serial.print((acceleration[0] < 0) ? "-" : " ");
  Serial.print(abs(acceleration[0]));
  Serial.print(",\ty: ");
  Serial.print((acceleration[1] < 0) ? "-" : " ");
  Serial.print(abs(acceleration[1]));
  Serial.print(",\tz: ");
  Serial.print((acceleration[2] < 0) ? "-" : " ");
  Serial.print(abs(acceleration[2]));
  Serial.println(" m/s^2");

  Serial.println("");
}

void printAngPosVel() {
  
  Serial.print("Angular Position  x: ");
  Serial.print((angularPosition[0] < 0) ? "-" : " ");
  Serial.print(abs(angularPosition[0]));
  Serial.print(",\ty: ");
  Serial.print((angularPosition[1] < 0) ? "-" : " ");
  Serial.print(abs(angularPosition[1]));
  Serial.print(",\tz: ");
  Serial.print((angularPosition[2] < 0) ? "-" : " ");
  Serial.print(abs(angularPosition[2]));
  Serial.println(" deg");

  Serial.print("Angular Velocity  x: ");
  Serial.print((angularVelocity[0] < 0) ? "-" : " ");
  Serial.print(abs(angularVelocity[0]));
  Serial.print(",\ty: ");
  Serial.print((angularVelocity[1] < 0) ? "-" : " ");
  Serial.print(abs(angularVelocity[1]));
  Serial.print(",\tz: ");
  Serial.print((angularVelocity[2] < 0) ? "-" : " ");
  Serial.print(abs(angularVelocity[2]));
  Serial.println(" deg/s");

  Serial.println(" ");
}

void printIMUReadings() {

  /* Print out the values */
  Serial.print(" Accelerometer  x: ");
  Serial.print((acceleration[0] < 0) ? "-" : " ");
  Serial.print(abs(acceleration[0]));
  Serial.print(",  y: ");
  Serial.print((acceleration[1] < 0) ? "-" : " ");
  Serial.print(abs(acceleration[1]));
  Serial.print(",  z: ");
  Serial.print((acceleration[2] < 0) ? "-" : " ");
  Serial.print(abs(acceleration[2]));
  Serial.println(" m/s^2");

  Serial.print(" Gyroscope      x: ");
  Serial.print((angularVelocity[0] < 0) ? "-" : " ");
  Serial.print(abs(angularVelocity[0]));
  Serial.print(",  y: ");
  Serial.print((angularVelocity[1] < 0) ? "-" : " ");
  Serial.print(abs(angularVelocity[1]));
  Serial.print(",  z: ");
  Serial.print((angularVelocity[2] < 0) ? "-" : " ");
  Serial.print(abs(angularVelocity[2]));
  Serial.println(" deg/s");

  Serial.println("");
}

void printBaroReadings() {
    Serial.print(" Temperature: ");
    Serial.print(bme.readTemperature());
    Serial.println(" *C");

    Serial.print(" Pressure: ");

    Serial.print(bme.readPressure() / 100.0F);
    Serial.println(" hPa");

    Serial.print(" Altitude: ");
    Serial.print(bme.readAltitude(groundLevelPressure / 100.0));  // Takes base value in hPa
    Serial.println(" m");

    /*Serial.print(" Humidity = ");
    Serial.print(bme.readHumidity());
    Serial.println(" %");*/

    Serial.println();
}

void printGPSReadings() {
  Serial.print("\nTime: ");
  if (GPS.hour < 10) { Serial.print('0'); }
  Serial.print(GPS.hour, DEC); Serial.print(':');
  if (GPS.minute < 10) { Serial.print('0'); }
  Serial.print(GPS.minute, DEC); Serial.print(':');
  if (GPS.seconds < 10) { Serial.print('0'); }
  Serial.print(GPS.seconds, DEC); Serial.print('.');
  if (GPS.milliseconds < 10) {
    Serial.print("00");
  } else if (GPS.milliseconds > 9 && GPS.milliseconds < 100) {
    Serial.print("0");
  }
  Serial.println(GPS.milliseconds);
  Serial.print(" Date: ");
  Serial.print(GPS.day, DEC);   Serial.print('/');
  Serial.print(GPS.month, DEC); Serial.print("/20");
  Serial.println(GPS.year, DEC);
  Serial.print(" Fix: ");       Serial.print((int)GPS.fix);
  Serial.print(" quality: ");   Serial.println((int)GPS.fixquality);
  if (GPS.fix) {
    Serial.print(" Location: ");
    Serial.print(GPS.latitude, 4);    Serial.print(GPS.lat);
    Serial.print(", ");
    Serial.print(GPS.longitude, 4);   Serial.println(GPS.lon);

    Serial.print("Speed (knots): ");  Serial.println(GPS.speed);
    Serial.print("Angle: ");          Serial.println(GPS.angle);
    Serial.print("Altitude: ");       Serial.println(GPS.altitude);
    Serial.print("Satellites: ");     Serial.println((int)GPS.satellites);
  }
}
