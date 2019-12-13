#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_MPU6050 mpu;
Adafruit_BME280 bme;

int minTimeStep = 20;   // ms

float accelBiases[3];
float gyroBiases[3];
float groundLevelPressure;

float position[3] = {0,0,0};
float velocity[3] = {0,0,0};
float acceleration[3] = {0,0,0};

void initIMU() {
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
  }else{
    Serial.println("Found MPU6050 chip");
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

void initBME() {
  if (!bme.begin()) {
    Serial.println("Failed to find BME280 chip");
  }else{
    Serial.println("Found BE280 chip");
  }
}

void setIMUBMEBiases() {
  
  /*Take a time average of all accel, gyro, and baro readings over 5 seconds*/
  int numSamples = (int)(1000 / minTimeStep);

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

    delay (minTimeStep);
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
