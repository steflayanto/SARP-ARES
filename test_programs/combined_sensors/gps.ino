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

void initGPS() {
  GPS.begin(9600);
  Serial.println("GPS Found!");
  Serial1.println(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  Serial1.println(PMTK_SET_NMEA_UPDATE_1HZ);
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
