#include "BluetoothSerial.h"
#include "PCF8574.h"
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include "SoftwareSerial.h"
#include <Wire.h>
#include <TinyGPS++.h>
#include "DFRobotDFPlayerMini.h"
#include <FastLED.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>


// ============================================================================
// HARDWARE CONFIGURATION
// ============================================================================

// E-compass Configuration
#define I2C_SDA 21
#define I2C_SCL 22

// DF Player Configuration
#define DFPLAYER_RX 14
#define DFPLAYER_TX 15

// SD Card Configuration
#define REASSIGN_PINS
#define SCK_PIN 23
#define MISO_PIN 19
#define MOSI_PIN 18
#define CS_PIN 5

// Motor Configuration
#define LFPWM 12
#define LBPWM 13
#define RFPWM 27
#define RBPWM 4

#define DEADZONE 15
#define RAMP_STEP 30
#define LOOP_DELAY 5
#define BT_TIMEOUT 5000

// Encoder Configuration
#define LEFT_ENCODER_PIN P0
#define RIGHT_ENCODER_PIN P2
#define WHEEL_CIRCUMFERENCE 0.6126
#define MAGNETS_PER_WHEEL 5
#define METERS_PER_TICK (WHEEL_CIRCUMFERENCE / MAGNETS_PER_WHEEL)

// RGB Configuration
#define LED_PIN     2
#define NUM_LEDS    10
#define BRIGHTNESS  64
#define LED_TYPE    WS2811
#define COLOR_ORDER GRB

#define UPDATES_PER_SECOND 100

// ============================================================================
// DATA STRUCTURES
// ============================================================================

struct Position {
  double latitude;
  double longitude;
};

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

// Hardware Objects
BluetoothSerial SerialBT;
PCF8574 pcf1(0x26);
PCF8574 pcf2(0x27);
HardwareSerial gpsSerial1(1);
HardwareSerial gpsSerial2(2);
TinyGPSPlus gps1;
TinyGPSPlus gps2;
SoftwareSerial DFPlayerSerial(DFPLAYER_RX, DFPLAYER_TX);
DFRobotDFPlayerMini myDFPlayer;
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(0x1E);
CRGB leds[NUM_LEDS];

// Device Configuration
String device_name = "Lawn Mower";

// E-compass Variables
sensors_event_t event; 
float heading;
// Hard iron offsets (bias)
float offsetX = 5.0455, offsetY = -26.9091;

// Soft iron scale factors
float scaleX = 0.7640, scaleY = 0.7520;

// Motor Control Variables
int currentLeft = 0, currentRight = 0;
int targetLeft = 0, targetRight = 0;

// Bluetooth Variables
String buffer = "";
unsigned long lastBtDataTime = 0;
bool btConnected = false;

// GPS & Navigation Variables
Position currentLocation;
Position currentTarget;
Position startLocation;
bool gpsStayStill = 0;
bool AutoDriving = false;
bool navigatingToWaypoint = false;
int currentWaypoint = 1;
unsigned long delayGPS = 0;
unsigned long gpsTime = 0;
bool gps1OK =0; 
bool gps2OK=0;
float headingBearing =0.0;
bool requirePosition =0;

// Relay Control Variables
bool GS12_1 = false, GS12_2 = false, GS5_1 = false, GS5_2 = false;

// Encoder Variables
unsigned long lastLeftTime = 0;
unsigned long lastRightTime = 0;
bool  lastLeftState = 0;
bool  lastRightState =0;
unsigned long leftEncoderCount = 0;
unsigned long rightEncoderCount = 0;
unsigned long lastLeftCount = 0, lastRightCount = 0, lastEncoderTime = 0;
float leftDistance = 0.0, rightDistance = 0.0, totalDistance = 0.0;
bool currentLeftState = 0;
bool currentRightState =0;

// Waypoint Management
unsigned int position_count = 0;
unsigned int x_val = 0, y_val = 0;
bool x_isNegative = false, y_isNegative = false;
bool turningRN = 0;
bool gpsReach =0;

// Hall Sensor Variables
bool left1_State = false, left2_State = false;
bool last_left1_State = false, last_left2_State = false;
int left1_Count = 0, left2_Count = 0;

// Debug number
byte debugNum =0;
bool ecompassStatus = 1;
bool gpsStatus = 1;

// ============================================================================
// SETUP FUNCTION
// ============================================================================

void setup() {
   // Initialize Motors
  analogWrite(RFPWM, 0);
  analogWrite(LFPWM, 0);
  analogWrite(RBPWM, 0);
  analogWrite(LBPWM, 0);
  delay(1000);
  Serial.begin(115200);
  esp_log_level_set("i2c.master", ESP_LOG_NONE);
  esp_log_level_set("i2c", ESP_LOG_NONE);
  // Initialize GPS
  gpsSerial1.begin(9600);
  gpsSerial2.begin(9600,SERIAL_8N1,33,32);


  // Initialize E-compass
  mag.begin();
  if(!mag.isConnected()){
    ecompassStatus = 0;
  }
  // Initialize DF Player
  DFPlayerSerial.begin(9600, SWSERIAL_8N1, DFPLAYER_RX, DFPLAYER_TX, false, 256);
  if (!myDFPlayer.begin(DFPlayerSerial, true, true)) {
    Serial.println(F("Unable to begin DFPlayer:"));
    Serial.println(F("1. Please recheck the connection!"));
    Serial.println(F("2. Please insert the SD card!"));
  } else {
    Serial.println("DFPlayer initialized with ESPSoftwareSerial");
    myDFPlayer.volume(20);
  }

  // Initialize SD Card
#ifdef REASSIGN_PINS
  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, CS_PIN);
  if (!SD.begin(CS_PIN)) {
#else
  if (!SD.begin()) {
#endif
    Serial.println("Card Mount Failed");
    return;
  }

  // Initialize Bluetooth
  SerialBT.begin(device_name);
  Serial.printf("The device with name \"%s\" is started.\nNow you can pair it with Bluetooth!\n", device_name.c_str());

  // Initialize PCF8574
  pcf1.pinMode(P0, OUTPUT);
  pcf1.pinMode(P1, OUTPUT);
  pcf1.pinMode(P2, OUTPUT);
  pcf1.pinMode(P3, OUTPUT);

  pcf2.pinMode(P0, INPUT);
  pcf2.pinMode(P1, INPUT);
  pcf2.pinMode(P2, INPUT);
  pcf2.pinMode(P3, INPUT);
  pcf2.pinMode(P4, INPUT);
  pcf2.pinMode(P5, INPUT);
  pcf2.pinMode(P6, INPUT);
  pcf2.pinMode(P7, INPUT);

  if (pcf1.begin()) {
    Serial.println("OK1");
  } else {
    Serial.println("KO1");
  }

  if (pcf2.begin()) {
    Serial.println("OK2");
  } else {
    Serial.println("KO2");
  }
  //Initialize RGB
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection( TypicalLEDStrip );
  FastLED.setBrightness(  BRIGHTNESS );
  varibleUpdate();
  lastBtDataTime = millis();
}

// ============================================================================
// MOTOR CONTROL FUNCTIONS
// ============================================================================

int applyDeadzone(int val, int zone) {
  if (abs(val) < zone) return 0;
  return val;
}

int smoothUpdate(int current, int target, int step) {
  if (current < target) {
    current += step;
    if (current > target) current = target;
  } else if (current > target) {
    current -= step;
    if (current < target) current = target;
  }
  return current;
}

void setMotor(int pwmFwdChannel, int pwmBwdChannel, int speed) {
  if (speed > 0) {
    analogWrite(pwmFwdChannel, speed);
    analogWrite(pwmBwdChannel, 0);
  } else if (speed < 0) {
    analogWrite(pwmFwdChannel, 0);
    analogWrite(pwmBwdChannel, -speed);
  } else {
    analogWrite(pwmFwdChannel, 0);
    analogWrite(pwmBwdChannel, 0);
  }
}

// ============================================================================
// BLUETOOTH COMMUNICATION
// ============================================================================

void btMessageClassification() {
  switch (buffer[0]) {
    case 'x':
      {
        navigatingToWaypoint = false;
        AutoDriving = false;
        int endingLetter = buffer.indexOf('E');

        if (endingLetter != -1) {
          String packet = buffer.substring(0, endingLetter);
          int xIndex = packet.indexOf('x');
          int yIndex = packet.indexOf('y');

          if (xIndex != -1 && yIndex != -1) {
            String x_str = packet.substring(xIndex + 1, yIndex);
            String y_str = packet.substring(yIndex + 1);

            int x_val_signed = x_str.toInt();
            int y_val_signed = y_str.toInt();

            int x_signed = applyDeadzone(x_val_signed, DEADZONE);
            int y_signed = applyDeadzone(y_val_signed, DEADZONE);

            targetLeft = constrain(y_signed + x_signed, -255, 255);
            targetRight = constrain(y_signed - x_signed, -255, 255);
          }
          buffer = buffer.substring(endingLetter + 1);
        }
        break;
      }

    case 'S':
      {
        int endingLetter = buffer.indexOf('E');
        if (endingLetter != -1) {
          String packet = buffer.substring(1, endingLetter);
          switch (packet.toInt()) {
            case 1: GS12_1 = !GS12_1; break;
            case 2: GS12_2 = !GS12_2; break;
            case 3: GS5_1 = !GS5_1; break;
            case 4: GS5_2 = !GS5_2; break;
            case 5:
              {
                  position_count++;
                  String coords = String(currentLocation.latitude, 6) + "|" + String(currentLocation.longitude, 6);
                  String filename = "/Map1/P" + String(position_count) + ".txt";
                  writeFile(SD, filename.c_str(), coords.c_str());
                  Serial.println(coords);
                break;
              }
            case 6:
              {
                deleteAllFilesInDir(SD, "/Map1");
                position_count = 0;
                break;
              }
            case 7: 
            {
              if(!requirePosition){
                requirePosition = 1;
              }
              break;
            }
            case 8: stopAutonomousNavigation(); break;
            case 9:
              {
                if (AutoDriving) advanceToNextWaypoint();
                break;
              }
          }
          buffer = buffer.substring(endingLetter + 1);
        }
        break;
      }

    case 'P':
      {
        int endingIndex = buffer.indexOf('E');
        if (endingIndex != -1) {
          String packet = buffer.substring(0, endingIndex + 1);
          Serial.println(packet);
          extractPositions(packet, 10);
          buffer = buffer.substring(endingIndex + 1);
        }
        break;
      }
  }
}

void checkBluetoothTimeout() {
  unsigned long currentTime = millis();
  if (btConnected && (currentTime - lastBtDataTime > BT_TIMEOUT)) {
    btConnected = false;
  }
}

// ============================================================================
// RELAY CONTROL
// ============================================================================

void varibleUpdate() {
  pcf1.digitalWrite(P0, GS12_1 ? HIGH : LOW);
  pcf1.digitalWrite(P1, GS12_2 ? HIGH : LOW);
  pcf1.digitalWrite(P2, GS5_1 ? HIGH : LOW);
  pcf1.digitalWrite(P3, GS5_2 ? HIGH : LOW);
}

// ============================================================================
// SD CARD FUNCTIONS
// ============================================================================

void debugReadFile(fs::FS &fs, const char *path) {
  Serial.printf("Reading file: %s\n", path);
  File file = fs.open(path);
  if (!file) {
    Serial.println("Failed to open file for reading");
    return;
  }
  Serial.print("Read from file: ");
  while (file.available()) {
    Serial.write(file.read());
  }
  file.close();
}

void writeFile(fs::FS &fs, const char *path, const char *message) {
  Serial.printf("Writing file: %s\n", path);
  File file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  if (file.print(message)) {
    Serial.println("File written");
  } else {
    Serial.println("Write failed");
  }
  file.close();
}

void deleteFile(fs::FS &fs, const char *path) {
  Serial.printf("Deleting file: %s\n", path);
  if (fs.remove(path)) {
    Serial.println("File deleted");
  } else {
    Serial.println("Delete failed");
  }
}

void deleteAllFilesInDir(fs::FS &fs, const char *dirname) {
  Serial.printf("Deleting all files in directory: %s\n", dirname);
  File root = fs.open(dirname);
  if (!root) {
    Serial.println("Failed to open directory");
    return;
  }
  if (!root.isDirectory()) {
    Serial.println("Not a directory");
    return;
  }

  File file = root.openNextFile();
  while (file) {
    if (file.isDirectory()) {
      Serial.print("Skipping directory: ");
      Serial.println(file.name());
    } else {
      if (!fs.remove(file.path())) {
        // Delete failed
      }
    }
    file = root.openNextFile();
  }
  Serial.println("All files deleted from directory");
}

void extractPositions(const String &dataString, int maxPositions) {
  if (dataString.length() < 3 || dataString.charAt(0) != 'P' || dataString.charAt(dataString.length() - 1) != 'E') {
    Serial.println(dataString);
    Serial.println("Invalid format: Missing P or E");
  }

  String cleanString = dataString.substring(2, dataString.length() - 2);
  int segmentCount = 0;
  String segments[20];

  int startIndex = 0;
  int endIndex = cleanString.indexOf('|');

  while (endIndex != -1 && segmentCount < 20) {
    segments[segmentCount++] = cleanString.substring(startIndex, endIndex);
    startIndex = endIndex + 1;
    endIndex = cleanString.indexOf('|', startIndex);
  }

  if (startIndex < cleanString.length()) {
    segments[segmentCount++] = cleanString.substring(startIndex);
  }

  if (segmentCount < 5) {
    Serial.println("Invalid format: Not enough segments");
  }

  String baseLatitude = segments[0];
  String baseLongitude = segments[1];
  int totalChunks = segments[2].toInt();
  int currentChunk = segments[3].toInt();

  if (currentChunk == 0 && !requirePosition) {
    deleteAllFilesInDir(SD, "/Map1");
  }

  int positionCount = 0;
  for (int i = 4; i < segmentCount && positionCount < maxPositions; i++) {
    String positionPair = segments[i];
    int commaIndex = positionPair.indexOf(',');
    if (commaIndex == -1) {
      Serial.println("Invalid position pair format");
      continue;
    }

    String latOffsetStr = positionPair.substring(0, commaIndex);
    String longOffsetStr = positionPair.substring(commaIndex + 1);

    position_count++;
    // Normal set way point
    if(!requirePosition){
      Serial.println(requirePosition);
      String filename = "/Map1/P" + String(position_count) + ".txt";
      String coordinates = baseLatitude + latOffsetStr + "|" + baseLongitude + longOffsetStr;
      writeFile(SD, filename.c_str(), coordinates.c_str());
      positionCount++;
    }
    //Input Start position before auto driving
    else if(requirePosition && segmentCount ==5){
      startLocation.latitude = (baseLatitude + latOffsetStr).toDouble();
      startLocation.longitude= (baseLongitude + longOffsetStr).toDouble();
      requirePosition = 0;
      startAutonomousNavigation();
      headingBearing = calculateBearing();
    }
  }

  if (currentChunk == totalChunks - 1) {
    position_count = 0;
  }
}

// ============================================================================
// E-COMPASS FUNCTIONS
// ============================================================================

void updateHeading() {
  float x;
  float y;
  x = (event.magnetic.x - offsetX) * scaleX;
  y = (event.magnetic.y - offsetY) * scaleY;
  float headingMeasure = atan2(y, x)+0.175;
  
  
  // Correct for when signs are reversed.
  if(headingMeasure  < 0)
    headingMeasure  += 2*PI;
    
  // Check for wrap due to addition of declination.
  if(headingMeasure  > 2*PI)
    headingMeasure  -= 2*PI;
   
  // Convert radians to degrees for readability.
  heading = headingMeasure  * 180/M_PI; 
}

// ============================================================================
// GPS & NAVIGATION FUNCTIONS
// ============================================================================

bool readWaypointFromSD(int waypointNumber, Position &target) {
  String filename = "/Map1/P" + String(waypointNumber) + ".txt";

  if (!SD.exists(filename)) {
    Serial.println("Waypoint file not found: " + filename);
    return false;
  }

  File file = SD.open(filename);
  if (!file) {
    Serial.println("Failed to open waypoint file: " + filename);
    return false;
  }

  String data = file.readString();
  file.close();

  int separatorIndex = data.indexOf('|');
  if (separatorIndex == -1) {
    Serial.println("Invalid waypoint format in file: " + filename);
    return false;
  }

  String latStr = data.substring(0, separatorIndex);
  String lonStr = data.substring(separatorIndex + 1);

  target.latitude = latStr.toDouble();
  target.longitude = lonStr.toDouble();

  Serial.println("Loaded waypoint " + String(waypointNumber) + ": Lat=" + String(target.latitude, 6) + ", Lon=" + String(target.longitude, 6));
  return true;
}

// This is the literal fix you asked for, but it's hard to read and inefficient.
float calculateBearing() {
  // Assumes 'PI' is defined (e.g., const double PI = 3.141592653589793;)
  
  return (float)fmod( (atan2( 
    /* This is 'y' */
    sin((currentTarget.longitude * PI / 180.0) - (startLocation.longitude * PI / 180.0)) * cos(currentTarget.latitude * PI / 180.0), 
    
    /* This is 'x' */
    cos(startLocation.latitude * PI / 180.0) * sin(currentTarget.latitude * PI / 180.0) - 
    sin(startLocation.latitude * PI / 180.0) * cos(currentTarget.latitude * PI / 180.0) * cos((currentTarget.longitude * PI / 180.0) - (startLocation.longitude * PI / 180.0))
  
  ) * 180.0 / PI) + 360.0, 360.0 );
}
float calculateDistance(const Position &start, const Position &end) {
  const float R = 6371000.0;
  float lat1 = start.latitude * PI / 180.0;
  float lat2 = end.latitude * PI / 180.0;
  float dLat = (end.latitude - start.latitude) * PI / 180.0;
  float dLon = (end.longitude - start.longitude) * PI / 180.0;

  float a = sin(dLat / 2) * sin(dLat / 2) + cos(lat1) * cos(lat2) * sin(dLon / 2) * sin(dLon / 2);
  float c = 2 * atan2(sqrt(a), sqrt(1 - a));

  return R * c;
}

bool setNextWaypoint() {
  if (readWaypointFromSD(currentWaypoint, currentTarget)) {
    Serial.println("Navigating to waypoint " + String(currentWaypoint));
    navigatingToWaypoint = true;
    return true;
  } else {
    Serial.println("No more waypoints or error reading waypoint " + String(currentWaypoint));
    navigatingToWaypoint = false;
    AutoDriving = false;
    return false;
  }
}

void navigateToWaypoint() {
  if (!navigatingToWaypoint) return;

  float headingError = headingBearing - heading;
  if (headingError > 180) headingError -= 360;
  if (headingError < -180) headingError += 360;

  float distance = calculateDistance(currentLocation, currentTarget);

  Serial.print("Current: ");
  Serial.print(startLocation.latitude, 6);
  Serial.print(", ");
  Serial.print(startLocation.longitude, 6);
  Serial.print(" | Target: ");
  Serial.print(currentTarget.latitude, 6);
  Serial.print(", ");
  Serial.print(currentTarget.longitude, 6);
  Serial.print(" | Bearing: ");
  Serial.print(headingBearing, 1);
  Serial.print("° | Heading: ");
  Serial.print(heading, 1);
  Serial.print("° | Error: ");
  Serial.print(headingError, 1);
  Serial.print("° | Distance: ");
  Serial.print(distance, 1);
  Serial.println("m");

//Distance
  if (distance < 5) {
    Serial.println("Waypoint " + String(currentWaypoint) + " reached!");
    startLocation = currentLocation;
    if (!advanceToNextWaypoint()) {
      Serial.println("Navigation complete! All waypoints reached.");
      stopAutonomousNavigation();
    }
    headingBearing = calculateBearing();
    return;
  }

  int baseSpeed = calculateAdaptiveSpeed(distance);

// Heading Error
  if (abs(headingError) < 3) {
    setMotor(LFPWM, LBPWM, -baseSpeed);
    setMotor(RFPWM, RBPWM, -baseSpeed);
    Serial.println("going straight");
  } 
  else if(headingError >0){
    setMotor(LFPWM, LBPWM, 80);
    setMotor(RFPWM, RBPWM, -80);
    Serial.println("Turning right");
  }
  else {
    setMotor(LFPWM, LBPWM, -80);
    setMotor(RFPWM, RBPWM, 80);
    Serial.println("Turning left");
  }
}

bool advanceToNextWaypoint() {
  currentWaypoint++;
  if (readWaypointFromSD(currentWaypoint, currentTarget)) {
    Serial.println("Advanced to waypoint " + String(currentWaypoint));
    Serial.print("Next target: ");
    Serial.print(currentTarget.latitude, 6);
    Serial.print(", ");
    Serial.println(currentTarget.longitude, 6);
    return true;
  } else {
    Serial.println("Final waypoint reached - navigation complete!");
    return false;
  }
}

void startAutonomousNavigation() {
  currentWaypoint = 1;
  if (setNextWaypoint()) {
    AutoDriving = true;
    Serial.println("Autonomous navigation started!");
  } else {
    Serial.println("Failed to start autonomous navigation - no waypoints found!");
  }
}

void stopAutonomousNavigation() {
  AutoDriving = false;
  navigatingToWaypoint = false;
  setMotor(LFPWM, LBPWM, 0);
  setMotor(RFPWM, RBPWM, 0);
  Serial.println("Autonomous navigation stopped!");
}

bool getCurrentGPSPosition(Position &position) {
  if (gps1.location.isValid()  && gps1.location.age() < 2000 && gps2.location.isValid()  && gps2.location.age() < 2000) { 
    if(!gps1.location.isUpdated() || !gps2.location.isUpdated()){
      gpsStayStill = 1;
    }
    else gpsStayStill = 0;
    position.latitude = (gps1.location.lat() + gps2.location.lat()) / 2.0;
    position.longitude = (gps1.location.lng() + gps2.location.lng()) / 2.0;
    return true;
  } 
  else if (gps1.location.isValid()  && gps1.location.age() < 2000) {
    if(!gps1.location.isUpdated()){
      gpsStayStill = 1;
    }
    else gpsStayStill = 0;
    position.latitude = gps1.location.lat() ;
    position.longitude = gps1.location.lng() ;
    return true;
  }
  else if (gps2.location.isValid() && gps2.location.age() < 2000) {
    if(!gps2.location.isUpdated() ){
      gpsStayStill = 1;
    }
    else gpsStayStill = 0;
    position.latitude = gps2.location.lat() ;
    position.longitude = gps2.location.lng() ;
    return true;
  }
  else{
    gpsStatus = 0;
    Serial.println("No valid GPS data from either module");
    return false;
  }
}

int calculateAdaptiveSpeed(float distance) {
  if (distance < 5.0) return 80;
  if (distance < 15.0) return 100;
  return 150;
}

// void printGPSStatus() {
//   static unsigned long lastStatus = 0;
//   if (millis() - lastStatus < 5000) return;
  
//   Serial.println("\n=== GPS STATUS ===");
//   Serial.print("GPS1 - Valid: "); Serial.print(gps1.location.isValid());
//   Serial.print(" | Age: "); Serial.print(gps1.location.age());
//   Serial.print("ms | Sats: "); 
//   if (gps1.satellites.isValid()) Serial.println(gps1.satellites.value());
//   else Serial.println("Unknown");
  
//   Serial.print("GPS2 - Valid: "); Serial.print(gps2.location.isValid());
//   Serial.print(" | Age: "); Serial.print(gps2.location.age());
//   Serial.print("ms | Sats: "); 
//   if (gps2.satellites.isValid()) Serial.println(gps2.satellites.value());
//   else Serial.println("Unknown");
  
//   if (gps1.location.isValid()) {
//     Serial.print("GPS1 Position: ");
//     Serial.print(gps1.location.lat(), 6);
//     Serial.print(", ");
//     Serial.println(gps1.location.lng(), 6);
//   }
  
//   if (gps2.location.isValid()) {
//     Serial.print("GPS2 Position: ");
//     Serial.print(gps2.location.lat(), 6);
//     Serial.print(", ");
//     Serial.println(gps2.location.lng(), 6);
//   }
//   Serial.println("==================");
  
//   lastStatus = millis();
// }
// ============================================================================
// ENCODER FUNCTIONS
// ============================================================================


void readEncoders() {

  currentLeftState = !pcf2.digitalRead(P2);
  currentRightState = !pcf2.digitalRead(P0);

  unsigned long currentTime = millis();

  if (currentLeftState && !lastLeftState && (currentTime - lastLeftTime > 20)) {
    leftEncoderCount++;
    lastLeftTime = currentTime;
  }

  if (currentRightState && !lastRightState && (currentTime - lastRightTime > 20)) {
    rightEncoderCount++;
    lastRightTime = currentTime;
  }

  lastLeftState = currentLeftState;
  lastRightState = currentRightState;
}

void updateOdometryAndEcompass() {

  unsigned long currentTime = millis();

  if(currentTime - lastEncoderTime > 20){
    debugRGB();
    if(mag.isConnected()){
      ecompassStatus = 1;
      mag.getEvent(&event);
    }
    else ecompassStatus = 0;
    updateHeading();
    readEncoders();
    unsigned long leftDelta = leftEncoderCount - lastLeftCount;
    unsigned long rightDelta = rightEncoderCount - lastRightCount;

    float leftDist = leftDelta * METERS_PER_TICK;
    float rightDist = rightDelta * METERS_PER_TICK;

    leftDistance += leftDist;
    rightDistance += rightDist;
    totalDistance += (leftDist + rightDist) / 2.0;

    lastLeftCount = leftEncoderCount;
    lastRightCount = rightEncoderCount;
    lastEncoderTime = currentTime;
  }
}

void resetOdometry() {
  leftEncoderCount = 0;
  rightEncoderCount = 0;
  lastLeftCount = 0;
  lastRightCount = 0;
  leftDistance = 0.0;
  rightDistance = 0.0;
  totalDistance = 0.0;
}

void navigateToWaypointWithEncoders() {
  if (!navigatingToWaypoint) return;

  float headingError = headingBearing - heading;
  if (headingError > 180) headingError -= 360;
  if (headingError < -180) headingError += 360;

  float distance = calculateDistance(currentLocation, currentTarget);

  Serial.print("Current: ");
  Serial.print(startLocation.latitude, 6);
  Serial.print(", ");
  Serial.print(startLocation.longitude, 6);
  Serial.print(" | Target: ");
  Serial.print(currentTarget.latitude, 6);
  Serial.print(", ");
  Serial.print(currentTarget.longitude, 6);
  Serial.print(" | Bearing: ");
  Serial.print(headingBearing, 1);
  Serial.print("° | Heading: ");
  Serial.print(heading, 1);
  Serial.print("° | Error: ");
  Serial.print(headingError, 1);
  Serial.print("° | Distance: ");
  Serial.print(distance, 1);
  Serial.print("m | TotalDistance");
  Serial.print(totalDistance, 1);
  Serial.println("m");
  

//Condition to reach point
  if (distance < 5 && !gpsReach) {
    Serial.println("hee");
    gpsReach = 1 ;
    resetOdometry();
  }
  else if(gpsReach && totalDistance > 5){
    Serial.println("kuay");
    gpsReach =0;
    Serial.println("Waypoint " + String(currentWaypoint) + " reached!");
    startLocation = currentTarget ;
    if (advanceToNextWaypoint()) {
      headingBearing = calculateBearing();
    }
    else{
      Serial.println("Navigation complete! All waypoints reached.");
      stopAutonomousNavigation();
    }
    return;
  }

  int baseSpeed = calculateAdaptiveSpeed(distance);

// Heading Error
  if (abs(headingError) < 3) {
    setMotor(LFPWM, LBPWM, -baseSpeed+10);
    setMotor(RFPWM, RBPWM, -baseSpeed);
    turningRN = 0;
    Serial.println("going straight");
  } 
  else if(headingError >0){
    setMotor(LFPWM, LBPWM, 80);
    setMotor(RFPWM, RBPWM, -80);
    turningRN = 1;
    Serial.println("Turning right");
  }
  else {
    setMotor(LFPWM, LBPWM, -80);
    setMotor(RFPWM, RBPWM, 80);
    turningRN = 1;
    Serial.println("Turning left");
  }
}

// ============================================================================
// RGB FUNCTIONS
// ============================================================================

void setAllLEDs(CRGB color) {
    fill_solid(leds, NUM_LEDS, color);
    FastLED.show();
}

void debugRGB(){
  hardwareConnectiontoDebugNum();
  switch(debugNum) {
  case 0:
    // Manual without bug
    setAllLEDs(CRGB::Honeydew);
    break;
  case 1:
    // Recevie Instruction path from Mobile app
    setAllLEDs(CRGB::Red);
    break;

  case 2:
    // Recevie Instruction path from Mobile app
    setAllLEDs(CRGB::Orchid);
    break;

  case 3:
    // Recevie Instruction path from Mobile app
    setAllLEDs(CRGB::Magenta);
    break;
  case 4:
    // Recevie Instruction path from Mobile app
    setAllLEDs(CRGB::Green);
    break;
  case 5:
    // Recevie Instruction path from Mobile app
    setAllLEDs(CRGB::Purple);
    break;
  default:
    // code block
    break;
  }
}

void hardwareConnectiontoDebugNum(){
  if(!ecompassStatus && !gpsStatus){
    debugNum = 1;
  }
  else if(!ecompassStatus ){
    debugNum = 2;
  }
  else if(!gpsStatus ){
    debugNum = 3;
  }
  else if(gpsStayStill){
    debugNum = 4;
  }
  else debugNum = 0;

  if(requirePosition){
    debugNum = 5;
  }
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
  updateOdometryAndEcompass();
    // Improved GPS processing
    // Process all available data
  while (gpsSerial1.available() > 0) {
    if(gps1.encode(gpsSerial1.read()))
      gps1OK = 1;
  }
    while (gpsSerial2.available() > 0) {
      if(gps2.encode(gpsSerial2.read()));
        gps2OK = 1;
  }

  if(gps1OK || gps2OK){
    if(getCurrentGPSPosition(currentLocation)){
      gpsStatus = 1;
    }
    else gpsStatus =0;
    // Print GPS status for debugging
    gps1OK =0;
    gps2OK =0;
  }

  // Bluetooth communication
  if (SerialBT.available()) {
    char c = SerialBT.read();
    buffer += c;
    lastBtDataTime = millis();
    btConnected = true;
  }

  if (btConnected) {
    varibleUpdate();
    btMessageClassification();
    checkBluetoothTimeout();
  }

  // Autonomous navigation
  if (AutoDriving && navigatingToWaypoint) {
    if (millis() - delayGPS > 20) {
      delayGPS = millis();
      navigateToWaypointWithEncoders();
      // navigateToWaypoint();
    }
  } else {
    // Manual control
    currentLeft = smoothUpdate(currentLeft, targetLeft, RAMP_STEP);
    currentRight = smoothUpdate(currentRight, targetRight, RAMP_STEP);
    setMotor(LFPWM, LBPWM, currentLeft);
    setMotor(RFPWM, RBPWM, currentRight);
  }
}
