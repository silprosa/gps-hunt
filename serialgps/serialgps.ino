#include <TinyGPS++.h>
#include <SoftwareSerial.h>

static const int RXPin = 4, TXPin = 3;
static const uint32_t GPSBaud = 9600;
int buzzerPin = 7; 
static const double targetLat = -0.3968649;  // Target latitude
static const double targetLng = 36.963148; // Target longitude
static const double thresholdDistance = 10.0; // Range in meters for buzzer activation

TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);

void setup() {
  Serial.begin(9600);
  ss.begin(GPSBaud);
  pinMode(buzzerPin, OUTPUT);
  Serial.println("GPS System Init...");
}

void loop() {
  while (ss.available() > 0) {
    gps.encode(ss.read());
    
    if (gps.location.isUpdated()) {
      double currentLat = gps.location.lat();
      double currentLng = gps.location.lng();

      // Calculate distance to target location
      double distanceToTarget = TinyGPSPlus::distanceBetween(
        currentLat, currentLng, targetLat, targetLng
      );

      // Print current location to Serial Monitor
      Serial.print("Latitude: ");
      Serial.print(currentLat, 6);
      Serial.print(" Longitude: ");
      Serial.print(currentLng, 6);

      // Print distance to Serial Monitor
      Serial.print(" Distance to Target: ");
      Serial.print(distanceToTarget);
      Serial.println(" meters");

      // Check if within range
      if (distanceToTarget <= thresholdDistance) {
        Serial.println("Within range! Buzzer ON.");
        digitalWrite(buzzerPin, HIGH); // Activate buzzer
      } else {
        Serial.println("Out of range. Buzzer OFF.");
        digitalWrite(buzzerPin, LOW); // Deactivate buzzer
      }
    }
  }
}
