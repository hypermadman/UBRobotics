#include <ESP8266WiFi.h>

void setup()

{
  int x;
  Serial.begin(115200);
  Serial.println();
  pinMode(2, OUTPUT);
  Serial.print("Setting soft-AP ... ");
  boolean result = WiFi.softAP("urmom", "1234567890");
  if(result == true)
  {
    Serial.println("Ready");
  }
  else
  {
    Serial.println("Failed!");
  }
  Serial.println(WiFi.localIP().toString());
}

void loop()
{
  Serial.printf("Stations connected = %d\n", WiFi.softAPgetStationNum());
  if(WiFi.softAPgetStationNum()>=1) {
      digitalWrite(2, HIGH);
  }
  else 
  { digitalWrite(2, LOW); };
  delay(3000);
  
}


