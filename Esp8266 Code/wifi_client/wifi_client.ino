#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

const char* ssid = "robot network";
const char* password = "1234567890";

WiFiUDP Udp;
unsigned int udpPort = 4210;  // local port to listen on
char incomingPacket[255];  // buffer for incoming packets
char  sendPacket[] = "Hi there! Got the message :-)";  // a reply string to send back
IPAddress target(192,168,0,1);
void setup()
{
  Serial.begin(115200);
  Serial.println();

  Serial.printf("Connecting to %s ", ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" connected");

  Udp.begin(udpPort);
  Serial.printf("Now listening for reply at IP %s, UDP port %d\n", WiFi.localIP().toString().c_str(), udpPort);
}

void loop()
{
  // send back a reply, to the IP address and port we got the packet from
  String msg = Serial.readStringUntil('\n');
  Udp.beginPacket(target, udpPort);
  Udp.write(msg.c_str());
  Udp.endPacket();

  int packetSize = Udp.parsePacket();
  if (packetSize)
  {
    // receive incoming UDP packets
    Serial.printf("Received %d bytes from %s, port %d\n", packetSize, Udp.remoteIP().toString().c_str(), Udp.remotePort());
    int len = Udp.read(incomingPacket, 255);
    if (len > 0)
    {
      incomingPacket[len] = 0;
    }
    Serial.printf("UDP packet contents: %s\n", incomingPacket);

  }
}

