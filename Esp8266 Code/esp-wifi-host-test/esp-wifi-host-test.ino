#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

WiFiUDP Udp;
unsigned int localUdpPort = 4210;  // local port to listen on
char incomingPacket[255];  // buffer for incoming packets

IPAddress local_IP(192,168,0,1);
IPAddress gateway(192,168,0,1);
IPAddress subnet(255,255,255,0);
bool ledstatus = true;

void setup()
{
  pinMode(2, OUTPUT);
  digitalWrite(2, ledstatus);
  Serial.begin(115200);
  Serial.println();

  Serial.print("Setting soft-AP configuration ... ");
  Serial.println(WiFi.softAPConfig(local_IP, gateway, subnet) ? "Ready" : "Failed!");

  Serial.print("Setting soft-AP ... ");
  Serial.println(WiFi.softAP("UDP-listener") ? "Ready" : "Failed!");
  
  Udp.begin(localUdpPort);
  Serial.printf("Now listening at IP %s, UDP port %d\n", WiFi.softAPIP().toString().c_str(), localUdpPort);
}


void loop()
{
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
    
    ledstatus = !ledstatus;
    digitalWrite(2, ledstatus);
    // send back a reply, to the IP address and port we got the packet from
    Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
    Udp.write(incomingPacket);
    Udp.endPacket();
  }
}
