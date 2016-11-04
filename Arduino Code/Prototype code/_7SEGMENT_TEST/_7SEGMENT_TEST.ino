#include <SPI.h>
#include <HCMAX7219.h>

HCMAX7219 HCMAX7219(10); // CS PIN

void setup() {
  // put your setup code here, to run once:
  HCMAX7219.Clear();
  HCMAX7219.print7Seg(1212, 4);
  HCMAX7219.Refresh();
}

void loop() {
  // put your main code here, to run repeatedly:

}
