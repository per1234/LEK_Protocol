#include <LEK_Protocol.h>
#include <LEK_Protocol_Definitions.h>
#include <SPI.h>
#include <Keyboard.h>
#include <Mouse.h>

const char* mynamer = "hamreceiver";
//LEK_Protocol master(kRECEIVER_DEPLOYED, 0x01, 0x01, mynamer, true);

const char* mynameg = "hamserver";
LEK_Protocol master(kGATEWAY_CONSOLE, 0x01, 0x01, mynameg, true);

void setup() 
{
  SPI.begin();
  Serial.begin(LEK_DEFAULT_SERIAL_BAUD_RATE);
  Serial.setTimeout(LEK_DEFAULT_SERIAL_TIMEOUT);
  Keyboard.begin();
  Mouse.begin();

  pinMode(LEK_LINK_LED_PIN, OUTPUT);
  pinMode(LEK_HB_LED_PIN, OUTPUT);
  pinMode(LEK_USB_DEVICE_MUX_PIN, OUTPUT);
  pinMode(LEK_USB_DEVICE_PWR_PIN, OUTPUT);

  pinMode(LEK_RFM_RESET_PIN, INPUT);
  master.begin();
}
 
void loop()
{
  master.spin();
}
