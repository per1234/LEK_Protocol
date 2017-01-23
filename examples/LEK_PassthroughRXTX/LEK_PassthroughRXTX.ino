#include <LEK_Protocol.h>
#include <LEK_Protocol_Definitions.h>

const char* myname = "hamcircle";

void setup() 
{
  LEK_Protocol master(kGATEWAY_SLAVE, 0x01, 0x01, myname, true);
}
 
void loop()
{

}
