#include <SoftwareSerial.h>

SoftwareSerial gps (4.3);

char data = '';

void  setup ()
{
 Serial . Begin (115200);            
 . gps begin (9600);
}


void  loop ()
{
  if (gps. available ())
  {
    = GPS data. read ();
     Serial . print (data);
  }
}
