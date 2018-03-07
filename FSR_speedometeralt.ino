/*
  Połączenia (schemat):
  GPS TX -> Arduino 3
  GPS RX -> Arduino 4
  ssd1306  SDA -> Arduino A4
  ssd1306 SCL -> Arduino A5
*/

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SoftwareSerial.h>

#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);

#if (SSD1306_LCDHEIGHT != 64)
#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif

// Connect the GPS RX/TX to arduino pins 3 and 4
SoftwareSerial serial = SoftwareSerial(3, 4);

const unsigned char UBLOX_INIT[] PROGMEM = {
  // Disable NMEA
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x24, // GxGGA off
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x2B, // GxGLL off
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x32, // GxGSA off
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x39, // GxGSV off
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x04, 0x40, // GxRMC off
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x05, 0x47, // GxVTG off

  // Disable UBX
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x17, 0xDC, //NAV-PVT off
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x12, 0xB9, //NAV-POSLLH off
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x13, 0xC0, //NAV-STATUS off

  // Enable UBX
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x07, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x18, 0xE1, //NAV-PVT on
  //0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x02,0x00,0x01,0x00,0x00,0x00,0x00,0x13,0xBE, //NAV-POSLLH on
  //0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x03,0x00,0x01,0x00,0x00,0x00,0x00,0x14,0xC5, //NAV-STATUS on

  // Rate
  0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x64, 0x00, 0x01, 0x00, 0x01, 0x00, 0x7A, 0x12, //(10Hz)
  //0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A, //(5Hz)
  //0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xE8, 0x03, 0x01, 0x00, 0x01, 0x00, 0x01, 0x39 //(1Hz)
};

const unsigned char UBX_HEADER[] = { 0xB5, 0x62 };

struct NAV_PVT {
  unsigned char cls;
  unsigned char id;
  unsigned short len;
  unsigned long iTOW;          // GPS time of week of the navigation epoch (ms)

  unsigned short year;         // Year (UTC)
  unsigned char month;         // Month, range 1..12 (UTC)
  unsigned char day;           // Day of month, range 1..31 (UTC)
  unsigned char hour;          // Hour of day, range 0..23 (UTC)
  unsigned char minute;        // Minute of hour, range 0..59 (UTC)
  unsigned char second;        // Seconds of minute, range 0..60 (UTC)
  char valid;                  // Validity Flags (see graphic below)
  unsigned long tAcc;          // Time accuracy estimate (UTC) (ns)
  long nano;                   // Fraction of second, range -1e9 .. 1e9 (UTC) (ns)
  unsigned char fixType;       // GNSSfix Type, range 0..5
  char flags;                  // Fix Status Flags
  unsigned char reserved1;     // reserved
  unsigned char numSV;         // Number of satellites used in Nav Solution

  long lon;                    // Longitude (deg)
  long lat;                    // Latitude (deg)
  long height;                 // Height above Ellipsoid (mm)
  long hMSL;                   // Height above mean sea level (mm)
  unsigned long hAcc;          // Horizontal Accuracy Estimate (mm)
  unsigned long vAcc;          // Vertical Accuracy Estimate (mm)

  long velN;                   // NED north velocity (mm/s)
  long velE;                   // NED east velocity (mm/s)
  long velD;                   // NED down velocity (mm/s)
  long gSpeed;                 // Ground Speed (2-D) (mm/s)
  long heading;                // Heading of motion 2-D (deg)
  unsigned long sAcc;          // Speed Accuracy Estimate
  unsigned long headingAcc;    // Heading Accuracy Estimate
  unsigned short pDOP;         // Position dilution of precision
  short reserved2;             // Reserved
  unsigned long reserved3;     // Reserved
};
NAV_PVT pvt;
void displayln(const char* format, ...)
{
  char buffer[64];
  va_list args;
  va_start(args, format);
  vsprintf(buffer, format, args);
  va_end(args);
  int len = strlen(buffer);
  for (uint8_t i = 0; i < len; i++) {
    display.write(buffer[i]);
  }
}
void calcChecksum(unsigned char* CK) {
  memset(CK, 0, 2);
  for (int i = 0; i < (int)sizeof(NAV_PVT); i++) {
    CK[0] += ((unsigned char*)(&pvt))[i];
    CK[1] += CK[0];
  }
}
long numGPSMessagesReceived = 0;
bool processGPS() {
  static int fpos = 0;
  static unsigned char checksum[2];
  const int payloadSize = sizeof(NAV_PVT);
  while ( serial.available() ) {
    byte c = serial.read();
    if ( fpos < 2 ) {
      if ( c == UBX_HEADER[fpos] )
        fpos++;
      else
        fpos = 0;
    }
    else {
      if ( (fpos - 2) < payloadSize )
        ((unsigned char*)(&pvt))[fpos - 2] = c;
      fpos++;
      if ( fpos == (payloadSize + 2) ) {
        calcChecksum(checksum);
      }
      else if ( fpos == (payloadSize + 3) ) {
        if ( c != checksum[0] )
          fpos = 0;
      }
      else if ( fpos == (payloadSize + 4) ) {
        fpos = 0;
        if ( c == checksum[1] ) {
          return true;
        }
      }
      else if ( fpos > (payloadSize + 4) ) {
        fpos = 0;
      }
    }
  }
  return false;
}
void setup()
{
  Serial.begin(9600);
  serial.begin(9600);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // initialize with the I2C addr 0x3D (for the 128x64)
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.clearDisplay();
  // send configuration data in UBX protocol
  for (unsigned int i = 0; i < sizeof(UBLOX_INIT); i++) {
    serial.write( pgm_read_byte(UBLOX_INIT + i) );
    delay(5); // simulating a 38400baud pace (or less), otherwise commands are not accepted by the device.
  }
}

unsigned long lastScreenUpdate = 0;
long maxSpeed = 0;
int numSV = 0;
int fixType = 0;
int block = 0;
int dok = 0;
int dok2 = 0;
int hacc = 0;
int hacc2 = 0;
int alt = 0;
int alt2 = 0;
int vacc = 0;
int vacc2 = 0;


void loop() {
  if ( processGPS() ) {
    /*
      Serial.print("#SV: ");      Serial.print(pvt.numSV);
      Serial.print(" fixType: "); Serial.print(pvt.fixType);
      Serial.print(" Date:");     Serial.print(pvt.year); Serial.print("/"); Serial.print(pvt.month); Serial.print("/"); Serial.print(pvt.day); Serial.print(" "); Serial.print(pvt.hour); Serial.print(":"); Serial.print(pvt.minute); Serial.print(":"); Serial.print(pvt.second);
      Serial.print(" lat/lon: "); Serial.print(pvt.lat/10000000.0f); Serial.print(","); Serial.print(pvt.lon/10000000.0f);
      Serial.print(" gSpeed: ");  Serial.print(pvt.gSpeed/1000.0f);
      Serial.print(" heading: "); Serial.print(pvt.heading/100000.0f);
      Serial.print(" hAcc: ");    Serial.print(pvt.hAcc/1000.0f);
      Serial.println();
    */
    fixType = pvt.fixType;
    numSV = pvt.numSV;
    numGPSMessagesReceived++;
    dok = pvt.sAcc;
    hacc = pvt.hAcc / 1000.0;
    vacc = pvt.vAcc / 1000.0;

//=====================================================
 
  


//=====================================================
    if (pvt.gSpeed < 5555 || pvt.gSpeed > 50000)
    {
      pvt.gSpeed = 0;
    }
    if (block == 0)
      pvt.gSpeed = 0;
 
  
   
    if ( pvt.gSpeed > maxSpeed )
      maxSpeed = pvt.gSpeed;
  }
//=====================================================

  updateScreen();

}
void updateScreen()
{
  unsigned long now = millis();
  if ( now - lastScreenUpdate < 1000 ) {
    return;
  }
  char buf[8];
  char buf2[8];
  char buf3[8];
  char buf4[8];
  
  display.clearDisplay();
  display.setCursor(0, 0);
  //display.setTextSize(2);
  //displayln("GPS: %d   ", numGPSMessagesReceived);
  // ===================================================================

float hacc = pvt.hAcc / 1000.0;
  dtostrf(hacc, 2, 1, buf4);

  float vacc2 = pvt.vAcc / 1000.0;
  dtostrf(vacc2, 2, 1, buf3);

  float dok2 = dok * 0.0036;
  dtostrf(dok2, 2, 1, buf2);

  float kmh = maxSpeed * 0.0036;
  dtostrf(kmh, 3, 2, buf);
  // ===================================================================
  if    (numSV >= 8 || block == 1)
  {
    alt = pvt.hMSL / 1000;
  if (alt > alt2)
   alt2 = alt ;

   
    display.clearDisplay();
    display.setTextSize(1);
    // displayln(" Predkosc maksymalna\n");
    display.setTextSize(2);
    displayln("%3s", buf);

    display.setTextSize(1);
    displayln("km/h");
    displayln(" (%s)\n\n\n", buf2);
    display.setTextSize(2);
    displayln("%d.0", alt2);
    display.setTextSize(1);
    displayln("m npm");
    displayln(" (%s)\n\n\n", buf3);
    display.setTextSize(1);
    displayln(" dokH:%3s m ", buf4);
    displayln(" Sat:%2d\n", numSV);
    
    block = 1;
  }
  else
  {
    {
      display.clearDisplay();
      display.setTextSize(1);
      displayln("====================\n");
      display.setTextSize(2);
      displayln("  MxSpeed\n");
      display.setTextSize(1);
      displayln("====================\n");
      //displayln(".\n");
      displayln("Fix:%s", fixType > 2 ? "3D " : "Brak");
      displayln(" dokH:%3s m\n\n", buf4);
      //displayln("  Sat: %2d\n\n", numSV);
      display.setTextSize(2);
      displayln("CZEKAJ...%d", numSV);
      display.setTextSize(1);
      //displayln(" v:%s V", volt);
    }
  }
  display.display();
  numGPSMessagesReceived = 0;
  lastScreenUpdate = millis();
}

