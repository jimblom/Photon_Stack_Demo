#include "SFE_LSM9DS1.h"
#include "SparkFunMAX17043.h"
#include "SFE_MicroOLED.h"
#include "SparkFunPhant.h"
#include "math.h"

//////////////////////////
// MicroOLED Definition //
//////////////////////////
#define PIN_RESET D7  // Connect RST to pin 9 (req. for SPI and I2C)
#define PIN_DC    D3  // Connect DC to pin 8 (required for SPI)
#define PIN_CS    A2 // Connect CS to pin 10 (required for SPI)
MicroOLED oled(MODE_SPI, PIN_RESET, PIN_DC, PIN_CS);

///////////////////////
// LSM9DS1 I2C Setup //
///////////////////////
// Comment out this section if you're using SPI
// SDO_XM and SDO_G are both grounded, so our addresses are:
#define LSM9DS1_M    0x1E // Would be 0x1C if SDO_XM is LOW
#define LSM9DS1_AG   0x6B // Would be 0x6A if SDO_G is LOW
LSM9DS1 dof(DOF_MODE_I2C, LSM9DS1_AG, LSM9DS1_M);

// Earth's magnetic field varies by location. Add or subtract
// a declination to get a more accurate heading. Calculate
// your's here:
// http://www.ngdc.noaa.gov/geomag-web/#declination
#define DECLINATION -8.58 // Declination (degrees) in Boulder, CO.

////////////////////
// Cube Variables //
////////////////////
#define SHAPE_SIZE 600
#define ROTATION_SPEED 1 // ms delay between cube draws

////////////////////////
// Phant Declarations //
////////////////////////
const char server[] = "data.sparkfun.com"; // Phant destination server
const char publicKey[] = "bGgKlaX6XofAWznJEvKQ"; // Phant public key
const char privateKey[] = "Vp12oNlgljC7pKWw5A4X"; // Phant private key
Phant phant(server, publicKey, privateKey); // Create a Phant object

///////////////////////
// Phant Post Timing //
///////////////////////
const int POST_RATE = 30000; // Time between posts, in ms.
unsigned long lastPost = 0; // global variable to keep track of last post time

/////////////////////////////
// Battery Graph Variables //
/////////////////////////////
unsigned long graphSampleRate = 1000;
unsigned long lastGraphSample = 0;

uint8_t currentPercent = 0;
const uint8_t MAX_PERCENT_HISTORY = 62;
float percentHistory[MAX_PERCENT_HISTORY];

int SCREEN_WIDTH = oled.getLCDWidth();
int SCREEN_HEIGHT = oled.getLCDHeight();

float d = 3;
float px[] = { -d,  d,  d, -d, -d,  d,  d, -d };
float py[] = { -d, -d,  d,  d, -d, -d,  d,  d };
float pz[] = { -d, -d, -d, -d,  d,  d,  d,  d };

float p2x[] = {0,0,0,0,0,0,0,0};
float p2y[] = {0,0,0,0,0,0,0,0};

float r[] = {0,0,0};

#define BLUE_BUTTON D2
#define GREEN_BUTTON D4

#define MODE_COUNT 2
enum t_mode
{
  MODE_CUBE,
  MODE_BATTERY
};
int currentMode = MODE_CUBE;

SYSTEM_MODE(SEMI_AUTOMATIC);

void setup()
{
  pinMode(BLUE_BUTTON, INPUT_PULLUP);
  pinMode(GREEN_BUTTON, INPUT_PULLUP);
  attachInterrupt(GREEN_BUTTON, changeMode, FALLING);
  attachInterrupt(BLUE_BUTTON, changeConnect, FALLING);

  Serial.begin(115200);

  uint16_t status = dof.begin();
  Serial.print("LSM9DS1 WHO_AM_I's returned: 0x");
  Serial.println(status, HEX);
  Serial.println("Should be 0x683D");
  Serial.println();

  oled.begin();
  oled.clear(ALL);  // Clear the display's memory (gets rid of artifacts)
}

void loop()
{
  // Draw something on the OLED
  switch (currentMode)
  {
  case MODE_CUBE:
    drawCube();
    break;
  case MODE_BATTERY:
    drawBatteryGraph();
    break;
  }

  // Post to Phant:
  if (Spark.connected())
    phantTimer();
}

void changeMode()
{
  while( digitalRead(GREEN_BUTTON) == LOW)
    ;
  currentMode = ((int)currentMode + 1) % MODE_COUNT;
}

void changeConnect()
{
  while( digitalRead(BLUE_BUTTON) == LOW)
    ;
  if (!Spark.connected())
    Spark.connect();
  else
    Spark.disconnect();
}

void phantTimer()
{
  // If it's been POST_RATE ms (default 30 seconds), try to post again.
  if (lastPost + POST_RATE < millis())
  {
  // If the post succeeds, update lastPost so we don't post for
  // another 30 seconds.
    if (postToPhant() > 0)
    {
      lastPost = millis();
    }
  }
}

int postToPhant()
{
  // Use phant.add(<field>, <value>) to add data to each field.
  // Phant requires you to update each and every field before posting,
  // make sure all fields defined in the stream are added here.
  phant.add("voltage", gauge.getVoltage());
  phant.add("percentage", gauge.getSOC());

  TCPClient client;
  char response[512];
  int i = 0;
  int retVal = 0;

  if (client.connect(server, 80)) // Connect to the server
  {
    // Post message to indicate connect success
    Serial.println("Posting!");

    // phant.post() will return a string formatted as an HTTP POST.
    // It'll include all of the field/data values we added before.
    // Use client.print() to send that string to the server.
    client.print(phant.post());
    delay(1000);
    // Now we'll do some simple checking to see what (if any) response
    // the server gives us.
    while (client.available())
    {
      char c = client.read();
      Serial.print(c);	// Print the response for debugging help.
      if (i < 512)
        response[i++] = c; // Add character to response string
    }
    // Search the response string for "200 OK", if that's found the post
    // succeeded.
    if (strstr(response, "200 OK"))
    {
      Serial.println("Post success!");
      retVal = 1;
    }
    else if (strstr(response, "400 Bad Request"))
    {	// "400 Bad Request" means the Phant POST was formatted incorrectly.
      // This most commonly ocurrs because a field is either missing,
      // duplicated, or misspelled.
      Serial.println("Bad request");
      retVal = -1;
    }
    else
    {
      // Otherwise we got a response we weren't looking for.
      retVal = -2;
    }
  }
  else
  {	// If the connection failed, print a message:
    Serial.println("connection failed");
    retVal = -3;
  }

  client.stop();	// Close the connection to server.
  return retVal;	// Return error (or success) code.

}

void drawBatteryGraph()
{

  if (lastGraphSample + graphSampleRate < millis())
  {
    percentHistory[currentPercent++] = gauge.getSOC();
    if (currentPercent >= MAX_PERCENT_HISTORY)
    {
      for (int i = 0; i < (MAX_PERCENT_HISTORY / 2); i++)
      {
        //percentHistory[i] = percentHistory[i + MAX_PERCENT_HISTORY / 2];
        percentHistory[i] = percentHistory[i * 2];
        //percentHistory[i + MAX_PERCENT_HISTORY / 2] = 0;
      }
      for (int i = MAX_PERCENT_HISTORY / 2; i<MAX_PERCENT_HISTORY; i++)
        percentHistory[i] = 0;

      graphSampleRate += graphSampleRate;
      currentPercent = MAX_PERCENT_HISTORY / 2;
    }
    lastGraphSample = millis();
  }
  oled.clear(PAGE);
  oled.setFontType(0);
  oled.setCursor(0, 0);
  oled.print("V: ");
  oled.print(gauge.getVoltage(), 2);
  oled.println(" V");

  oled.setCursor(0, oled.getFontHeight() + 1);
  oled.print("%: ");
  oled.print(gauge.getSOC(), 2);
  oled.println(" %");

  uint8_t graphTopY = 2 * (oled.getFontHeight() + 1);
  uint8_t graphBottomY = oled.getLCDHeight() - 1;
  oled.line(1, graphTopY, 1, oled.getLCDHeight() - 1);
  oled.line(1, graphBottomY, oled.getLCDWidth(), graphBottomY);
  for (int i = 0; i < MAX_PERCENT_HISTORY; i++)
  {
    if (percentHistory[i] != 0)
      oled.pixel(i + 2, graphBottomY - ((graphBottomY - graphTopY) * (percentHistory[i] / 100.0)));
  }
  oled.setCursor(oled.getLCDWidth()/2, oled.getLCDHeight() - oled.getFontHeight() - 1);
  oled.print((long) graphSampleRate / 1000);

  oled.display();
}

void drawCube()
{
  dof.readAccel();
  dof.readMag();
  float ax = dof.calcAccel(dof.ax);
  float ay = dof.calcAccel(dof.ay);
  float az = dof.calcAccel(dof.az);
  float mx = dof.calcMag(dof.mx);
  float my = dof.calcMag(dof.my);
  float mz = dof.calcMag(dof.mz);
  float roll = calcRoll(ax, ay, az);
  float pitch = calcPitch(ax, ay, az);
  float heading = calcHeading(mx, my, mz, pitch, roll);

  Serial.print("Roll: "); Serial.println(180.0 / M_PI * roll);
  Serial.print("Pitch: "); Serial.println(180.0 / M_PI *pitch);
  Serial.print("Heading: "); Serial.println(180.0 / M_PI * heading);
  r[0] = -roll;
  r[1] = pitch;
  float headingTolerance = 0 * M_PI / 180.0;
  if ((pitch < headingTolerance) && (roll < headingTolerance) &&
      (pitch > -headingTolerance) && (roll > -headingTolerance))
    r[2] = -heading;
  else
    r[2] = 0;
  if (r[0] >= 360.0*M_PI/180.0) r[0] = 0;
	if (r[1] >= 360.0*M_PI/180.0) r[1] = 0;
	if (r[2] >= 360.0*M_PI/180.0) r[2] = 0;

	for (int i=0;i<8;i++)
	{
		float px2 = px[i];
		float py2 = cos(r[0])*py[i] - sin(r[0])*pz[i];
		float pz2 = sin(r[0])*py[i] + cos(r[0])*pz[i];

		float px3 = cos(r[1])*px2 + sin(r[1])*pz2;
		float py3 = py2;
		float pz3 = -sin(r[1])*px2 + cos(r[1])*pz2;

		float ax = cos(r[2])*px3 - sin(r[2])*py3;
		float ay = sin(r[2])*px3 + cos(r[2])*py3;
		float az = pz3-150;

		p2x[i] = SCREEN_WIDTH/2+ax*SHAPE_SIZE/az;
		p2y[i] = SCREEN_HEIGHT/2+ay*SHAPE_SIZE/az;
	}

	oled.clear(PAGE);
	for (int i=0;i<3;i++)
	{
		oled.line(p2x[i],p2y[i],p2x[i+1],p2y[i+1]);
		oled.line(p2x[i+4],p2y[i+4],p2x[i+5],p2y[i+5]);
		oled.line(p2x[i],p2y[i],p2x[i+4],p2y[i+4]);
	}
	oled.line(p2x[3],p2y[3],p2x[0],p2y[0]);
	oled.line(p2x[7],p2y[7],p2x[4],p2y[4]);
	oled.line(p2x[3],p2y[3],p2x[7],p2y[7]);
	oled.display();
}

float calcPitch(float x, float y, float z)
{
  return atan2(x, sqrt(y * y) + (z * z));
}

float calcRoll(float x, float y, float z)
{
  return atan2(y, sqrt(x * x) + (z * z));
}

float calcHeading(float hx, float hy, float hz, float roll, float pitch)
{

  float heading;

  if (hy == 0)
    heading = (hx < 0) ? M_PI : 0;
  else
    heading = atan2(hx, hy);

  heading -= DECLINATION * M_PI / 180.0;

  return heading;
}
