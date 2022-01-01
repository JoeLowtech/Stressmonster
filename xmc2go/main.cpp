#include <Arduino.h>
#include <Dps310.h>
#include <PacketSerial.h>

/*
  Stressmonster Pressure measuring
  Collects pressure data and sends the data via Uart to the PC
  
  Additional library:
  PacketSerial by Christopher Baker
*/

/*
  Package Handler with SLIP Protocol
  Simple Package Encoder/Decoder, less overhead than asci
*/ 
SLIPPacketSerial MyPacketSerial;

// Pressure data in Pascal. For Hectopascal divide with 100
// Pressure data is 24 bit long -> 120000 Pa
// Dps310 Opject

Dps310 PressureSensor = Dps310();

// Timer variable
unsigned long previousMillis = 0;
const long intervalMillis = 500;

void onPacketReceived(const uint8_t* buffer, size_t size)
{
    // Process your decoded incoming packet here.
}

void setup()
{
  MyPacketSerial.begin(9600);
  MyPacketSerial.setPacketHandler(&onPacketReceived);
  while (!Serial);

/*********************************************************************
  Call begin to initialize Dps310PressureSensor
  The parameter 0x76 is the bus address. The default address is 0x77 and does not need to be given.
  Dps310PressureSensor.begin(Wire, 0x76);
  Use the commented line below instead to use the default I2C address. 
  ********************************************************************/
  PressureSensor.begin(Wire);


  int16_t temp_mr = 2, temp_osr = 2, prs_mr = 5, prs_osr = 2;
  int16_t ret = PressureSensor.startMeasureBothCont(temp_mr, temp_osr, prs_mr, prs_osr);

/**********************************************************************************************
  temp_mr = temperature measure rate (value from 0 to 7)
  2^temp_mr temperature measurement results per second

  temp_osr = temperature oversampling rate (value from 0 to 7)
  2^temp_osr internal temperature measurements per result
  A higher value increases precision

  prs_mr = pressure measure rate (value from 0 to 7)
  2^prs_mr pressure measurement results per second

  prs_osr = pressure oversampling rate (value from 0 to 7)
  2^prs_osr internal pressure measurements per result
  A higher value increases precision

  startMeasureBothCont enables background mode
  temperature and pressure ar measured automatically
  High precision and hgh measure rates at the same time are not available.
  Consult Datasheet (or trial and error) for more information

  Use one of the commented lines below instead to measure only temperature or pressure
  int16_t ret = Dps310PressureSensor.startMeasureTempCont(temp_mr, temp_osr);
  int16_t ret = Dps310PressureSensor.startMeasurePressureCont(prs_mr, prs_osr);
*************************************************************************************************/

  if (ret != 0)
  {
    Serial.print("Init FAILED! ret = ");
    Serial.println(ret);
  }
  else
  {
    Serial.println("Init complete!");
  }
}

void loop() { 
  unsigned char pressureCount = 20;
  int32_t pressure[pressureCount];
  unsigned char temperatureCount = 20;
  int32_t temperature[temperatureCount];
  
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= intervalMillis) {
    previousMillis = currentMillis;

    /************************************************************************************************************
    This function writes the results of continuous measurements to the arrays given as parameters
    The parameters temperatureCount and pressureCount should hold the sizes of the arrays temperature and pressure when the function is called
    After the end of the function, temperatureCount and pressureCount hold the numbers of values written to the arrays
    Note: The Dps310 cannot save more than 32 results. When its result buffer is full, it won't save any new measurement results 
    ***************************************************************************************************************/
    int16_t ret = PressureSensor.getContResults(temperature, temperatureCount, pressure, pressureCount);
  
    if (ret != 0)
    {
        Serial.println();
        Serial.println();
        Serial.print("FAIL! ret = ");
        Serial.println(ret);
    }
    else
    //sending data
    {
      MyPacketSerial.send((uint8_t*)pressure,pressureCount*4);
    }
  }
}
