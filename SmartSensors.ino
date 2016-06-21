/*
	This file is part of Waag Society's Smart Citizens Lab project.

	This code is free software: you can 
	redistribute it and/or modify it under the terms of the GNU 
	General Public License as published by the Free Software 
	Foundation, either version 3 of the License, or (at your option) 
	any later version.

	This code is distributed in the hope 
	that it will be useful, but WITHOUT ANY WARRANTY; without even 
	the implied warranty of MERCHANTABILITY or FITNESS FOR A 
	PARTICULAR PURPOSE. See the GNU General Public License for more 
	details.

	You should have received a copy of the GNU General Public License
	along with this code. If not, see 
	<http://www.gnu.org/licenses/>.
*/

/* *******************************************************
/  Instructions

In order to use this code you will have to select the right sensor by setting the SensorIndex value below:

 0: MQ-135 Gas sensor, connect pint A0 on the sensor board to pin A0 on arduino
 1: Seeed AirQuality, connect the yellow wire to A0
 2: Seeed Humidity Temperature sensor, connect to A0
 3: K30 CO2 sensor, connect to I2C
 4: Temperature sensor, connect to A0
 5: Conductivity, connect to A0
 6: Sound, connect to "out" to A0

*/

int SensorIndex = 3;


/* *******************************************************
/  Libraries
*/
#include <Wire.h>
#include "LiquidCrystal_I2C.h" // Needed for operating the LCD screen
#include <DHT.h>
/* *******************************************************
*/

/* *******************************************************
/  LCD
*/
// Set the LCD address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(0x27,16,2);
/* *******************************************************
*/


/* *******************************************************
/  Sensor
*/
String SensorNames[] = {"MQ135 Gas Sensor","Seeed Airquality","Seeed HumTemp","K30 CO2 sensor","Thermistor", "Conductivity", "Sound"};
const int SensorPins[] = {A0, A0, A0, 0, A0, A0, A0}; 
String SensorUnits[] = {" / 1024", " / 1024", "", " ppm", " *C", " / 1024 Res", " / 502"};
/* *******************************************************
*/

/* *******************************************************
/  Sensor specific code
*/

// 2: Seeed HumTemp
#define DHTTYPE DHT22   // DHT 22  (AM2302)
DHT dht(SensorPins[SensorIndex], DHTTYPE);

// 3: K30 CO2 sensor
int co2Addr = 0x68;

// 6: Sound
const int sampleWindow = 50; // Sample window width in mS (50 mS = 20Hz)
unsigned int sample;

/* *******************************************************
*/
  
void setup() {            //This function gets called when the Arduino starts
  Serial.begin(9600);   //This code sets up the Serial port at 115200 baud rate

  delay(500);
  
  Serial.println("Smart Sensors");

  // Initialize the LCD and print a message
  lcd.init(); // start the LCD
  lcd.backlight(); // enable the backlight
  lcd.clear(); // Clear the screen from any character
  lcd.setCursor(0,0); // Start writing from position 0 row 0, so top left
  lcd.print(SensorNames[SensorIndex]);
  delay(1000);
  
  /* *******************************************************
  /  Sensor specific code
  */
  
  if(SensorIndex == 2) {
    dht.begin();
  }
  
  /* ******************************************************* */
}


/* *******************************************************
/  Loop, this code is constantly repeated
*/ 
void loop() {
  if(SensorIndex != 6) { lcd.clear(); }
  lcd.setCursor(0,0);
  lcd.print(SensorNames[SensorIndex]);
  
  int SensorValue = 0;
  
  if(SensorIndex == 0 || SensorIndex == 1 || SensorIndex == 5) {
    lcd.setCursor(0,1);
    SensorValue = analogRead(SensorPins[SensorIndex]);
    lcd.print(SensorValue);
    lcd.print(SensorUnits[SensorIndex]);
  }
  
  if(SensorIndex == 1) {
    if (SensorValue >= 0)// if a valid data returned.
    {
        if (SensorValue > 700) {
          Serial.println("High pollution!");
          lcd.print(" High");
        }
        else if (SensorValue > 500) {
          Serial.println("Medium pollution!");
          lcd.print(" Medium");
        }
        else if (SensorValue > 200) {
          Serial.println("Low pollution!");
          lcd.print(" Low");
        }
        else {
          Serial.println("Fresh air");
          lcd.print(" Fresh");
        }
    }
  }
  
  if(SensorIndex == 2) {
    float h = dht.readHumidity();
    float t = dht.readTemperature();
    if (isnan(t) || isnan(h)) {
        Serial.println("Failed to read from DHT");
    }
    else {
        Serial.print("Humidity: "); 
        Serial.print(h);
        Serial.print(" %\t");
        Serial.print("Temperature: "); 
        Serial.print(t);
        Serial.println(" *C");
        lcd.setCursor(0,1);
        lcd.print("H ");
        lcd.print((int) h);
        lcd.print(" % ");
        lcd.print("T ");
        lcd.print((int) t);
        lcd.print(" *C");
    }
  }
  
  if(SensorIndex == 3) {
    int co2Value = readCO2();
    if(co2Value > 0)
    {
      Serial.print("CO2 Value: ");
      Serial.print(co2Value);
      Serial.println("ppm");
      lcd.setCursor(0,1);
      lcd.print(co2Value);
      lcd.print(SensorUnits[SensorIndex]);
    }
    else
    {
      Serial.println("Checksum failed / Communication failure");
    }
    delay(1500);
  }
  
  if(SensorIndex == 4) {
    Serial.println(Thermister(analogRead(SensorPins[SensorIndex])));
    lcd.setCursor(0,1);
    lcd.print(Thermister(analogRead(SensorPins[SensorIndex])));
    lcd.print(SensorUnits[SensorIndex]);
  }
  
  if(SensorIndex == 6) {
    unsigned long startMillis= millis();  // Start of sample window
    unsigned int peakToPeak = 0;   // peak-to-peak level
     
    unsigned int signalMax = 0;
    unsigned int signalMin = 1024;
     
    // collect data for 50 mS
    while (millis() - startMillis < sampleWindow) {
      sample = analogRead(0);
      if (sample < 1024) { // toss out spurious readings
        if (sample > signalMax) {
          signalMax = sample;  // save just the max levels
        }
        else if (sample < signalMin) {
          signalMin = sample;  // save just the min levels
        }
      }
    }
    peakToPeak = signalMax - signalMin;  // max - min = peak-peak amplitude
    //double volts = (peakToPeak * 3.3) / 1024;  // convert to volts
     
    Serial.println(peakToPeak);
    lcd.setCursor(0,1); 
    lcd.print(peakToPeak);
    lcd.print(SensorUnits[SensorIndex]);
  }
  else {
    delay(500);
  }
}
/* *******************************************************
*/
int readCO2()
{
  int co2_value = 0;
  // We will store the CO2 value inside this variable.
  digitalWrite(13, HIGH);
  // On most Arduino platforms this pin is used as an indicator light.
  //////////////////////////
  /* Begin Write Sequence */
  //////////////////////////
  Wire.beginTransmission(co2Addr);
  Wire.write(0x22);
  Wire.write(0x00);
  Wire.write(0x08);
  Wire.write(0x2A);
  Wire.endTransmission();
  /////////////////////////
  /* End Write Sequence. */
  /////////////////////////
  /*
  We wait 10ms for the sensor to process our command.
  The sensors's primary duties are to accurately
  measure CO2 values. Waiting 10ms will ensure the
  data is properly written to RAM
  */
  delay(10);
  /////////////////////////
  /* Begin Read Sequence */
  /////////////////////////
  /*
  Since we requested 2 bytes from the sensor we must
  read in 4 bytes. This includes the payload, checksum,
  and command status byte.
  */
  Wire.requestFrom(co2Addr, 4);
  byte i = 0;
  byte buffer[4] = {0, 0, 0, 0};
  /*
  Wire.available() is not nessessary. Implementation is obscure but we leave
  it in here for portability and to future proof our code
  */
  while(Wire.available())
    {
    buffer[i] = Wire.read();
    i++;
    }
  ///////////////////////
  /* End Read Sequence */
  ///////////////////////
  /*
  Using some bitwise manipulation we will shift our buffer
  into an integer for general consumption
  */
  co2_value = 0;
  co2_value |= buffer[1] & 0xFF;
  co2_value = co2_value << 8;
  co2_value |= buffer[2] & 0xFF;
  byte sum = 0; //Checksum Byte
  sum = buffer[0] + buffer[1] + buffer[2]; //Byte addition utilizes overflow
  if(sum == buffer[3])
  {
    // Success!
    digitalWrite(13, LOW);
    return co2_value;
  }
  else
  {
    // Failure!
    /*
    Checksum failure can be due to a number of factors,
    fuzzy electrons, sensor busy, etc.
    */
    digitalWrite(13, LOW);
    return 0;
  }
}

// Thermistor function
double Thermister(int RawADC) {  //Function to perform the fancy math of the Steinhart-Hart equation
  double Temp;
  Temp = log(((10240000/RawADC) - 10000));
  Temp = 1 / (0.001129148 + (0.000234125 + (0.0000000876741 * Temp * Temp ))* Temp );
  Temp = Temp - 273.15;              // Convert Kelvin to Celsius
  //Temp = (Temp * 9.0)/ 5.0 + 32.0; // Celsius to Fahrenheit - comment out this line if you need Celsius
  return Temp;
}
