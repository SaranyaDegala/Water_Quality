#include <OneWire.h>          // Include library for OneWire protocol
#include <DallasTemperature.h> // Include library for DS18B20 temperature sensor

// Define constants and pins
#define TdsSensorPin 27       // Pin connected to the TDS sensor
#define VREF 3.3              // Analog reference voltage (Volt) of the ADC
#define SCOUNT 30             // Number of sample points for averaging

// Initialize variables for TDS sensor
int analogBuffer[SCOUNT];     // Array to store analog readings
int analogBufferTemp[SCOUNT]; // Temporary array for median filtering
int analogBufferIndex = 0;    // Current index for writing to the buffer
int copyIndex = 0;            // Index for copying data for processing
float averageVoltage = 0;     // Variable to store average voltage
float tdsValue = 0;           // Variable to store calculated TDS value
float temperature = 25;       // Default temperature for TDS compensation

// Setup OneWire and DallasTemperature for DS18B20
const int oneWireBus = 4;     // Pin for OneWire bus (DS18B20)
OneWire oneWire(oneWireBus);  // Create OneWire instance
DallasTemperature sensors(&oneWire); // Create DallasTemperature instance

// Turbidity sensor
int turbiditySensorPin = 34; // ADC pin connected to the turbidity sensor

// Function to calculate the median value from an array
int getMedianNum(int bArray[], int iFilterLen) {
  int bTab[iFilterLen]; // Temporary array for sorting
  for (byte i = 0; i < iFilterLen; i++)
    bTab[i] = bArray[i]; // Copy values to temporary array
  int i, j, bTemp;

  // Sort the temporary array using bubble sort
  for (j = 0; j < iFilterLen - 1; j++) {
    for (i = 0; i < iFilterLen - j - 1; i++) {
      if (bTab[i] > bTab[i + 1]) {
        bTemp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }

  // Calculate median value
  if ((iFilterLen & 1) > 0) {
    bTemp = bTab[(iFilterLen - 1) / 2]; // Odd number of elements
  } else {
    // Even number of elements
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2; 
  }
  return bTemp; // Return the median value
}

void setup() {
  Serial.begin(115200); // Start serial communication at 115200 baud
  pinMode(TdsSensorPin, INPUT); // Set TDS sensor pin as input
  pinMode(turbiditySensorPin, INPUT); // Set turbidity sensor pin as input
  sensors.begin(); // Initialize DS18B20 sensors
}

void loop() {
  // Read TDS sensor data every 40 milliseconds
  static unsigned long analogSampleTimepoint = millis();
  if (millis() - analogSampleTimepoint > 40U) { 
    analogSampleTimepoint = millis(); // Update the timepoint
    analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin); // Read analog value
    analogBufferIndex++; // Move to the next index
    if (analogBufferIndex == SCOUNT) {
      analogBufferIndex = 0; // Wrap around if at the end of buffer
    }
  }
  
  // Print TDS value every 800 milliseconds
  static unsigned long printTimepoint = millis();
  if (millis() - printTimepoint > 800U) {
    printTimepoint = millis(); // Update the timepoint
    for (copyIndex = 0; copyIndex < SCOUNT; copyIndex++) {
      analogBufferTemp[copyIndex] = analogBuffer[copyIndex]; // Copy buffer for processing
      
      // Calculate stable voltage value using median filtering
      averageVoltage = getMedianNum(analogBufferTemp, SCOUNT) * (float)VREF / 4096.0;

      // Temperature compensation formula
      float compensationCoefficient = 1.0 + 0.02 * (temperature - 25.0);
      // Apply temperature compensation
      float compensationVoltage = averageVoltage / compensationCoefficient;
      
      // Convert voltage to TDS value using a specific formula
      tdsValue = (133.42 * compensationVoltage * compensationVoltage * compensationVoltage - 
                  255.86 * compensationVoltage * compensationVoltage + 
                  857.39 * compensationVoltage) * 0.5;

      // Print calculated TDS value
      Serial.print("TDS Value: ");
      Serial.print(tdsValue, 0); // Print TDS value as integer
      Serial.println(" ppm");
    }
  }
  
  // Read turbidity sensor data
  int sensorValue = analogRead(turbiditySensorPin); // Read the turbidity sensor value
  int turbidity = map(sensorValue, 0, 4095, 100, 0); // Map the value to turbidity percentage
  
  // Print turbidity value
  Serial.print("Turbidity: ");
  Serial.println(turbidity); // Print turbidity value

  // Determine water quality based on turbidity value
  if (turbidity < 20) {
    Serial.println("Water is CLEAR"); // Clear water
  } 
  else if (turbidity >= 20 && turbidity < 50) {
    Serial.println("Water is CLOUDY"); // Cloudy water
  } 
  else if (turbidity >= 50) {
    Serial.println("Water is DIRTY"); // Dirty water
  }

  // Read temperature data from DS18B20
  sensors.requestTemperatures(); // Request temperature readings
  float temperatureC = sensors.getTempCByIndex(0); // Get temperature in Celsius
  float temperatureF = sensors.getTempFByIndex(0); // Get temperature in Fahrenheit
  
  // Print temperature
  Serial.print("Temperature: ");
  Serial.print(temperatureC); // Print temperature in Celsius
  Serial.println(" ºC");
  
  delay(1000); // Delay before the next loop iteration
}
