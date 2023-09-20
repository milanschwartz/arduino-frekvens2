// [COMPONENTS]
// A:Arduino Nano 33 BLE SENSE
// M: Max7219 Dotmatrix Module 8x8
// P1: Potmeter 10k
// P2: Potmeter 10k

// [CIRCUIT]
// A.D11 - M.DIN
// A.D12 - M.CS
// A.D13 - M.CLK
// A.+3V3 - P1.VCC, P2.VCC, M.VCC
// A.GND - P1.GND, P2.GND, M.GND
// A.A7 - P1.ANA
// A.A6 - P2.ANA

#include <PDM.h>
#include "arduinoFFT.h"
#include "LedController.hpp"

arduinoFFT FFT;
//This value MUST ALWAYS be a power of 2
const uint16_t samples = 512;
const double samplingFrequency = 41667;
double vReal[samples];
double vImag[samples];

// default number of output channels
static const char channels = 1;
// default PCM output frequency
static const int frequency = samplingFrequency;
// Buffer to read samples into, each sample is 16-bits
short sampleBuffer[samples];
// Number of audio samples read
volatile int samplesRead;

#define DIN 11
#define CS 12
#define CLK 13
LedController<1, 1> lc = LedController<1, 1>();

#define PM1_PIN A6
#define PM2_PIN A7
int pm1;
int pm2;

void setup() {
  lc.init(DIN, CLK, CS);
  lc.setIntensity(8);
  lc.clearMatrix();

  Serial.begin(115200);
  while (!Serial) {};

  // Configure the data receive callback
  PDM.onReceive(onPDMdata);
  PDM.setBufferSize(samples);

  // Optionally set the gain
  // Defaults to 20 on the BLE Sense and 24 on the Portenta Vision Shield
  // PDM.setGain(30);

  // Initialize PDM with:
  // - one channel (mono mode)
  // - a 16 kHz sample rate for the Arduino Nano 33 BLE Sense
  // - a 32 kHz or 64 kHz sample rate for the Arduino Portenta Vision Shield
  if (!PDM.begin(channels, frequency)) {
    Serial.println("Failed to start PDM!");
    while (1)
      ;
  }
  Serial.println("Ready");
}

void loop() {
  pm1 = map(analogRead(PM1_PIN), 0, 1023, -10, 10);
  pm2 = map(analogRead(PM2_PIN), 0, 1023, 0, 1000000);

  //Serial.print("PM1: ");Serial.println(pm1);
  //Serial.print("PM2: ");Serial.println(pm2);

  // Wait for samples to be read
  if (samplesRead) {
    for (uint16_t i = 0; i < samples; i++) {
      vReal[i] = (double)sampleBuffer[i];
      vImag[i] = 0.0;  //Imaginary part must be zeroed in case of looping to avoid wrong calculations and overflows
    }
    FFT = arduinoFFT(vReal, vImag, samples, samplingFrequency); /* Create FFT object */
    FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);            /* Weigh data */
    FFT.Compute(FFT_FORWARD);                                   /* Compute FFT */
    FFT.ComplexToMagnitude();                                   /* Compute magnitudes */
    drawSound();
    samplesRead = 0;
    //delay(10); /* Repeat after delay */
  }
}

void drawSound() {
  int max_bins = 8;
  double bin_values[max_bins] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  int samples_per_bin = samplesRead / max_bins;
  double max_power = pm2;  //400000

  for (int bin = 0; bin < max_bins; bin++) {
    for (int samplenr = 0; samplenr < samples_per_bin; samplenr++) {
      bin_values[bin] += vReal[bin * (samples_per_bin) + samplenr];
    }
    //Serial.print(bin);Serial.print(" "); Serial.println(bin_values[bin]);
    //if (bin_values[bin] > max_power) { max_power = band_values[bin]; }
  }

  for (int bin = 0; bin < max_bins; bin++) {
    int bin_value_power = fscale(0, max_power, 0, 8, bin_values[bin], pm1);  //pm1=6
    // turn on led in the band's column according to the power of the
    lc.setRow(0, bin, lc.reverse(pow(2, bin_value_power) - 1));
  }
}

/**
 * Callback function to process the data from the PDM microphone.
 * NOTE: This callback is executed as part of an ISR.
 * Therefore using `Serial` to print messages inside this function isn't supported.
 * */
void onPDMdata() {
  // Query the number of available bytes
  int bytesAvailable = PDM.available();

  // Read into the sample buffer
  PDM.read(sampleBuffer, bytesAvailable);

  // 16-bit, 2 bytes per sample
  samplesRead = bytesAvailable / 2;
}

// maps to tuneable scale
float fscale(float originalMin, float originalMax, float newBegin, float newEnd, float inputValue, float curve) {

  float OriginalRange = 0;
  float NewRange = 0;
  float zeroRefCurVal = 0;
  float normalizedCurVal = 0;
  float rangedValue = 0;
  boolean invFlag = 0;

  // condition curve parameter
  // limit range

  if (curve > 10) curve = 10;
  if (curve < -10) curve = -10;

  curve = (curve * -.1);   // - invert and scale - this seems more intuitive - postive numbers give more weight to high end on output
  curve = pow(10, curve);  // convert linear scale into lograthimic exponent for other pow function

  /*
   Serial.println(curve * 100, DEC);   // multply by 100 to preserve resolution  
   Serial.println();
   */

  // Check for out of range inputValues
  if (inputValue < originalMin) {
    inputValue = originalMin;
  }
  if (inputValue > originalMax) {
    inputValue = originalMax;
  }

  // Zero Refference the values
  OriginalRange = originalMax - originalMin;

  if (newEnd > newBegin) {
    NewRange = newEnd - newBegin;
  } else {
    NewRange = newBegin - newEnd;
    invFlag = 1;
  }

  zeroRefCurVal = inputValue - originalMin;
  normalizedCurVal = zeroRefCurVal / OriginalRange;  // normalize to 0 - 1 float

  /*
  Serial.print(OriginalRange, DEC);  
   Serial.print("   ");  
   Serial.print(NewRange, DEC);  
   Serial.print("   ");  
   Serial.println(zeroRefCurVal, DEC);  
   Serial.println();  
   */

  // Check for originalMin > originalMax  - the math for all other cases i.e. negative numbers seems to work out fine
  if (originalMin > originalMax) {
    return 0;
  }

  if (invFlag == 0) {
    rangedValue = (pow(normalizedCurVal, curve) * NewRange) + newBegin;

  } else  // invert the ranges
  {
    rangedValue = newBegin - (pow(normalizedCurVal, curve) * NewRange);
  }

  return rangedValue;
}