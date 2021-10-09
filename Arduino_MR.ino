// include the following libraries
#include <SPI.h>
#include <SD.h>
#include <Adafruit_MCP4728.h>
#include <Adafruit_ADS1X15.h>
#include <Wire.h>
#include "parameters.h"
#include "config.h"
Adafruit_MCP4728 mcp;
Adafruit_ADS1115 ads;
////////////////////
// SD Card Globals //
/////////////////////
bool sdCardPresent = false; // Keeps track of if SD card is plugged in
String logFileName; // Active logging file
String logFileBuffer; // Buffer for logged data. Max is set in config

void setup() {

  Serial.begin(9600);

  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  if (!mcp.begin()) {
    Serial.println("Failed to find MCP4728 chip"); //initialize the DAC chip
  }
  if (!ads.begin()) {
    Serial.println("Failed to initialize ADS1115"); //initialize the ADC chip
  }
  if ( initSD() ) //initialize the SD card breakout
  {
    sdCardPresent = true;
    // Get the next, available log file name
    logFileName = nextLogFile();
  }
  //PINS INITIALIZING
  // define the reading channels transistors and initliaze them to 0
  for (int i = 0; i < Pin_count; i++) {
    pinMode(Vref[i], OUTPUT);
    digitalWrite(Vref[i], LOW);
  }
  // define the writing channels transistors and initliaze them to 0
  pinMode(V1, OUTPUT);
  digitalWrite(V1, LOW);
  pinMode(V2, OUTPUT);
  digitalWrite(V2, LOW);

  mcp.setChannelValue(Vplus, 0); // initialize the vplus source to 0
  mcp.setChannelValue(Vneg, 0); // initialize the vneg source to 0
  mcp.setChannelValue(vgs1, 0); // initialize vgs1 transistor
  mcp.setChannelValue(vgs2, 0); // initialize vgs2 transistor
}

void loop() {
  Start_Time = millis(); // mark the start time of the measurment
  switch (mode) { // mode variable define the test carried
    case 1: // positive sweep test
      for (int i = 0; i < cycles; i++) {
        Serial.print("cycle: ");
        Serial.println(i + 1);
        Serial.print("\n\n");
        Sweep_P();
        if (dual == 1) // if dual feature is selected
        {
          Reverse = 1;
          Sweep_P();
        }
        Reverse = 0;
      }
      break;
    case 2: // negative sweep test
      for (int j = 0; j < cycles; j++) {
        Serial.print("cycle: ");
        Serial.println(j + 1);
        Serial.print("\n\n");
        Sweep_N();
        if (dual == 1)
        {
          Reverse = 1;
          Sweep_N();
        }
        Reverse = 0;
      }
      break;
    case 3: // SET-RESET test
      for (int j = 0; j < cycles; j++) {
        Serial.print("cycle: ");
        Serial.println(j + 1);
        Serial.print("\n\n");
        Sweep_P();
        if (dual == 1)
        {
          Reverse = 1;
          Sweep_P();
        }
        mcp.setChannelValue(vgs1, 0); // turn off vgs1 transistor
        Reverse = 0;
        Sweep_N();
        if (dual == 1)
        {
          Reverse = 1;
          Sweep_N();
        }
        mcp.setChannelValue(vgs2, 0); // turn off vgs2 transistor
        Reverse = 0;
      }
      break;
    case 4: // DC test
      DC_bias();
      break;
    case 5: // Pulse test
      Read_Pulse();
      break;
  }
  // switch both sources off
  mcp.setChannelValue(Vplus, 0);
  mcp.setChannelValue(Vneg, 0);
  // after test finished go to idle state
  stop();
}

void Sweep_P(void)
{
  // define output parameters that logged to the SD card
  float V = 0;
  float I = 0;
  float RM = 0;
  float T = 0;
  mcp.setChannelValue(Vneg, 4095); // to avoid reverse leaking
  mcp.setChannelValue(vgs1, vgs); // define the compliance current in positive bias
  if (Reverse == 0) { // during forward sweep
    int j = 0;
    VplusD = startvoltage1; // set the power source to start voltage
    int points = (endvoltage1 - startvoltage1) / stepvoltage1; // calculate total number of points
    for (j = 0; j <= points; j++) {
      digitalWrite(V1, HIGH); // set V1 transistor ON
      mcp.setChannelValue(Vplus, VplusD); // write the voltage to the power source pin
      delay(200);
      VMpD = ads.readADC_SingleEnded(VMp); // measure the MR positive terminal voltage
      VMpA = ads.computeVolts(VMpD); // convert the digital value to analog value
      delay(50);
      VMnD = ads.readADC_SingleEnded(VMn); // measure the MR negative terminal voltage
      VMnA = ads.computeVolts(VMnD); // convert the digital value to analog value
      VM = VMpA - VMnA; // compute measured VM
      digitalWrite(V1, LOW); // SET V1 transistor off
      RM = ReadMR_pos(readvoltage1); // call the resistance read function and get RM
      T = (millis() - Start_Time) / 1000.0; // calculate the time of the measurment
      if (RM == 0) {
        I = 0;
      }
      else {
        I = VM / RM;
      }
      if (measuredV == true) { // for measured feature, set logged V value to VM
        V = VM;
      }
      else {
        V = VplusD / 819.0; // for source feature, set logged V value to VplusD from the DAC
      }
      Serial.print("j = ");
      Serial.print(j);
      Serial.print("\t");
      Serial.print("T = ");
      Serial.print(T, 2);
      Serial.print("\t");
      Serial.print("V = ");
      Serial.print(V, 4);
      Serial.print(" V\t");
      Serial.print("I = ");
      Serial.print(I * 1E6);
      Serial.print(" uA\t");
      Serial.print("RM = ");
      Serial.print(RM);
      Serial.print(" ohm\t");
      Serial.println();
      Serial.println("");
      if (sdCardPresent)
        logIMUData(j, T, V, I, RM); // log the values to an SD card
      VplusD = VplusD + stepvoltage1; // increment the power supply with the step voltage for the next sweeping point
      pointer1 = j; // set the pointer to the last sweeping point for the reverse sweep if dual = 1
    }
  }
  if (Reverse == 1) // reverse sweep (if dual = 1)
  {
    int j = pointer1 + 1;
    VplusD = VplusD - stepvoltage1; // decrement the power suply with the step voltage
    for (int i = pointer1; i >= 1; i--) { // repeate from the end voltage to start voltage
      VplusD = VplusD - stepvoltage1;
      digitalWrite(V1, HIGH); // set V1 transistor ON
      mcp.setChannelValue(Vplus, VplusD);
      delay(200);
      VMpD = ads.readADC_SingleEnded(VMp); // measure the MR positive terminal voltage
      VMpA = ads.computeVolts(VMpD); // convert the digital value to analog value
      delay(50);
      VMnD = ads.readADC_SingleEnded(VMn); // measure the MR negative terminal voltage
      VMnA = ads.computeVolts(VMnD); // convert the digital value to analog value
      VM = VMpA - VMnA; // compute measured VM
      digitalWrite(V1, LOW); // set V1 transistor OFF
      RM = ReadMR_pos(readvoltage1); // call the resistance read function and get RM
      T = (millis() - Start_Time) / 1000.0; // calculate the time of the measurment
      if (RM == 0) {
        I = 0;
      }
      else {
        I = VM / RM;
      }
      if (measuredV == true) { // for measured feature, set logged V value to VM
        V = VM;
      }
      else {
        V = VplusD / 819.0; // for source feature, set logged V value to VplusD from the DAC
      }
      Serial.print("i = ");
      Serial.print(i);
      Serial.print("\t");
      Serial.print("T = ");
      Serial.print(T, 2);
      Serial.print("\t");
      Serial.print("V = ");
      Serial.print(V, 4);
      Serial.print(" V\t");
      Serial.print("I = ");
      Serial.print(I * 1E6);
      Serial.print(" uA\t");
      Serial.print("RM = ");
      Serial.print(RM);
      Serial.print(" ohm\t");
      Serial.println();
      Serial.println("------------------------------------------------------------");
      if (sdCardPresent)
        logIMUData(j, T, V, I, RM); // Log new data
      j = j + 1;
    }
  }
}

void Sweep_N(void)
{
  // define output parameters that logged to the SD card
  float V = 0;
  float I = 0;
  float RM = 0;
  float T = 0;
  mcp.setChannelValue(vgs2, vgsNeg); // define the compliance current in negative bias
  mcp.setChannelValue(Vplus, 4095); // to avoid reverse leaking
  if (Reverse == 0) { // during forward sweep
    int j = 0;
    VnegD = startvoltage2; // set the power source to start voltage
    int points = (endvoltage2 - startvoltage2) / stepvoltage2; // calculate total number of points
    for (j = 0; j <= points; j++) {
      digitalWrite(V2, HIGH); // set V2 transistor ON
      mcp.setChannelValue(Vneg, VnegD); // write the voltage to the power source pin
      delay(200);
      VMpD = ads.readADC_SingleEnded(VMp); // measure the MR positive terminal voltage
      VMpA = ads.computeVolts(VMpD); // convert the digital value to analog value
      delay(50);
      VMnD = ads.readADC_SingleEnded(VMn);// measure the MR negative terminal voltage
      VMnA = ads.computeVolts(VMnD); // convert the digital value to analog value
      VM = VMpA - VMnA; // compute measured VM
      digitalWrite(V2, LOW);  // SET V2 transistor off
      RM = ReadMR_neg(readvoltage2); // call the resistance read function and get RM
      T = (millis() - Start_Time) / 1000.0; // calculate the time of the measurment
      if (RM == 0) {
        I = 0;
      }
      else {
        I = VM / RM;
      }
      if (measuredV == true) { // for measured feature, set logged V value to VM
        V = VM;
      }
      else {
        V = (VnegD / 819.0) * -1; // for source feature, set logged V value to VplusD from the DAC
      }
      Serial.print("j = ");
      Serial.print(j);
      Serial.print("\t");
      Serial.print("T = ");
      Serial.print(T, 2);
      Serial.print("\t");
      Serial.print("V = ");
      Serial.print(V, 4);
      Serial.print(" V\t");
      Serial.print("I = ");
      Serial.print(I * 1E6);
      Serial.print(" uA\t");
      Serial.print("RM = ");
      Serial.print(RM);
      Serial.print(" ohm\t");
      Serial.println();
      Serial.println("------------------------------------------------------------");
      if (sdCardPresent)
        logIMUData(j, T, V, I, RM); // log the values to an SD card
      VnegD = VnegD + stepvoltage2; // increment the power supply with the step voltage for the next sweeping point
      pointer2 = j; // set the pointer to the last sweeping point for the reverse sweep if dual = 1
    }
  }
  if (Reverse == 1) // reverse sweep (if dual = 1)
  {
    int j = pointer2 + 1;
    VnegD = VnegD - stepvoltage2; // decrement the power suply with the step voltage
    for (int i = pointer2; i >= 1; i--) { // repeate from the end voltage to start voltage
      VnegD = VnegD - stepvoltage2;
      digitalWrite(V2, HIGH); // set V2 transistor ON
      mcp.setChannelValue(Vneg, VnegD);
      delay(200);
      VMpD = ads.readADC_SingleEnded(VMp); // measure the MR positive terminal voltage
      VMpA = ads.computeVolts(VMpD); // convert the digital value to analog value
      delay(50);
      VMnD = ads.readADC_SingleEnded(VMn); // measure the MR negative terminal voltage
      VMnA = ads.computeVolts(VMnD); // convert the digital value to analog value
      VM = VMpA - VMnA; // compute measured VM
      digitalWrite(V2, LOW); // set V2 transistor OFF
      RM = ReadMR_neg(readvoltage2); // call the resistance read function and get RM
      T = (millis() - Start_Time) / 1000.0; // calculate the time of the measurment
      if (RM == 0) {
        I = 0;
      }
      else {
        I = VM / RM;
      }
      if (measuredV == true) { // for measured feature, set logged V value to VM
        V = VM;
      }
      else {
        V = (VplusD / 819.0) * -1; // for source feature, set logged V value to VplusD from the DAC
      }
      Serial.print("i = ");
      Serial.print(i);
      Serial.print("\t");
      Serial.print("T = ");
      Serial.print(T, 2);
      Serial.print("\t");
      Serial.print("V = ");
      Serial.print(V, 4);
      Serial.print(" V\t");
      Serial.print("I = ");
      Serial.print(I * 1E6);
      Serial.print(" uA\t");
      Serial.print("RM = ");
      Serial.print(RM);
      Serial.print(" ohm\t");
      Serial.println();
      Serial.println("------------------------------------------------------------");
      if (sdCardPresent)
        logIMUData(j, T, V, I, RM); // Log new data
      j = j + 1;
    }
  }
}


void DC_bias(void)
{

  delay(hold4 * 1000); // delay measurment with the specified hold time
  // define output parameters that logged to the SD card
  float V = 0;
  float I = 0;
  float RM = 0;
  float T = 0;
  mcp.setChannelValue(vgs1, vgs); // define the compliance current in positive bias
  mcp.setChannelValue(Vneg, 4095); // to avoid reverse leaking
  int points = samples4;
  for (int j = 1; j <= points; j++)
  {
    digitalWrite(V1, HIGH); // set V1 transistor ON
    mcp.setChannelValue(Vplus, dcbias); // set the power source to dc voltage
    delay(200);
    delay(interval4 * 1000); // delay reading pulse by the interval time
    VMpD = ads.readADC_SingleEnded(VMp); // measure the MR positive terminal voltage
    VMpA = ads.computeVolts(VMpD); // convert the digital value to analog value
    delay(50);
    VMnD = ads.readADC_SingleEnded(VMn); // measure the MR negative terminal voltage
    VMnA = ads.computeVolts(VMnD); // convert the digital value to analog value
    VM = VMpA - VMnA; // compute measured VM
    digitalWrite(V1, LOW); // set V1 transistor OFF
    RM = ReadMR_pos(readvoltage4); // call the resistance read function and get RM
    T = (millis() - Start_Time) / 1000.0; // calculate the time of the measurment
    if (RM == 0) {
      I = 0;
    }
    else {
      I = VM / RM;
    }
    if (measuredV == true) { // for measured feature, set logged V value to VM
      V = VM;
    }
    else {
      V = VplusD / 819.0; // for source feature, set logged V value to VplusD from the DAC
    }

    Serial.print("j = ");
    Serial.print(j);
    Serial.print("\t");
    Serial.print("T = ");
    Serial.print(T, 2);
    Serial.print("\t");
    Serial.print("V = ");
    Serial.print(V, 4);
    Serial.print(" V\t");
    Serial.print("I = ");
    Serial.print(I * 1E6);
    Serial.print(" uA\t");
    Serial.print("RM = ");
    Serial.print(RM);
    Serial.print(" ohm\t");
    Serial.println();
    Serial.println("------------------------------------------------------------");
    if (sdCardPresent)
      logIMUData(j, T, V, I, RM); // log the values to an SD card
    if (T > (interval4 * samples4)) { // if the measurment is taking more time than interval*samples break
      break;
    }
  }
}

void Read_Pulse(void)
{
  delay(hold5 * 1000); // delay measurment with the specified hold time
  // delay measurment with the specified hold time
  float V = 0;
  float I = 0;
  float RM = 0;
  float T = 0;
  mcp.setChannelValue(vgs1, vgs); // define the compliance current in positive bias
  mcp.setChannelValue(Vneg, 4095); // to avoid reverse leaking
  int points = samples5;
  for (int j = 1; j <= points; j++)
  {
    mcp.setChannelValue(Vplus, 0); // set the source to 0 V (idle state)
    digitalWrite(V1, HIGH); // set V1 transistor ON
    delay(interval5 * 1000); // delay reading pulse by the interval time
    mcp.setChannelValue(Vplus, readvoltage5);  // set the power source to reading voltage
    delay(200);
    delay(width5 * 1000); // set the pulse width
    VMpD = ads.readADC_SingleEnded(VMp); // measure the MR positive terminal voltage
    VMpA = ads.computeVolts(VMpD); // convert the digital value to analog value
    delay(50);
    VMnD = ads.readADC_SingleEnded(VMn); // measure the MR negative terminal voltage
    VMnA = ads.computeVolts(VMnD); // convert the digital value to analog value
    VM = VMpA - VMnA; // compute measured VM
    mcp.setChannelValue(Vplus, 0); // set power source to idle state again
    digitalWrite(V1, LOW); // set V1 transistor OFF
    RM = ReadMR_pos(readvoltage5); // call the resistance read function and get RM
    T = (millis() - Start_Time) / 1000.0; // calculate the time of the measurment
    if (RM == 0) {
      I = 0;
    }
    else {
      I = VM / RM;
    }
    if (measuredV == true) { // for measured feature, set logged V value to VM
      V = VM;
    }
    else {
      V = readvoltage5 / 819.0; // for source feature, set logged V value to VplusD from the DAC
    }
    Serial.print("j = ");
    Serial.print(j);
    Serial.print("\t");
    Serial.print("T = ");
    Serial.print(T, 2);
    Serial.print("\t");
    Serial.print("V = ");
    Serial.print(V, 4);
    Serial.print(" V\t");
    Serial.print("I = ");
    Serial.print(I * 1E6);
    Serial.print(" uA\t");
    Serial.print("RM = ");
    Serial.print(RM);
    Serial.print(" ohm\t");
    Serial.println();
    Serial.println("------------------------------------------------------------");
    if (sdCardPresent)
      logIMUData(j, T, V, I, RM); // log the values to an SD card
  }
}

float ReadMR_pos(int readvoltage)
{
  float readvoltageA = readvoltage / 819.0; // calculate the analog reading voltage
  mcp.setChannelValue(Vplus, readvoltage); // set the power source to the read voltage
  float I = 0.0;
  float RM = 0.0;
  float V = 0.0;
  if (startp == 1) // at first reading set the current reading channel to CH1 with Rref1
  {
    CurrentVrefp = Vref[0];
    CurrentRrefp = Rref[0];
    startp = 0;
  }
  Serial.print("Vref = ");
  Serial.print(CurrentVrefp);
  Serial.print("\t Rref = ");
  Serial.print(CurrentRrefp);
  Serial.println();
  Serial.println();
  digitalWrite(CurrentVrefp, HIGH); // set current reading channel ON
  delay(950);
  VMpD = ads.readADC_SingleEnded(VMp); // measure the MR positive terminal voltage
  VMpA = ads.computeVolts(VMpD); // convert the digital value to analog value
  delay(50);
  VMnD = ads.readADC_SingleEnded(VMn); // measure the MR negative terminal voltage
  VMnA = ads.computeVolts(VMnD); // convert the digital value to analog value
  VM = VMpA - VMnA; // compute measured VM
  // if the measured VM is too small compared to read voltage and current channel is CH1 switch to CH2 and update VM
  if ((VM < 0.1 * readvoltageA) && CurrentVrefp == Vref[0])
  {
    digitalWrite(CurrentVrefp, LOW);
    CurrentVrefp = Vref[1];
    CurrentRrefp = Rref[1];
    digitalWrite(CurrentVrefp, HIGH);
    delay(950);
    VMpD = ads.readADC_SingleEnded(VMp);
    VMpA = ads.computeVolts(VMpD);
    delay(50);
    VMnD = ads.readADC_SingleEnded(VMn);
    VMnA = ads.computeVolts(VMnD);
    VM =  VMpA - VMnA;
    Serial.print("reference changed to 10k");
    Serial.println();
  }
  // if the measured VM is too big compared to read voltage and current channel is CH2 switch to CH1 and update VM
  else if ((VM > 0.90 * readvoltageA) && CurrentVrefp == Vref[1])
  {
    digitalWrite(CurrentVrefp, LOW);
    CurrentVrefp = Vref[0];
    CurrentRrefp = Rref[0];
    digitalWrite(CurrentVrefp, HIGH);
    delay(950);
    VMpD = ads.readADC_SingleEnded(VMp);
    VMpA = ads.computeVolts(VMpD);
    delay(50);
    VMnD = ads.readADC_SingleEnded(VMn);
    VMnA = ads.computeVolts(VMnD);
    VM =  VMpA - VMnA;
    Serial.print("reference changed to 1M");
    Serial.println();
  }

  digitalWrite(CurrentVrefp, LOW); // set current reading channel off
  I = (readvoltageA - VMpA) / CurrentRrefp; // calculate I and RM
  RM = VM / I; // calculate I and RM
  Serial.print("VMp = ");
  Serial.print(VMpA, 4);
  Serial.print("\t");
  Serial.print("VMn = ");
  Serial.print(VMnA, 4);
  Serial.print("\t");
  Serial.print("VM = ");
  Serial.print(VM, 4);
  Serial.print("\t");
  Serial.println();
  Serial.println();
  Serial.print("Vread = ");
  Serial.print(readvoltageA);
  Serial.print(" V\t");
  Serial.print("I = ");
  Serial.print(I * 1E6);
  Serial.print(" uA\t");
  Serial.print("RM = ");
  Serial.print(RM);
  Serial.print(" ohm\t");
  Serial.println();
  Serial.println("------------------------------------------------------------");

  RM = abs(RM);

  return RM; // return the memristor resistance
}

float ReadMR_neg(int readvoltage)
{
  float readvoltageA = readvoltage / 819.0; // calculate the analog reading voltage
  mcp.setChannelValue(Vneg, readvoltage); // set the power source to the read voltage
  float I = 0.0;
  float RM = 0.0;
  float V = 0.0;
  if (startn == 1) // at first reading set the current reading channel to CH3 with Rref3
  {
    CurrentVrefn = Vref[2];
    CurrentRrefn = Rref[2];
    startn = 0;
  }
  Serial.print("Vref = ");
  Serial.print(CurrentVrefn);
  Serial.print("\t Rref = ");
  Serial.print(CurrentRrefn);
  Serial.println();
  Serial.println();
  digitalWrite(CurrentVrefn, HIGH); // set current reading channel ON
  delay(950);
  VMpD = ads.readADC_SingleEnded(VMp); // measure the MR positive terminal voltage
  VMpA = ads.computeVolts(VMpD); // convert the digital value to analog value
  delay(50);
  VMnD = ads.readADC_SingleEnded(VMn); // measure the MR negative terminal voltage
  VMnA = ads.computeVolts(VMnD); // convert the digital value to analog value
  VM =  VMnA - VMpA; // compute measured VM
  // if the measured VM is too small compared to read voltage and current channel is CH3 switch to CH4 and update VM
  if ((VM < 0.1 * readvoltageA) && CurrentVrefn == Vref[2])
  {
    digitalWrite(CurrentVrefn, LOW);
    CurrentVrefn = Vref[3];
    CurrentRrefn = Rref[3];
    digitalWrite(CurrentVrefn, HIGH);
    delay(950);
    VMpD = ads.readADC_SingleEnded(VMp);
    VMpA = ads.computeVolts(VMpD);
    delay(50);
    VMnD = ads.readADC_SingleEnded(VMn);
    VMnA = ads.computeVolts(VMnD);
    VM =  VMnA - VMpA;
    Serial.print("reference changed to 10k");
    Serial.println();
  }
  // if the measured VM is too big compared to read voltage and current channel is CH4 switch to CH3 and update VM
  else if ((VM > 0.90 * readvoltageA) && CurrentVrefn == Vref[3])
  {
    digitalWrite(CurrentVrefn, LOW);
    CurrentVrefn = Vref[2];
    CurrentRrefn = Rref[2];
    digitalWrite(CurrentVrefn, HIGH);
    delay(950);
    VMpD = ads.readADC_SingleEnded(VMp);
    VMpA = ads.computeVolts(VMpD);
    delay(50);
    VMnD = ads.readADC_SingleEnded(VMn);
    VMnA = ads.computeVolts(VMnD);
    VM =  VMnA - VMpA;
    Serial.print("reference changed to 1M");
    Serial.println();
  }
  digitalWrite(CurrentVrefn, LOW); // set current reading channel off
  I = (readvoltageA - VMnA) / CurrentRrefn; // calculate I and RM
  RM = (VM / I); // calculate I and RM
  Serial.print("VMp = ");
  Serial.print(VMpA, 4);
  Serial.print("\t");
  Serial.print("VMn = ");
  Serial.print(VMnA, 4);
  Serial.print("\t");
  Serial.print("VM = ");
  Serial.print(VM, 4);
  Serial.print("\t");
  Serial.println();
  Serial.println();
  Serial.print("Vread = ");
  Serial.print(readvoltageA);
  Serial.print(" V\t");
  Serial.print("I = ");
  Serial.print(I * 1E6);
  Serial.print(" uA\t");
  Serial.print("RM = ");
  Serial.print(RM);
  Serial.print(" ohm\t");
  Serial.println();
  Serial.println("------------------------------------------------------------");
  RM = abs(RM);

  return RM; // return the memristor resistance
}

// function that prepare the logged string to sd card
void logIMUData(int j, float T, float V, float I, float RM)
{
  String imuLog = "";
  imuLog += String(j) + " ";
  imuLog += String(T, 2) + " ";
  imuLog += String(V, 4) + " ";
  imuLog += String(I, 10) + " ";
  imuLog += String(RM, 3) + " ";

  // Remove last comma/space:
  imuLog.remove(imuLog.length() - 2, 2);
  imuLog += "\r\n"; // Add a new line

  logFileBuffer = ""; // Clear SD log buffer

  // Add new line to SD log buffer
  logFileBuffer += imuLog;

  sdLogString(logFileBuffer); // Log SD buffer
}

// Log a string to the SD card
bool sdLogString(String toLog)
{
  // Open the current file name:
  File logFile = SD.open(logFileName, FILE_WRITE);

  // If the file will get too big with this new string, create
  // a new one, and open it.
  if (logFile.size() > (SD_MAX_FILE_SIZE - toLog.length()))
  {
    logFileName = nextLogFile();
    logFile = SD.open(logFileName, FILE_WRITE);
  }

  // If the log file opened properly, add the string to it.
  if (logFile)
  {
    logFile.print(toLog);
    logFile.close();
    return true; // Return success
  }

  return false; // Return fail
}

bool initSD(void)
{
  // SD.begin should return true if a valid SD card is present
  if ( !SD.begin(SD_CHIP_SELECT_PIN) )
  {
    return false;
  }

  return true;
}
// function that prepare the txt logged file
String nextLogFile(void)
{
  String filename;
  int logIndex = 0;

  for (int i = 0; i < LOG_FILE_INDEX_MAX; i++)
  {
    // Construct a file with PREFIX[Index].SUFFIX
    filename = String(LOG_FILE_PREFIX);
    filename += String(logIndex);
    filename += ".";
    filename += String(LOG_FILE_SUFFIX);
    // If the file name doesn't exist, return it
    if (!SD.exists(filename))
    {
      return filename;
    }
    // Otherwise increment the index, and try again
    logIndex++;
  }

  return "";
}
// idle function when test is finished
void stop()
{
  while (1);
}
