This Code is an implementation of MemChar: Low-Power and Low-Cost Arduino-‎Based Characterization Tool for Memristor ‎Devices

Please refer to the paper for circuit and connection guidance, also a Simulation video of MemChar functionality is added to the files. 

The work is publushed in IEEE Transactions on Instrumentation and Measurement. please cite the paper when using the code 

Paper: 10.1109/TIM.2022.3144202


components needed: 

Arduino Mega 2560 Rev3 [23]‎
Adafruit MCP4728 I2C Quad DAC [24]‎
Adafruit ADS1115 16-Bit ADC [25]‎
SparkFun level shifting microSD breakout [26]‎
‎8 x VN10KM N-channel enhanced MOSFET ‎transistors [27]‎
‎2 x 1 MΩ resistors
‎2 x 10 kΩ resistors
‎2 x 100 nF capacitors


Software used: 

Arduino Software (IDE)

Please install the folowing libraries in Arduino IDE before using the code:

SPI.h
SD.h
Wire.h
Adafruit_MCP4728.h
Adafruit_ADS1X15.h
