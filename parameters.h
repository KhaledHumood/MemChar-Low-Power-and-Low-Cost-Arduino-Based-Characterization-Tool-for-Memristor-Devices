int mode = 4;
// please choose the test: 
// 1- positive sweep
// 2- negative sweep
// 3- SET-RESET (positive sweep then negative sweep)
// 4- DC bias
// 5- read pulses

///////////////////////////////  common parameters for all modes ////////////////////////
bool measuredV = true; // true report measured voltage, false report programmed voltage
int vgs = 819*5; // to control positive compliance current;
int dual = 1; // 0 no dual sweep 1 for dual sweep
int vgsNeg = 4095; // to control compliance current on negative bias;
int cycles = 1; // for sweeps modes only (1,2,3)

///////////////////////////////  positive sweep mode parameters ////////////////////////
// Use  Voltage(digital) = floor[Voltage(analog)*819] to convert voltage
// analog values to digital numbers writtin to the pin (12 bit dac); 
int startvoltage1 = 0;
int endvoltage1 = 819*3.2;
int stepvoltage1 =  82; // multiple of (0.00122 V)
int readvoltage1 = 409; // equivilant to 0.5 V
///////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////  negative sweep mode parameters ////////////////////////
// Use  Voltage(digital) = floor[Voltage(analog)*819] to convert voltage
// analog values to digital numbers writtin to the pin (12 bit dac); 
int startvoltage2 = 0;
int endvoltage2 = 819*3.2;
int stepvoltage2 =  82; // multiple of (0.00122 V)
int readvoltage2 = 409; // equivilant to 0.5 V
///////////////////////////////////////////////////////////////////////////////////////


/////////////////////////////// DC bias parameters //////////////////////////
// this test will apply a constant DC bias voltage with reading pulses 
// Use  Voltage(digital) = floor[Voltage(analog)*819] to convert voltage
// analog values to digital numbers writtin to the pin (12 bit dac); 
int dcbias = 819; // the applied DC voltage 
int readvoltage4 = 409; // equivilant to 0.5 V
float interval4 = 5; //apply a reading every x sec
int samples4 = 900; // number of reading samples (control the duration of the test) 
float hold4 = 0.0; // hold time before the test in x sec
//////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////// Read Pulses parameters  //////////////////////////
// this test will apply a train of reading pulses 
// Use  Voltage(digital) = floor[Voltage(analog)*819] to convert voltage
// analog values to digital numbers writtin to the pin (12 bit dac); 
int readvoltage5 = 205; // equivilant to 0.25 V
float interval5 = 30; //apply a reading every x sec
int samples5 = 128; // number of reading pulses (control the duration of the test) 
float width5 = 0; // width of the reading pulse in x sec 
float hold5 = 0.0; // hold time before the test in x sec
/////////////////////////////////////////////////////////////////////////////////////////
