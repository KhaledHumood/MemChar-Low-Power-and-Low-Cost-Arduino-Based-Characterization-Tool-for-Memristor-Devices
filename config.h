// pins initialiing
// reading channels and reisistances
const int Pin_count = 4;
const int Vref[Pin_count] = {8, 4, 19, 22}; //CH1,CH2,CH3,CH4
const float Rref[Pin_count] = {1.014E6, 9.77E3, 0.998E6, 9.89E3}; //Rref1,Rref2,Rref3,Rref4
int CurrentVrefp = 0; // current positive reading channel
float CurrentRrefp = 0; // current positive resistance 
int CurrentVrefn = 0; // current negative reading channel
float CurrentRrefn = 0; // current negative resistance 
// writing channels
const int V1 = 3; // V1 transistor
const int V2 = 26; // V2 transistor
// DAC channels
const int Vplus = MCP4728_CHANNEL_A;
const int vgs1 = MCP4728_CHANNEL_B;
const int Vneg = MCP4728_CHANNEL_C;
const int vgs2 = MCP4728_CHANNEL_D;
// ADC channels
int VMp = 0;
int VMn = 1;
// variable inizialing
int VplusD = 0; // current voltage supply digital value
float VplusA = 0.0; // current voltage supply analog value
int VnegD = 0; // current voltage supply digital value
float VnegA = 0.0; // current voltage supply analog value
int16_t VMpD = 0;  // memristor positive terminal digital value reading
float VMpA = 0; // memristor positive terminal analog value reading
int16_t VMnD = 0;  // memristor negative terminal digital value reading
float VMnA = 0; // memristor negative terminal analog value reading
float VM = 0; // analog voltage accross memristor
unsigned long Start_Time = 0;
int startp = 1; // to set the current reading channel to CH1 under positive bias for the first reading
int startn = 1; // to set the current reading channel to CH3 under negative bias for the first reading
int pointer1 = 0; // Dual reverse positive sweep pointer
int pointer2 = 0; // Dual reverse negative sweep pointer
int Reverse = 0;  // Dual sweep
///////////////////////
// SD Logging Config //
///////////////////////
#define ENABLE_SD_LOGGING true // Default SD logging (can be changed via serial menu)
#define LOG_FILE_INDEX_MAX 999 // Max number of "logXXX.txt" files
#define LOG_FILE_PREFIX "log"  // Prefix name for log files
#define LOG_FILE_SUFFIX "txt"  // Suffix name for log files
#define SD_MAX_FILE_SIZE 5000000 // 5MB max file size, increment to next file before surpassing
#define SD_LOG_WRITE_BUFFER_SIZE 1024 // Experimentally tested to produce 100Hz logs
#define SD_CHIP_SELECT_PIN 53
