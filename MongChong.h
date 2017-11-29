// Only modify this file to include
// - function definitions (prototypes)
// - include files
// - extern variable definitions
// In the appropriate section

#ifndef MongChong_H_
#define MongChong_H_
#include "Arduino.h"
//add your includes for the project MongChong here

#define FORWARD 0x09
#define BAKCWARD 0x06
#define LEFT_U 0x0A
#define RIGHT_U 0x05
#define LEFT 0x08
#define RIGHT 0x01
#define STOP 0x00

//end of add your includes here
#ifdef __cplusplus
extern "C" {
#endif
void loop();
void setup();
#ifdef __cplusplus
} // extern "C"
#endif

//add your function definitions for the project UltraInfra here

void ADC_Compare(void);
unsigned char SensorD_read(void);
void SensorA_read(void);
void DAC_CH_Write(unsigned int ch, unsigned int da);
void DAC_setting(unsigned int data);
void infrared_init();
void Motor_mode(int da);
void Motor_Control(char da, unsigned int OC_value);

//Do not add code below this line
#endif /* MongChong_H_ */
