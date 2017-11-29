#include "MongChong.h"
// Do not remove the include below

/* 과목명 : 스마트 소프트웨어 프로젝트
 * 제출일 : 2017.06.16 금요일
 * 이름 : 1415003 강예지, 1415067 정도연
 * 과제명 : 
*/

int Motor[6] = {22, 23, 24, 25, 4, 5}; //모터를 사용하는 데에 이용하는 핀번호들
int move_vel = 70; //직진할 때 스마트카의 속도
int rot_vel = 200; //회전할 때 스마트카의 속도

//적외선 센서를 위한 변수들
int S_DIN = 42, S_SCLK = 43, S_SYNCN = 44;
int IN_SEN_EN = 26;
int SensorA[8] = {A0, A1, A2, A3, A4, A5, A6, A7};
int SensorD[8] = {30, 31, 32, 33, 34, 35, 36, 37}; //적외선 센서 사용을 위한 핀 번호 및 변수들 설정
unsigned int Buff_A[9] = {0,0,0,0,0,0,0,0};
unsigned int ADC_MAX[8] = {0,0,0,0,0,0,0,0};
unsigned int ADC_MIN[8] = {1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023};
unsigned int ADC_MEAN[8] = {0,0,0,0,0,0,0,0};
unsigned char Sensor_data = 0;
int DAC_data = 0;
int direction_data1 = 0; //방향 변환을 위해 사용하는 방향값

int count = 0; //블럭의 번호 변수
bool check = false; //실시간 count의 값을 늘려야하는 상태인지 
bool precheck = false;// check 상태 비교를 위해 기존의 check값을 저장해 놓은 변수

//초음파 센서를 위한 변수들
char RX_buf[17];
unsigned char RX_buf2[7];
unsigned char TX_buf1[5] = {0x76, 0x00, 0xAF, 0xE0, 0x8F};  //초음파 센서 열기 - 좌측, 전방, 우측 센서 사용
unsigned char TX_buf2[5] = {0x76, 0x00, 0x0F, 0x00, 0x0F}; //초음파 센싱 중지
int data=0; //각 초음파 센서별 장애물 여부 저장
/* RX_(방향)_flag 변수 값든 장애물이 15센치 밑에 장애물이 감지되었을 때 값이 1로 변한다.
 * (방향)_flag 변수들은 RX_(방향)_flag 값이 4가 되어 확실히 장애물이 감지되었다고 판단하는 변수로, 실시간 각 방향별 장애물 여부 판단 변수들이다.
 * (방향)_flag의 값을 실시간 장애물 판단 변수로 한 칸 내에서 값이 계속해서 변하므로 따로 한칸의 최종 정보를 저장할 변수가 필요하다.
 * block_(방향)_flag 변수들은 주차장 한 칸 별로 최종 상태를 저장하는 변수들이다.
 * block_(방향)_flag 변수는  다음 칸을 셀 때 0으로 리셋한다.*/
int RX_Left_flag=1;
int Left_flag=0; //실시간 왼쪽 장애물 flag
int block_Left_flag = 0; //한 칸의 최종 왼쪽 장애물 flag

int RX_Right_flag=1;
int Right_flag=0; //실시간 오른쪽 장애물 flag
int block_Right_flag = 0; //한 칸의 최종 오른쪽 장애물 flag

int RX_Front_flag=0;
int Front_flag=0; //실시간 전방 장애물 flag
int block_Front_flag=0; //한 칸의 최종 전방 장애물 flag 

//The setup function is called once at startup of the sketch
void setup()
{
	// Add your initialization code here
	int z;
	pinMode(IN_SEN_EN, OUTPUT);
	pinMode(S_DIN, OUTPUT);
	pinMode(S_SYNCN, OUTPUT);
	pinMode(S_SCLK, OUTPUT);
	digitalWrite(S_SCLK, LOW);
	digitalWrite(S_SYNCN, HIGH);
	digitalWrite(IN_SEN_EN, HIGH);

	for(z=0; z<6; z++){
		pinMode(Motor[z], OUTPUT);
		digitalWrite(Motor[z], LOW);
	}

	for(z=0; z<8; z++){
		pinMode(SensorD[z], INPUT);
	}
	Serial.begin(115200); //어플리케이션과 블루투스 통신 시작 
	DAC_setting(0x9000);
	for(z=0; z<8; z++){
		DAC_CH_Write(z, 255);
	}
	infrared_init();
	Serial1.begin(115200); //초음파 센싱 시작 
	Serial1.write(TX_buf1,5); //초음파 센싱 시작 
}

// The loop function is called in an endless loop
void loop()
{
	//Add your repeated code here
	int speed = 0, direction_data = 0;
	Sensor_data = SensorD_read();
	unsigned int Sensor_input = (~Sensor_data)&0xFF;

	//스마트 카는 라인 트레이싱을 기본으로 한다. 
	switch(Sensor_input){
	case 0x18: //00011000
	case 0x10: //00010000
	case 0x08: //00001000
	case 0x38: //00111000
	case 0x1C: //00011100
	case 0x3C: //00111100
		direction_data = FORWARD;
		speed = move_vel;
		break;

	case 0x0C: //00001100
	case 0x04: //00000100
	case 0x06: //00000110
	case 0x0E: //00001110
	case 0x1E: //00011110
	case 0x0F: //00001111
		direction_data = RIGHT;
		speed = rot_vel;
		break;

	case 0x30: //00110000
	case 0x20: //00100000
	case 0x60: //01100000
	case 0x70: //01110000
	case 0x78: //01111000
	case 0xF0: //11110000
		direction_data = LEFT;
		speed = rot_vel;
		break;

	case 0x07: //00000111
	case 0x03: //00000011
	case 0x02: //00000010
	case 0x01: //00000001
		direction_data = RIGHT_U;
		speed = rot_vel;
		break;

	case 0xC0: //11000000
	case 0x40: //01000000
	case 0x80: //10000000
	case 0xE0: //11100000
		direction_data = LEFT_U;
		speed = rot_vel;
		break;
		

 	//오른쪽에 라인이 있는 경우
	case 0x1F: //00011111
	case 0x3F: //00111111
	case 0x7F: //01111111
		if(block_Front_flag){ //전방에 잘못주차된 차량이 있다면
			switch(count){ //블럭의 위치에 따라 알고리즘이 다르므로, 1번과 4번, 2번과 5번을 각각 묶는다.
			case 1:
			case 4:
			//1번과 4번에 주차된 차량의 경우에는 왼쪽으로 전방의 차량을 회피한후 오른쪽 선을 만나면 항상 우회전을 해주어야 한다. 
			//다음은 하드웨어적인 오류를 보정하여 시스템의 정확도를 높이기 위해 다음과 같은 코드로 보정을 해주었다. 
				direction_data = RIGHT;
				speed = rot_vel;
				Motor_Control('A', speed);
				Motor_mode(direction_data);
				delay(600);
				direction_data = RIGHT_U;
				speed = rot_vel;
				Motor_Control('A', speed);
				Motor_mode(direction_data);
				delay(200);
				break;
			case 2:
			case 5:
			//2번과 5번에 주차된 차량의 경우에는 회피 후, 오른쪽에 라인이 있음에도 불구하고 직진을 하여야 한다. 
				direction_data = FORWARD;
				speed = move_vel;
				Motor_Control('A', speed);
				Motor_mode(direction_data);
				delay(200);
				block_Front_flag = 0; //2번 5번에 잘못 주차된 차량이 있다고 판단된 상태에서 직진을 했다면 그 칸이 끝난다는 이야기이므로 block_Front_flag를 0으로 바꿔준다. 
			}
		}
		else{ 
			// 기본적으로 전방에 잘못 주차된 차량이 없다면 오른쪽으로 회전한다. 
			direction_data = RIGHT_U;
			speed = rot_vel;
			Motor_Control('A', speed);
			Motor_mode(direction_data);
			delay(200);
		}

		break;

	//한 줄을 만나는 경우
	case 0xFF: //11111111 
		if(block_Front_flag){ //전방에 잘못 주차된 차량이 있는 경우
			switch(count){ //블럭의 위치에 따라 알고리즘이 다르므로, 1번과 4번, 2번과 5번을 각각 묶는다.
			case 1:
			case 4:
			//1번과 4번에 주차된 차량의 경우에는 왼쪽으로 전방의 차량을 회피한후 오른쪽 선을 두번 만나 두번다 우회전을 한 후, 한줄(1111 1111)을 만난다면 좌회전을 해주어야 한다. 
				direction_data = LEFT_U;
				speed = rot_vel;
				Motor_Control('A', speed);
				Motor_mode(direction_data);
				delay(500);
				block_Front_flag = 0;
				break;
			case 2:
			case 5:
			//2번과 5번에 주차된 차량의 경우에는 전방의 장애물을 피해 왼쪽으로 회피한 다음, 한 줄(1111 1111)을 만나는 경우 우회전을 해주어야 한다. 
			//다음은 하드웨어적인 오류를 보정하여 시스템의 정확도를 높이기 위해 다음과 같은 코드로 보정을 해주었다. 
				direction_data = RIGHT;
				speed = rot_vel;
				Motor_Control('A', speed);
				Motor_mode(direction_data);
				delay(600);
				direction_data = RIGHT_U;
				speed = rot_vel;
				Motor_Control('A', speed);
				Motor_mode(direction_data);
				delay(200);
				break;
			}
		}
		else{	// 기본적으로 전방에 잘못 주차된 차량이 없다면 오른쪽으로 회전한다. 

			direction_data = RIGHT;
			speed = rot_vel;
		}
		break;

	case 0: //00000000
		direction_data = RIGHT_U;
		speed = rot_vel;
		break;

	default:
		direction_data = FORWARD;
		speed = move_vel;
		break;
	}
	
	if(direction_data1 != direction_data){
		direction_data1 = direction_data;
		Motor_Control('A', speed);
		Motor_mode(direction_data);
	}

	//카운트를 해도 되는지 확인
	if((Sensor_input & 0xFC) == 0xFC){ //1111 1100
		check = true;
	}
	else check = false;

	//상태 변화가 있나 확인
	if(precheck != check){
		if(check == true) {
			if(block_Front_flag){ //앞에 장애물이 있는 경우는 회피를 하고, 한 블럭의 상태를 출력하지 않도록 설정

			}
			else{ //앞에 장애물이 없는 경우에 좌우의 장애물 여부를 확인 후 한 블럭의 상태를 어플리케이션에 출력
				if(block_Left_flag){ //좌측에 장애물이 있는 경우
					Serial.println(count);
				}
				else{ //좌측에 장애물이 없는 경우
					Serial.println(count+50);
				}
				delay(50);

				if(block_Right_flag){ //우측에 장애물이 있는 경우
					Serial.println(count+10);
				}
				else{ //우측에 장애물이 없는 경우
					Serial.println(count+60);
				}
				delay(50);

				count++;
				count = count%6;
			}
			block_Left_flag = 0;
			block_Right_flag = 0;
		}
		precheck = check; //check값이 변하면 변한 값을 precheck에 저장하여 다음 checkd의 변화를 판단
	}

	if(Front_flag){
		block_Front_flag = Front_flag;
		Serial.println(count+20);

		Motor_Control('A', speed);
		Motor_mode(STOP);
		delay(100);

		direction_data = LEFT_U; //전방에 장애물이 있을 경우, 피하기 위해 왼쪽으로 돌아 왼쪽에 선을 찾아 라인 트레이싱을 진행한다
		speed = rot_vel;
		Motor_Control('A', speed);
		Motor_mode(direction_data);
		delay(600);

		Serial1.write(TX_buf1,5);
		Front_flag = 0;
	}

	if(Left_flag){  //장애물이 연속으로 4번 찍히면 왼쪽에 차량(올바르게 주차된 차량)이라고 판단 
		block_Left_flag = Left_flag; // 한 블럭 내에 올바르게 주차된 차량이 있음을 저장 
		Serial1.write(TX_buf1,5);
		Left_flag = 0;
	}

	if(Right_flag){  //오른쪽 초음파 센서에 4번 역속 센싱이 되면 오른쪽에 차량(불법 주차 차량)이라고 판
		block_Right_flag = Right_flag; //한 블럭 내에 불법 주차 차량이 있음을 저장 
		Serial1.write(TX_buf1,5);
		Right_flag = 0;
	}
}

//적외선 센싱을 위한 함수 
void ADC_Compare(void){
	int z;
	for(z=0; z<8; z++){
		if(ADC_MAX[z] < Buff_A[z])
			ADC_MAX[z] = Buff_A[z];
		if(ADC_MIN[z] > Buff_A[z])
			ADC_MIN[z] = Buff_A[z];
	}
}

void SensorA_read(void){
	int z;
	for(z=0; z<8; z++){
		Buff_A[z] = analogRead(SensorA[z]);
	}
}

void DAC_CH_Write(unsigned int ch, unsigned int da){
	unsigned int data = ((ch<<12)&0x7000) | ((da<<4)&0x0FF0);
	DAC_setting(data);
}

void DAC_setting(unsigned int data){
	int z;
	digitalWrite(S_SCLK, HIGH);
	delayMicroseconds(1);
	digitalWrite(S_SCLK, LOW);
	delayMicroseconds(1);
	digitalWrite(S_SYNCN, LOW);
	delayMicroseconds(1);

	for(z=16; z>0; z--){
		digitalWrite(S_DIN, (data>>(z-1))&0x1);
		digitalWrite(S_SCLK, HIGH);
		delayMicroseconds(1);
		digitalWrite(S_SCLK, LOW);
		delayMicroseconds(1);
	}
	digitalWrite(S_SYNCN, HIGH);
}

void infrared_init(){

	int z, error = 0;
	Serial.println("infrared init Start");
	Motor_Control('A', 160);
	while(1){
		Motor_mode(LEFT_U);
		for(z=0; z<50; z++){
			SensorA_read();
			ADC_Compare();
			delay(8);
		}
		Motor_mode(RIGHT_U);
		for(z=0; z<100; z++){
			SensorA_read();
			ADC_Compare();
			delay(8);
		}

		Motor_mode(LEFT_U);
		delay(600);
		Motor_mode(STOP);
		delay(1000);

		for(z=0; z<8; z++){
			if((ADC_MAX[z] - ADC_MIN[z] < 200)){
				error++;
			}
		}
		if(error == 0){
			Serial.println("\n\rinfrared init END");
			break;
		}
		else{
			error = 0;
			Serial.println("\n\rinfrared init Restart");
			for(z=0; z<8; z++){
				ADC_MAX[z] = 0;
				ADC_MIN[z] = 1023;
			}
		}
	}

	for(z=0; z<8; z++){
		ADC_MEAN[z] = (ADC_MAX[z] + ADC_MIN[z])/2;
		DAC_CH_Write(z, ADC_MEAN[z]/4);
	}
}

//자동차 움직임을 제어하는 함수 
void Motor_mode(int da){
	int z;
	for(z=0; z<4; z++){
		digitalWrite(Motor[z], (da>>z)&0x01);
	}
}

void Motor_Control(char da, unsigned int OC_value){
	switch(da){
	case 'L':
		analogWrite(Motor[4], OC_value);
		break;
	case 'R':
		analogWrite(Motor[5], OC_value);
		break;
	case 'A':
		analogWrite(Motor[4], OC_value);
		analogWrite(Motor[5], OC_value);
		break;
	}
}

unsigned char SensorD_read(void){
	unsigned char data = 0, z;
	for(z=0; z<8; z++){
		data>>=1;
		if(digitalRead(SensorD[z])) data |= 0x80;
	}
	return data;
}

//초음파 센싱을 위한 함수 
void serialEvent1(){
	unsigned char z,tmp=0;
	Serial1.readBytes(RX_buf,17); //reading 17 bytes in
	if((RX_buf[0] == 0x76) && (RX_buf[1] == 0))
	{
		for(z=2;z<16;z++)
			tmp += (unsigned char)RX_buf[z];
		tmp = tmp & 0xFF;
		if((unsigned char)RX_buf[16] == tmp) // No error
		{
			for(z=0;z<7;z++) // front sensor sensing values
			{
				RX_buf2[z]=(unsigned char)RX_buf[z+4];
			}
			data = 0;

			for(z=0;z<7;z++){
				if(RX_buf2[z]<15)
					data = data | (0x01<<z);
			}
			if((data & 0x03)!=0){  //왼쪽에 장애물이 있으면(0000 0011)
				if(RX_Left_flag == 4){
					Left_flag = 1;
					Serial1.write(TX_buf2, 5);
					RX_Left_flag=0;
				}
				else if(RX_Left_flag!=4){
					RX_Left_flag++;
				}
			}

			if((data & 0x60)!=0){  //오른쪽에 장애물이 있으면(0110 0000)
				if(RX_Right_flag == 4){
					Right_flag = 1;
					Serial1.write(TX_buf2, 5);
					RX_Right_flag=0;
				}
				else if(RX_Right_flag!=4){
					RX_Right_flag++;
				}
			}
			
			if((data & 0x1C)!=0){  //전방에 장애물이 있으면(0001 1100)
				if(RX_Front_flag == 4){
					Front_flag = 1;
					Serial1.write(TX_buf2, 5);
					RX_Front_flag=0;
				}
				else if(RX_Right_flag!=4){
					RX_Front_flag++;
				}
			}
		}
	}
	else {
		for(z=1;z<17;z++){
			if(RX_buf[z]==0x76){
				if(z!=16){
					if(RX_buf[z+1]==0){
						tmp = z;
						break;
					}
				}
				else
					tmp = z;
			}
		}
		Serial1.readBytes(RX_buf,tmp);
	}
}
