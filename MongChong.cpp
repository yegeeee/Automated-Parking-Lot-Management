#include "MongChong.h"
// Do not remove the include below

/* ����� : ����Ʈ ����Ʈ���� ������Ʈ
 * ������ : 2017.06.16 �ݿ���
 * �̸� : 1415003 ������, 1415067 ������
 * ������ : 
*/

int Motor[6] = {22, 23, 24, 25, 4, 5}; //���͸� ����ϴ� ���� �̿��ϴ� �ɹ�ȣ��
int move_vel = 70; //������ �� ����Ʈī�� �ӵ�
int rot_vel = 200; //ȸ���� �� ����Ʈī�� �ӵ�

//���ܼ� ������ ���� ������
int S_DIN = 42, S_SCLK = 43, S_SYNCN = 44;
int IN_SEN_EN = 26;
int SensorA[8] = {A0, A1, A2, A3, A4, A5, A6, A7};
int SensorD[8] = {30, 31, 32, 33, 34, 35, 36, 37}; //���ܼ� ���� ����� ���� �� ��ȣ �� ������ ����
unsigned int Buff_A[9] = {0,0,0,0,0,0,0,0};
unsigned int ADC_MAX[8] = {0,0,0,0,0,0,0,0};
unsigned int ADC_MIN[8] = {1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023};
unsigned int ADC_MEAN[8] = {0,0,0,0,0,0,0,0};
unsigned char Sensor_data = 0;
int DAC_data = 0;
int direction_data1 = 0; //���� ��ȯ�� ���� ����ϴ� ���Ⱚ

int count = 0; //���� ��ȣ ����
bool check = false; //�ǽð� count�� ���� �÷����ϴ� �������� 
bool precheck = false;// check ���� �񱳸� ���� ������ check���� ������ ���� ����

//������ ������ ���� ������
char RX_buf[17];
unsigned char RX_buf2[7];
unsigned char TX_buf1[5] = {0x76, 0x00, 0xAF, 0xE0, 0x8F};  //������ ���� ���� - ����, ����, ���� ���� ���
unsigned char TX_buf2[5] = {0x76, 0x00, 0x0F, 0x00, 0x0F}; //������ ���� ����
int data=0; //�� ������ ������ ��ֹ� ���� ����
/* RX_(����)_flag ���� ���� ��ֹ��� 15��ġ �ؿ� ��ֹ��� �����Ǿ��� �� ���� 1�� ���Ѵ�.
 * (����)_flag �������� RX_(����)_flag ���� 4�� �Ǿ� Ȯ���� ��ֹ��� �����Ǿ��ٰ� �Ǵ��ϴ� ������, �ǽð� �� ���⺰ ��ֹ� ���� �Ǵ� �������̴�.
 * (����)_flag�� ���� �ǽð� ��ֹ� �Ǵ� ������ �� ĭ ������ ���� ����ؼ� ���ϹǷ� ���� ��ĭ�� ���� ������ ������ ������ �ʿ��ϴ�.
 * block_(����)_flag �������� ������ �� ĭ ���� ���� ���¸� �����ϴ� �������̴�.
 * block_(����)_flag ������  ���� ĭ�� �� �� 0���� �����Ѵ�.*/
int RX_Left_flag=1;
int Left_flag=0; //�ǽð� ���� ��ֹ� flag
int block_Left_flag = 0; //�� ĭ�� ���� ���� ��ֹ� flag

int RX_Right_flag=1;
int Right_flag=0; //�ǽð� ������ ��ֹ� flag
int block_Right_flag = 0; //�� ĭ�� ���� ������ ��ֹ� flag

int RX_Front_flag=0;
int Front_flag=0; //�ǽð� ���� ��ֹ� flag
int block_Front_flag=0; //�� ĭ�� ���� ���� ��ֹ� flag 

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
	Serial.begin(115200); //���ø����̼ǰ� ������� ��� ���� 
	DAC_setting(0x9000);
	for(z=0; z<8; z++){
		DAC_CH_Write(z, 255);
	}
	infrared_init();
	Serial1.begin(115200); //������ ���� ���� 
	Serial1.write(TX_buf1,5); //������ ���� ���� 
}

// The loop function is called in an endless loop
void loop()
{
	//Add your repeated code here
	int speed = 0, direction_data = 0;
	Sensor_data = SensorD_read();
	unsigned int Sensor_input = (~Sensor_data)&0xFF;

	//����Ʈ ī�� ���� Ʈ���̽��� �⺻���� �Ѵ�. 
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
		

 	//�����ʿ� ������ �ִ� ���
	case 0x1F: //00011111
	case 0x3F: //00111111
	case 0x7F: //01111111
		if(block_Front_flag){ //���濡 �߸������� ������ �ִٸ�
			switch(count){ //���� ��ġ�� ���� �˰����� �ٸ��Ƿ�, 1���� 4��, 2���� 5���� ���� ���´�.
			case 1:
			case 4:
			//1���� 4���� ������ ������ ��쿡�� �������� ������ ������ ȸ������ ������ ���� ������ �׻� ��ȸ���� ���־�� �Ѵ�. 
			//������ �ϵ�������� ������ �����Ͽ� �ý����� ��Ȯ���� ���̱� ���� ������ ���� �ڵ�� ������ ���־���. 
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
			//2���� 5���� ������ ������ ��쿡�� ȸ�� ��, �����ʿ� ������ �������� �ұ��ϰ� ������ �Ͽ��� �Ѵ�. 
				direction_data = FORWARD;
				speed = move_vel;
				Motor_Control('A', speed);
				Motor_mode(direction_data);
				delay(200);
				block_Front_flag = 0; //2�� 5���� �߸� ������ ������ �ִٰ� �Ǵܵ� ���¿��� ������ �ߴٸ� �� ĭ�� �����ٴ� �̾߱��̹Ƿ� block_Front_flag�� 0���� �ٲ��ش�. 
			}
		}
		else{ 
			// �⺻������ ���濡 �߸� ������ ������ ���ٸ� ���������� ȸ���Ѵ�. 
			direction_data = RIGHT_U;
			speed = rot_vel;
			Motor_Control('A', speed);
			Motor_mode(direction_data);
			delay(200);
		}

		break;

	//�� ���� ������ ���
	case 0xFF: //11111111 
		if(block_Front_flag){ //���濡 �߸� ������ ������ �ִ� ���
			switch(count){ //���� ��ġ�� ���� �˰����� �ٸ��Ƿ�, 1���� 4��, 2���� 5���� ���� ���´�.
			case 1:
			case 4:
			//1���� 4���� ������ ������ ��쿡�� �������� ������ ������ ȸ������ ������ ���� �ι� ���� �ι��� ��ȸ���� �� ��, ����(1111 1111)�� �����ٸ� ��ȸ���� ���־�� �Ѵ�. 
				direction_data = LEFT_U;
				speed = rot_vel;
				Motor_Control('A', speed);
				Motor_mode(direction_data);
				delay(500);
				block_Front_flag = 0;
				break;
			case 2:
			case 5:
			//2���� 5���� ������ ������ ��쿡�� ������ ��ֹ��� ���� �������� ȸ���� ����, �� ��(1111 1111)�� ������ ��� ��ȸ���� ���־�� �Ѵ�. 
			//������ �ϵ�������� ������ �����Ͽ� �ý����� ��Ȯ���� ���̱� ���� ������ ���� �ڵ�� ������ ���־���. 
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
		else{	// �⺻������ ���濡 �߸� ������ ������ ���ٸ� ���������� ȸ���Ѵ�. 

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

	//ī��Ʈ�� �ص� �Ǵ��� Ȯ��
	if((Sensor_input & 0xFC) == 0xFC){ //1111 1100
		check = true;
	}
	else check = false;

	//���� ��ȭ�� �ֳ� Ȯ��
	if(precheck != check){
		if(check == true) {
			if(block_Front_flag){ //�տ� ��ֹ��� �ִ� ���� ȸ�Ǹ� �ϰ�, �� ���� ���¸� ������� �ʵ��� ����

			}
			else{ //�տ� ��ֹ��� ���� ��쿡 �¿��� ��ֹ� ���θ� Ȯ�� �� �� ���� ���¸� ���ø����̼ǿ� ���
				if(block_Left_flag){ //������ ��ֹ��� �ִ� ���
					Serial.println(count);
				}
				else{ //������ ��ֹ��� ���� ���
					Serial.println(count+50);
				}
				delay(50);

				if(block_Right_flag){ //������ ��ֹ��� �ִ� ���
					Serial.println(count+10);
				}
				else{ //������ ��ֹ��� ���� ���
					Serial.println(count+60);
				}
				delay(50);

				count++;
				count = count%6;
			}
			block_Left_flag = 0;
			block_Right_flag = 0;
		}
		precheck = check; //check���� ���ϸ� ���� ���� precheck�� �����Ͽ� ���� checkd�� ��ȭ�� �Ǵ�
	}

	if(Front_flag){
		block_Front_flag = Front_flag;
		Serial.println(count+20);

		Motor_Control('A', speed);
		Motor_mode(STOP);
		delay(100);

		direction_data = LEFT_U; //���濡 ��ֹ��� ���� ���, ���ϱ� ���� �������� ���� ���ʿ� ���� ã�� ���� Ʈ���̽��� �����Ѵ�
		speed = rot_vel;
		Motor_Control('A', speed);
		Motor_mode(direction_data);
		delay(600);

		Serial1.write(TX_buf1,5);
		Front_flag = 0;
	}

	if(Left_flag){  //��ֹ��� �������� 4�� ������ ���ʿ� ����(�ùٸ��� ������ ����)�̶�� �Ǵ� 
		block_Left_flag = Left_flag; // �� �� ���� �ùٸ��� ������ ������ ������ ���� 
		Serial1.write(TX_buf1,5);
		Left_flag = 0;
	}

	if(Right_flag){  //������ ������ ������ 4�� ���� ������ �Ǹ� �����ʿ� ����(�ҹ� ���� ����)�̶�� ��
		block_Right_flag = Right_flag; //�� �� ���� �ҹ� ���� ������ ������ ���� 
		Serial1.write(TX_buf1,5);
		Right_flag = 0;
	}
}

//���ܼ� ������ ���� �Լ� 
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

//�ڵ��� �������� �����ϴ� �Լ� 
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

//������ ������ ���� �Լ� 
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
			if((data & 0x03)!=0){  //���ʿ� ��ֹ��� ������(0000 0011)
				if(RX_Left_flag == 4){
					Left_flag = 1;
					Serial1.write(TX_buf2, 5);
					RX_Left_flag=0;
				}
				else if(RX_Left_flag!=4){
					RX_Left_flag++;
				}
			}

			if((data & 0x60)!=0){  //�����ʿ� ��ֹ��� ������(0110 0000)
				if(RX_Right_flag == 4){
					Right_flag = 1;
					Serial1.write(TX_buf2, 5);
					RX_Right_flag=0;
				}
				else if(RX_Right_flag!=4){
					RX_Right_flag++;
				}
			}
			
			if((data & 0x1C)!=0){  //���濡 ��ֹ��� ������(0001 1100)
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
