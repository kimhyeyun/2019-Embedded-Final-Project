#include "includes.h"

#define F_CPU	16000000UL	// CPU frequency = 16 Mhz
#include <avr/io.h>	
#include <avr/interrupt.h>
#include <util/delay.h>

#define CDS_VALUE 871  //ADC ���ذ�

#define  TASK_STK_SIZE  OS_TASK_DEF_STK_SIZE
#define  N_TASKS     4
#define	 FND_VAL_W1	19
#define	 FND_VAL_W2	20
#define	 FND_VAL_I	1
#define	 FND_VAL_N	21
#define	 FND_VAL_L	22
#define	 FND_VAL_O	0
#define	 FND_VAL_S	5
#define	 FND_VAL_E	23
#define  FND_VAL_DOT 16

#define DO 17
#define RE 43
#define MI 66
#define FA 77
#define SOL 97
#define LA 114
#define TI 129
#define UDO 137

#define ON 1
#define OFF 0

#define DEFAULT_PLAY_TIME 150

OS_STK       TaskStk[N_TASKS][TASK_STK_SIZE];
OS_EVENT	  *Mbox[3];		//mail box, 0 : level ����(level -> led), 1 : ����� ����(led -> fnd)
OS_EVENT	  *queue_to_led; //message queue, ����ġ ���� Ƚ���� led task�� ����

OS_FLAG_GRP   *l_grp;		//event flag , led task ���� ���� ��, level task�� �������� �ʰ� �ϴ� flag
OS_FLAG_GRP   *b_grp;		//event flag,  �ٸ� task�� ������ ���� ������, buzzer task�� �������� �ʰ� �ϴ� flag 

OS_EVENT	  *Sem;			//semaphore, �������� ����, critical section ��ȣ�� ���� ���


void * led_queue[4];		//message queue���� ����ϴ� �迭

/* task prototype */
void  FndTask(void *data);
void  LevelTask(void *data);
void  LedTask(void *data);
void BuzzerTask(void* data);

/* ����ϴ� �Լ��� prototype */
unsigned short read_adc();
void ctrl_level(unsigned short value);
void clean_fnd();

INT8U err; // ����
volatile int level = 1;			//����ϴ� level 1~3
volatile int switch_state = OFF;  //��� Ȯ��, ON : LED task, OFF : Level task
volatile int clicked_switch = OFF; //led task���� ����ġ ������ �� ���� Ȯ��
volatile int switch_count = -1;	  //switch ������ Ƚ��, �� ���ڿ� ���� led�� ������.
volatile int decision_level = OFF;	//level ���� Ȯ�� flag
volatile int buzzer_state = OFF;	//buzzer ���� 
volatile int mel_idx = 0;			//buzzer���� melody�� ������ �ϴ� index

const unsigned char melody[25] = { MI,RE,DO,RE,MI,MI,MI,RE,RE,RE,MI,SOL,SOL,MI,RE,DO,RE,MI,MI,MI,RE,RE,MI,RE,DO };	//����� �뷡

/* fnd�� ����� ������ ���� */
unsigned char FND_DATA[] = {
	0x3f, // 0, O
	0x06, // 1, I
	0x5b, // 2
	0x4f, // 3
	0x66, // 4
	0x6d, // 5, S
	0x7d, // 6
	0x27, // 7
	0x7f, // 8
	0x6f, // 9
	0x77, // A
	0x7c, // B
	0x39, // C
	0x5e, // D
	0x79, // E
	0x71, // F
	0x80, // .
	0x40, // -
	0x08, // _
	0x3c, // W1
	0x1e, // W2
	0x54, // N
	0x38, // L
	0x79  // E
};

//Timer interrupt
ISR(TIMER2_OVF_vect) {
	if (buzzer_state == ON) {
		PORTB = 0x00;
		buzzer_state = OFF;
	}
	else {
		PORTB = 0x10;
		buzzer_state = ON;
	}
	TCNT2 = melody[mel_idx];
}

// iterrupt 4(����ġ1)
ISR(INT4_vect) {
	OSSemPend(Sem, 0, &err);
	if (switch_state == OFF) { 
		PORTA = 0x00;
	}
	switch_state = ON;
	clicked_switch = ON;	
	// ����ġ�� 8�������� ������(LED������ 8��)
	if (switch_count < 8) {
		switch_count++;
		OSQPost(queue_to_led, &switch_count);	//led task�� ���� Ƚ�� ����
	}

	OSSemPost(Sem);
}

// iterrupt 4(����ġ2) - ������ ������ ������
ISR(INT5_vect) {
	OSSemPend(Sem, 0, &err);
	decision_level = ON;
	OSSemPost(Sem);
}

int main(void)
{
	int i;
	OSInit();

	OS_ENTER_CRITICAL();

	/* �������� ���� ���� */
	DDRA = 0xff; //A��Ʈ�� LED ��� ��Ʈ�� ���
	DDRB = 0x10; //B��Ʈ�� ���� ��� ��Ʈ�� ���
	DDRE = 0xCF; //E��Ʈ�� ����ġ �Է� ��Ʈ�� ���
	EICRB = 0x0A; //External Interrupt�� trigger ������ ����ϴ� register, ���ͷ�Ʈ 4,5, trigger�� falling edge
	EIMSK = 0x30; //External Interrupt�� �������� ��� ����ϴ� register, ���ͷ�Ʈ 4,5, enable
	SREG |= 1 << 7; //��ü���� ���ͷ�Ʈ ��� ��Ʈ�� 1�� ����
	
	ADMUX = 0x00;	//5V ���� ���� ���, ������ ����, ADC0��� ����
	ADCSRA = 0x87;	//ADC�� enable, single conversion ���, ���������Ϸ� 128���� ����

	TCCR2 = 0x03;   //���������Ϸ��� 32���ַ�
	TIMSK = 0x00; //overflow ���ͷ�Ʈ�� Disable

	TCNT2 = melody[mel_idx]; //Timer count(on this, means melody)

	/* semaphore, mail box, message queue, event flag create  */
	Sem = OSSemCreate(1);
	for (i = 0; i < 3; i++)
	{
		Mbox[i] = OSMboxCreate((void *)0);
	}
	queue_to_led = OSQCreate(led_queue, 4);
	l_grp = OSFlagCreate(0x00, &err);
	b_grp = OSFlagCreate(0x00, &err);

	/* task ���� */
	OSTaskCreate(FndTask, (void *)0, (void *)&TaskStk[0][TASK_STK_SIZE - 1], 1);
	OSTaskCreate(BuzzerTask, (void *)0, (void *)&TaskStk[1][TASK_STK_SIZE - 1], 2);
	OSTaskCreate(LevelTask, (void *)0, (void *)&TaskStk[2][TASK_STK_SIZE - 1], 3);
	OSTaskCreate(LedTask, (void *)0, (void *)&TaskStk[3][TASK_STK_SIZE - 1], 4);
	

	OSStart();

	return 0;
}


/* led task - ���� : �ð� �ȿ� ��� led�� �Ѹ� win, �׷��� ������ lose */
void LedTask(void *data)
{
	int on_cnt;		//switch ���� Ƚ��
	char result;	//���� ���
	int stop_time;	//���� �ð�
	int current_level;	//���� ���� ���� 
	int time_count = 0;	//�ð� üũ 
	data = data;

	PORTA = 0x00;	

	current_level = *(int*)(OSMboxPend(Mbox[0], 0, &err));	// level task�� ���� ���� �޾ƿ� ������ ��� 
	while (1) {
		stop_time = DEFAULT_PLAY_TIME - ((current_level - 1) * 50);	//������ ���� ����ð� ���
		time_count++;

		/* �ð� ���� �Ǿ� �� ��� */
		if (time_count >= stop_time) {
			OSSemPend(Sem, 0, &err);
			PORTA = 0x00;
			result = 'L';
			OSMboxPost(Mbox[1], (void*)& result);	//L�� fnd task�� ���� 
			time_count = 0;	
			OSSemPost(Sem);
		}
		/* ����ġ ������ task ������ ��� */
		else if (result != 'L' && clicked_switch == ON) {
			on_cnt = *(int*)(OSQPend(queue_to_led, 0, &err));	//����ġ ���� �� �޾ƿ���

			clicked_switch = OFF;	

			/* led ���� �Ѽ� �̱� ��� */
			if (on_cnt == 7) {
				OSSemPend(Sem, 0, &err);
				PORTA = 0xff;
				result = 'W';
				OSMboxPost(Mbox[1], (void*)& result);	//W�� fnd task�� ����
				time_count = 0;
				OSSemPost(Sem);
			}
			/* ���� ���� �� */
 			else {
				OSSemPend(Sem, 0, &err);
				PORTA += 0x80 >> on_cnt;	//LED �ѱ�
				OSSemPost(Sem);
			}
		}
		_delay_ms(50);
	}

}

/* fnd task - led task�κ��� ���� ��� �� �޾ƿͼ� ��� */
void FndTask(void *data)
{
	char value; // result 
	int current_level; // level
	int pend_time = 300;	//fnd ����� �ð�

	data = data;

	/* �������� �����Ʈ�� ���� */
	DDRC = 0xff;	//C��Ʈ�� fnd ��
	DDRG = 0x0f;	//G��Ʈ�� fnd ���� 

	while (1) {
		value = *(char*)(OSMboxPend(Mbox[1], 0, &err));	//led�κ��� ����� �޾ƿ���
		/* �̱� ��� */
		if (value == 'W') {	//WIN ���
			while (pend_time--) {
				PORTC = FND_DATA[FND_VAL_N];
				PORTG = 0x01; // 1��° �ڸ�
				_delay_ms(2);
				PORTC = FND_DATA[FND_VAL_I];
				PORTG = 0x02; // 2��° �ڸ�
				_delay_ms(2);
				PORTC = FND_DATA[FND_VAL_W2];
				PORTG = 0x04; // 3��° �ڸ�
				_delay_ms(2);
				PORTC = FND_DATA[FND_VAL_W1];
				PORTG = 0x08; // 4��° �ڸ�
				_delay_ms(2);
			}
			OSFlagPost(b_grp, 0xff, OS_FLAG_SET, &err);	//buzzer task ����� 
		}
		/* �� ��� */
		else {
			while (pend_time--) { //LOSE
				PORTC = FND_DATA[FND_VAL_E];
				PORTG = 0x01; // 1��° �ڸ�
				_delay_ms(2);
				PORTC = FND_DATA[FND_VAL_S];
				PORTG = 0x02; // 2��° �ڸ�
				_delay_ms(2);
				PORTC = FND_DATA[FND_VAL_O];
				PORTG = 0x04; // 3��° �ڸ�
				_delay_ms(2);
				PORTC = FND_DATA[FND_VAL_L];
				PORTG = 0x08; // 4��° �ڸ�
				_delay_ms(2);
			}
			OSFlagPost(l_grp, 0xff, OS_FLAG_SET, &err);	//level task �����
			break;	
		}
		
		clean_fnd();
		pend_time = 300;
	
	}
}

/* level task - level �����ϴ� task, �������� �̿��ؼ� ���� ������ �����ϴ�. (������ 1 ~3) */
void LevelTask(void *data) {
	unsigned short value;

	data = data;
	while (1) {
		/* ����ġ2�� level ������ �� ��� */
		if (decision_level == ON) {
			clean_fnd();
			OSMboxPost(Mbox[0], (void*)& level);	//������ ���� led task�� ���� 
			OSFlagPend(l_grp, 0xff, OS_FLAG_WAIT_SET_ALL + OS_FLAG_CONSUME, 0, &err);	//led task ������ �� �ֵ��� ��� 
		}
		value = read_adc();	//������ �� �޾ƿ���
		ctrl_level(value);	//�������� ���� ���� ���� 

		/* fnd�� ������ ������ ȭ�鿡 ������ */
		PORTC = FND_DATA[level];
		PORTG = 0x01; // 1��° �ڸ�
		_delay_ms(2);
		PORTC = FND_DATA[FND_VAL_O];
		PORTG = 0x02; // 2��° �ڸ�
		_delay_ms(2);
		PORTC = FND_DATA[FND_VAL_DOT];
		PORTG = 0x04; // 3��° �ڸ�
		_delay_ms(2);
		PORTC = FND_DATA[FND_VAL_L];
		PORTG = 0x08; // 4��° �ڸ�
		_delay_ms(2);
	}

}

/* buzzer task - win�� ��� ����� �뷡�� ���´�. */
void BuzzerTask(void* data) {
	data = data;
	
	OSFlagPend(b_grp, 0xff, OS_FLAG_WAIT_SET_ALL, 0, &err);		//�ٸ� task���� ��� ������ ��ĥ ������ ��� 
	TIMSK = 0X40;  //enable/Disable overflow interrupt
	TCNT2 = melody[mel_idx]; //Timer count(on this, means melody)
	
	/* ����� �뷡 ��� */
	while (mel_idx < 25) {
		mel_idx = mel_idx + 1;
		_delay_ms(500);
	}
	OSFlagPost(l_grp, 0xff, OS_FLAG_SET, &err);		//�뷡 ��� level task ����� 
}

/* ���� ���� ���� �Լ��� */
unsigned short read_adc()
{
	unsigned char adc_low, adc_high;
	unsigned short value;
	ADCSRA |= 0x40; // ADC start conversion, ADSC = '1'
	while ((ADCSRA & (0x10)) != 0x10); // ADC ��ȯ �Ϸ� �˻�
	adc_low = ADCL;
	adc_high = ADCH;
	value = (adc_high << 8) | adc_low;

	return value;
}

void ctrl_level(unsigned short value) {	//���� ���� �Լ�
	int pend_time = 300;
	int time_led = 0;
	/* ���ذ� ��ŭ ��ο����� */
	if (value < CDS_VALUE) {
		OSSemPend(Sem, 0, &err);
		level++;	//level ����
		if (level == 4) {
			level = 1;
		}
		_delay_ms(100);
		OSSemPost(Sem);
	}
}

/* fnd ���� �Լ� */
void clean_fnd() {
	// fnd ���� ����
	PORTC = 0x00;
	PORTG = 0x01; // 1��° �ڸ�
	_delay_ms(2);
	PORTG = 0x02; // 2��° �ڸ�
	_delay_ms(2);
	PORTG = 0x04; // 3��° �ڸ�
	_delay_ms(2);
	PORTG = 0x08; // 4��° �ڸ�
	_delay_ms(2);

	PORTA = 0x00;
}
