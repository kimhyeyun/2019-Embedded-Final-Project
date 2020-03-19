#include "includes.h"

#define F_CPU	16000000UL	// CPU frequency = 16 Mhz
#include <avr/io.h>	
#include <avr/interrupt.h>
#include <util/delay.h>

#define CDS_VALUE 871  //ADC 기준값

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
OS_EVENT	  *Mbox[3];		//mail box, 0 : level 전송(level -> led), 1 : 결과값 전송(led -> fnd)
OS_EVENT	  *queue_to_led; //message queue, 스위치 누른 횟수를 led task로 전송

OS_FLAG_GRP   *l_grp;		//event flag , led task 수행 중일 때, level task가 동작하지 않게 하는 flag
OS_FLAG_GRP   *b_grp;		//event flag,  다른 task의 수행이 끝날 때까지, buzzer task를 동작하지 않게 하는 flag 

OS_EVENT	  *Sem;			//semaphore, 공유변수 사용시, critical section 보호를 위해 사용


void * led_queue[4];		//message queue에서 사용하는 배열

/* task prototype */
void  FndTask(void *data);
void  LevelTask(void *data);
void  LedTask(void *data);
void BuzzerTask(void* data);

/* 사용하는 함수들 prototype */
unsigned short read_adc();
void ctrl_level(unsigned short value);
void clean_fnd();

INT8U err; // 에러
volatile int level = 1;			//사용하는 level 1~3
volatile int switch_state = OFF;  //모드 확인, ON : LED task, OFF : Level task
volatile int clicked_switch = OFF; //led task에서 스위치 눌렀는 지 여부 확인
volatile int switch_count = -1;	  //switch 눌리는 횟수, 이 숫자에 따라 led가 켜진다.
volatile int decision_level = OFF;	//level 결정 확인 flag
volatile int buzzer_state = OFF;	//buzzer 상태 
volatile int mel_idx = 0;			//buzzer에서 melody를 나오게 하는 index

const unsigned char melody[25] = { MI,RE,DO,RE,MI,MI,MI,RE,RE,RE,MI,SOL,SOL,MI,RE,DO,RE,MI,MI,MI,RE,RE,MI,RE,DO };	//비행기 노래

/* fnd에 출력할 데이터 정의 */
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

// iterrupt 4(스위치1)
ISR(INT4_vect) {
	OSSemPend(Sem, 0, &err);
	if (switch_state == OFF) { 
		PORTA = 0x00;
	}
	switch_state = ON;
	clicked_switch = ON;	
	// 스위치는 8번까지만 누르기(LED개수가 8개)
	if (switch_count < 8) {
		switch_count++;
		OSQPost(queue_to_led, &switch_count);	//led task로 누른 횟수 전송
	}

	OSSemPost(Sem);
}

// iterrupt 4(스위치2) - 누르면 레벨이 결정됨
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

	/* 레지스터 값들 설정 */
	DDRA = 0xff; //A포트를 LED 출력 포트로 사용
	DDRB = 0x10; //B포트를 버저 출력 포트로 사용
	DDRE = 0xCF; //E포트를 스위치 입력 포트로 사용
	EICRB = 0x0A; //External Interrupt의 trigger 설정에 사용하는 register, 인터럽트 4,5, trigger를 falling edge
	EIMSK = 0x30; //External Interrupt의 개별적인 제어를 담당하는 register, 인터럽트 4,5, enable
	SREG |= 1 << 7; //전체적인 인터럽트 허용 비트를 1로 설정
	
	ADMUX = 0x00;	//5V 기준 전압 사용, 오른쪽 정렬, ADC0사용 설정
	ADCSRA = 0x87;	//ADC를 enable, single conversion 모드, 프리스케일러 128분주 설정

	TCCR2 = 0x03;   //프리스케일러를 32분주로
	TIMSK = 0x00; //overflow 인터럽트를 Disable

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

	/* task 생성 */
	OSTaskCreate(FndTask, (void *)0, (void *)&TaskStk[0][TASK_STK_SIZE - 1], 1);
	OSTaskCreate(BuzzerTask, (void *)0, (void *)&TaskStk[1][TASK_STK_SIZE - 1], 2);
	OSTaskCreate(LevelTask, (void *)0, (void *)&TaskStk[2][TASK_STK_SIZE - 1], 3);
	OSTaskCreate(LedTask, (void *)0, (void *)&TaskStk[3][TASK_STK_SIZE - 1], 4);
	

	OSStart();

	return 0;
}


/* led task - 게임 : 시간 안에 모든 led를 켜면 win, 그렇지 않으면 lose */
void LedTask(void *data)
{
	int on_cnt;		//switch 누른 횟수
	char result;	//게임 결과
	int stop_time;	//종료 시간
	int current_level;	//현재 설정 레벨 
	int time_count = 0;	//시간 체크 
	data = data;

	PORTA = 0x00;	

	current_level = *(int*)(OSMboxPend(Mbox[0], 0, &err));	// level task로 부터 레벨 받아올 때까지 대기 
	while (1) {
		stop_time = DEFAULT_PLAY_TIME - ((current_level - 1) * 50);	//레벨에 따라 종료시간 계산
		time_count++;

		/* 시간 종료 되어 진 경우 */
		if (time_count >= stop_time) {
			OSSemPend(Sem, 0, &err);
			PORTA = 0x00;
			result = 'L';
			OSMboxPost(Mbox[1], (void*)& result);	//L을 fnd task로 전송 
			time_count = 0;	
			OSSemPost(Sem);
		}
		/* 스위치 눌러서 task 수행한 경우 */
		else if (result != 'L' && clicked_switch == ON) {
			on_cnt = *(int*)(OSQPend(queue_to_led, 0, &err));	//스위치 누른 값 받아오기

			clicked_switch = OFF;	

			/* led 전부 켜서 이긴 경우 */
			if (on_cnt == 7) {
				OSSemPend(Sem, 0, &err);
				PORTA = 0xff;
				result = 'W';
				OSMboxPost(Mbox[1], (void*)& result);	//W를 fnd task로 전송
				time_count = 0;
				OSSemPost(Sem);
			}
			/* 아직 게임 중 */
 			else {
				OSSemPend(Sem, 0, &err);
				PORTA += 0x80 >> on_cnt;	//LED 켜기
				OSSemPost(Sem);
			}
		}
		_delay_ms(50);
	}

}

/* fnd task - led task로부터 게임 결과 값 받아와서 출력 */
void FndTask(void *data)
{
	char value; // result 
	int current_level; // level
	int pend_time = 300;	//fnd 출력할 시간

	data = data;

	/* 레지스터 출력포트로 설정 */
	DDRC = 0xff;	//C포트로 fnd 값
	DDRG = 0x0f;	//G포트로 fnd 선택 

	while (1) {
		value = *(char*)(OSMboxPend(Mbox[1], 0, &err));	//led로부터 결과값 받아오기
		/* 이긴 경우 */
		if (value == 'W') {	//WIN 출력
			while (pend_time--) {
				PORTC = FND_DATA[FND_VAL_N];
				PORTG = 0x01; // 1번째 자리
				_delay_ms(2);
				PORTC = FND_DATA[FND_VAL_I];
				PORTG = 0x02; // 2번째 자리
				_delay_ms(2);
				PORTC = FND_DATA[FND_VAL_W2];
				PORTG = 0x04; // 3번째 자리
				_delay_ms(2);
				PORTC = FND_DATA[FND_VAL_W1];
				PORTG = 0x08; // 4번째 자리
				_delay_ms(2);
			}
			OSFlagPost(b_grp, 0xff, OS_FLAG_SET, &err);	//buzzer task 깨우기 
		}
		/* 진 경우 */
		else {
			while (pend_time--) { //LOSE
				PORTC = FND_DATA[FND_VAL_E];
				PORTG = 0x01; // 1번째 자리
				_delay_ms(2);
				PORTC = FND_DATA[FND_VAL_S];
				PORTG = 0x02; // 2번째 자리
				_delay_ms(2);
				PORTC = FND_DATA[FND_VAL_O];
				PORTG = 0x04; // 3번째 자리
				_delay_ms(2);
				PORTC = FND_DATA[FND_VAL_L];
				PORTG = 0x08; // 4번째 자리
				_delay_ms(2);
			}
			OSFlagPost(l_grp, 0xff, OS_FLAG_SET, &err);	//level task 깨우기
			break;	
		}
		
		clean_fnd();
		pend_time = 300;
	
	}
}

/* level task - level 결정하는 task, 광센서를 이용해서 레벨 설정이 가능하다. (레벨은 1 ~3) */
void LevelTask(void *data) {
	unsigned short value;

	data = data;
	while (1) {
		/* 스위치2로 level 결정이 된 경우 */
		if (decision_level == ON) {
			clean_fnd();
			OSMboxPost(Mbox[0], (void*)& level);	//결정된 레벨 led task로 전송 
			OSFlagPend(l_grp, 0xff, OS_FLAG_WAIT_SET_ALL + OS_FLAG_CONSUME, 0, &err);	//led task 수행할 수 있도록 대기 
		}
		value = read_adc();	//광센서 값 받아오기
		ctrl_level(value);	//광센서에 따른 레벨 결정 

		/* fnd로 설정할 레벨을 화면에 보여줌 */
		PORTC = FND_DATA[level];
		PORTG = 0x01; // 1번째 자리
		_delay_ms(2);
		PORTC = FND_DATA[FND_VAL_O];
		PORTG = 0x02; // 2번째 자리
		_delay_ms(2);
		PORTC = FND_DATA[FND_VAL_DOT];
		PORTG = 0x04; // 3번째 자리
		_delay_ms(2);
		PORTC = FND_DATA[FND_VAL_L];
		PORTG = 0x08; // 4번째 자리
		_delay_ms(2);
	}

}

/* buzzer task - win인 경우 비행기 노래가 나온다. */
void BuzzerTask(void* data) {
	data = data;
	
	OSFlagPend(b_grp, 0xff, OS_FLAG_WAIT_SET_ALL, 0, &err);		//다른 task들이 모두 수행을 마칠 때까지 대기 
	TIMSK = 0X40;  //enable/Disable overflow interrupt
	TCNT2 = melody[mel_idx]; //Timer count(on this, means melody)
	
	/* 비행기 노래 출력 */
	while (mel_idx < 25) {
		mel_idx = mel_idx + 1;
		_delay_ms(500);
	}
	OSFlagPost(l_grp, 0xff, OS_FLAG_SET, &err);		//노래 출력 level task 깨우기 
}

/* 조도 센서 관련 함수들 */
unsigned short read_adc()
{
	unsigned char adc_low, adc_high;
	unsigned short value;
	ADCSRA |= 0x40; // ADC start conversion, ADSC = '1'
	while ((ADCSRA & (0x10)) != 0x10); // ADC 변환 완료 검사
	adc_low = ADCL;
	adc_high = ADCH;
	value = (adc_high << 8) | adc_low;

	return value;
}

void ctrl_level(unsigned short value) {	//레벨 결정 함수
	int pend_time = 300;
	int time_led = 0;
	/* 기준값 만큼 어두워지면 */
	if (value < CDS_VALUE) {
		OSSemPend(Sem, 0, &err);
		level++;	//level 증가
		if (level == 4) {
			level = 1;
		}
		_delay_ms(100);
		OSSemPost(Sem);
	}
}

/* fnd 관련 함수 */
void clean_fnd() {
	// fnd 전부 끄기
	PORTC = 0x00;
	PORTG = 0x01; // 1번째 자리
	_delay_ms(2);
	PORTG = 0x02; // 2번째 자리
	_delay_ms(2);
	PORTG = 0x04; // 3번째 자리
	_delay_ms(2);
	PORTG = 0x08; // 4번째 자리
	_delay_ms(2);

	PORTA = 0x00;
}
