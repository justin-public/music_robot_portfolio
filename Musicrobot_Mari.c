

#include <avr/io.h>
#include <avr/interrupt.h>

typedef unsigned char uint8;
typedef unsigned int uint16;

#define ADDR      PINA
#define DEVICE   (PINF >> 4)
   

#define ON  1
#define OFF 0

#define STX     0xff    //   
#define CPP2    0x48    // 프로토콜 두번째 - 고정 값 ,, ComPare Protocol 2 
#define CPP3    DEVICE  // 프로토콜 세번째 - 악기종류 ,, ComPare Protocol 3
#define ETX 	0xf8    //	

#define Servo_RX_BYTE_INTERVAL  delay_u(30)
#define Start_position                3200
#define Servo_Init_torque             1792 // (0x0700 = 1792 ,, min = 0 ,, max = 4095)
#define Servo_Init_speed              2048 // (0x0400 = 1024 ,, min = 0 ,, max = 4095)
#define servo_return_delay            100  //ms
#define servo_init_delay            delay_m(2000)
#define offset -30

uint8
Tx_buffer[8]={0,0,0,0,0,0,0,0},
Rx_buffer[2]={0,0},
servo_return_flag=0,
servo_move_flag[45],
servo_timer_count[45];	
	
int Servo_angle_correct[45];


uint16 
T_count          = 0;

uint8
TX_flag          = 0,
StatusLED_flag   = 0,
Serial_input     = 0;



//---------------------------------------------------------------- 함수원형 선언
void delay_125n(void);
void delay_250n(unsigned char time_ns);
void delay_u (unsigned char time_us);
void delay_m(unsigned int time_ms);
void StatusLED (uint8 status);
void SelSerial(unsigned int ddata);
void tx_send1(unsigned char Tx_data);
void tx_string1(uint8 *str_data, uint16 length);
uint8 SUM ();

//-----------------------------------------------------------------------------





//---------------------------------------------------------------- 초기화 함수
void timer0_init(void) // 타이머 매치 인터럽트 1ms
{
	TCCR0=0x0C; OCR0=249;  TIMSK=2;  //16000000/  64/(1+249)= 1000Hz=1m
}

void port_init(void)   
{	
	DDRA   = 0x00;
	PORTA  = 0x00;
	DDRB   = 0xC0;
	PORTB  = 0x00;
	DDRC   = 0xBF;  // STATUS 출력 LED BIT7
	PORTC  = 0x00;
	DDRD   = 0x78;
	PORTD  = 0x08;
	DDRE   = 0xE2;
	PORTE  = 0x02;
	DDRF   = 0x00;
	DDRG   = 0x00;
	
/*	
//-----------A_PORT
    DDRA  = 0x00;  // A포트는 모두 입력모드
    PORTA = 0x00;  // 초기 입력
//-----------------

//-----------B_PORT
    DDRB  = 0xF0;  // H = | 1 1 1 1 | = | OC1C, OC1B, OC1A, OC10 |, L = | 0 0 0 0 | =
    PORTB = 0x00;  // 초기 입력
//-----------------

//-----------C_PORT
    DDRC  = 0xC0;  // H = | 1,1,0,0 |, L = | 0,0,0,0 | ---- PC0~5 입력, PC6~7 출력
    PORTC = 0x00;  // 초기 입력
//-----------------

//-----------D_PORT
    DDRD  = 0x08;  // H = | 0,0,0,0 | = SW1 입력 , L = | 1,0,0,0 | = | TXD1, RXD1, INT0, INT1 |
    PORTD = 0x00;  // 초기 입력
//-----------------

//-----------E_PORT
    DDRE  = 0x02;  // H = | 0,0,0,0 | = | INT7, INT6, OC3C, OC3B | , L = | 0,0,1,0 | = | OC3A, X, TXD0, RXD1 |
    PORTE = 0x00;  // 초기 입력
//-----------------

//-----------F_PORT
    DDRF  = 0xFF;  // F포트는 모두 출력
    PORTF = 0x00;  // 초기 입력
//-----------------

//-----------G_PORT
    DDRG  = 0xFF;  // G포트는 모두 출력
    PORTG = 0x00;  // 초기 입력
//-----------------
*/
} 
void serial_init(void)
{
	// UART0 Set
	UCSR0A = 0x00;
	UCSR0B = 0x90;
	UCSR0C = 0x06;
	UBRR0H = 0x00;
	UBRR0L = 51;  // 51 : 19200bps  , 8 : 115200bps
	
	// UART1 Set
	UCSR1A = 0x00;
	UCSR1B = 0x08;
	UCSR1C = 0x06;
	UBRR1H = 0x00;
	UBRR1L = 8;   // 115200bps
}

void mari_servo_motor_init(void)
{
	uint16 i,temp;
	SelSerial(3);   // Mari 데이터 송신 채널
	servo_init_delay;    // 숫자가 작을수록 안쪽으로
	Servo_angle_correct[1]=offset+100;
	Servo_angle_correct[2]=offset+20;
	Servo_angle_correct[3]=offset+250;
	Servo_angle_correct[4]=offset+230;
	Servo_angle_correct[5]=offset+50;
	Servo_angle_correct[6]=offset;
	Servo_angle_correct[7]=offset+40;
	Servo_angle_correct[8]=offset+60;
	Servo_angle_correct[9]=offset+270;
	Servo_angle_correct[10]=offset+180;
	Servo_angle_correct[11]=offset+60;
	Servo_angle_correct[12]=offset+200;
	Servo_angle_correct[13]=offset+150;
	Servo_angle_correct[14]=offset+150;
	Servo_angle_correct[15]=offset+220;
	Servo_angle_correct[16]=offset+150;
	Servo_angle_correct[17]=offset+230;
	Servo_angle_correct[18]=offset+175;
	Servo_angle_correct[19]=offset;
	Servo_angle_correct[20]=offset-100;
	Servo_angle_correct[21]=offset+130;
	Servo_angle_correct[22]=offset+270;
	Servo_angle_correct[23]=offset+200;
	Servo_angle_correct[24]=offset+130;
	Servo_angle_correct[25]=offset+120;
	Servo_angle_correct[26]=offset+250;//
	Servo_angle_correct[27]=offset+250;
	Servo_angle_correct[28]=offset+150;
	Servo_angle_correct[29]=offset+120;
	Servo_angle_correct[30]=offset+50;  //공연전 입고된 서보 옵셋
	Servo_angle_correct[31]=offset+100;
	Servo_angle_correct[32]=offset+60;
	Servo_angle_correct[33]=offset+45;
	Servo_angle_correct[34]=offset+150;
	Servo_angle_correct[35]=offset+90;
	Servo_angle_correct[36]=offset;
	Servo_angle_correct[37]=offset+95;   //공연전 입고된 서보 옵셋
	Servo_angle_correct[38]=offset;
	Servo_angle_correct[39]=offset;
	Servo_angle_correct[40]=offset+50;
	Servo_angle_correct[41]=offset+50;
	Servo_angle_correct[42]=offset+50;
	Servo_angle_correct[43]=offset+150;
	Servo_angle_correct[44]=offset+50;
	
	//-----------------------------서보토크초기화
	Tx_buffer[0] = 0x96;  // 서보 프로토콜 헤더
	Tx_buffer[1] = 0x00;  // 서보 ID (0x00 : 모든 서보 호출)
	Tx_buffer[2] = 0x22;  // 서보 토크 레지스터 어드레스
	Tx_buffer[3] = 0x02;  // 레지스터 어드레스 byte 갯수 (토크 레지스터는 2byte)
	Tx_buffer[4] = (uint8)(Servo_Init_torque);  
	Tx_buffer[5] = (uint8)(Servo_Init_torque >> 8);  // 초기 토크셋 (0x0700 = 1792 ,, min = 0 ,, max = 4095)
	Tx_buffer[6] = SUM();
	tx_string1(Tx_buffer,7);
	Servo_RX_BYTE_INTERVAL;

	//------------------------------------
	
	//----------------------------서보속도초기화	
	Tx_buffer[0] = 0x96;  // 서보 프로토콜 헤더
	Tx_buffer[1] = 0x00;  // 서보 ID (0x00 : 모든 서보 호출)
	Tx_buffer[2] = 0x20;  // 서보 이동속도 레지스터 어드레스
	Tx_buffer[3] = 0x02;  // 레지스터 어드레스 byte 갯수 (토크 레지스터는 2byte)
	Tx_buffer[4] = (uint8)(Servo_Init_speed);   
	Tx_buffer[5] = (uint8)(Servo_Init_speed >> 8);  // 초기 속도셋 (0x0400 = 1024 ,, min = 0 ,, max = 4095)
	Tx_buffer[6] = SUM();
	tx_string1(Tx_buffer,7);
	Servo_RX_BYTE_INTERVAL;

	//------------------------------------
	
	//---------------------------서보초기포지션
	for(i = 1 ; i < 45 ; i++)
	{
		Tx_buffer[0] = 0x96;
		Tx_buffer[1] = (uint8)(i);
		Tx_buffer[2] = 0x1E;
		Tx_buffer[3] = 0x02;
		Tx_buffer[4] = (uint8)(Start_position + Servo_angle_correct[i]);
		Tx_buffer[5] = (uint8)((Start_position + Servo_angle_correct[i]) >> 8);
		Tx_buffer[6] = SUM();
		tx_string1(Tx_buffer,7);
		Servo_RX_BYTE_INTERVAL;

	}
	//------------------------------------

}
void system_init(void)  // 초기화 함수 활성화 했는지 항상 확인
{
	port_init();
	timer0_init();
	serial_init();
	mari_servo_motor_init();
	sei();
}

//---------------------------------------------------------------------------




//----------------------------------------------------------------- 딜레이 함수

void delay_125n(void)
{
	cli();    // Global Interrupts

	asm ("NOP");		  // 1 cycles
	asm ("NOP");		  // 1 cycles

	sei();	  // Global Interrupts
}
void delay_250n(unsigned char time_ns)
{
	cli();    // Global Interrupts

	register unsigned char i;

	for(i=0; i<time_ns; i++)  // 3 cycles
	{
	  	asm ("NOP");          // 1 cycles
	}
	sei();	  // Global Interrupts
}
void delay_u (unsigned char time_us)
{
	cli();    // Global Interrupts

	register unsigned char i;

	for(i=0; i<time_us; i++)  // 3 cycles
	{
	  	asm ("NOP");          // 1 cycles
	  	asm ("NOP");          // 1 cycles
	  	asm ("NOP");          // 1 cycles
	  	asm ("NOP");          // 1 cycles
	  	asm ("NOP");          // 1 cycles
	  	asm ("NOP");          // 1 cycles
	  	asm ("NOP");          // 1 cycles
	  	asm ("NOP");          // 1 cycles
	  	asm ("NOP");          // 1 cycles
	  	asm ("NOP");          // 1 cycles
	  	asm ("NOP");          // 1 cycles
	  	asm ("NOP");          // 1 cycles
	  	asm ("NOP");          // 1 cycles = 16Mhz
	}
	sei();	  // Global Interrupts
}
void delay_m(unsigned int time_ms) // 1ms
{
	cli();    // Global Interrupts

	register unsigned int i;
 	for(i=0; i<time_ms; i++)  // 3 cycles = 187.5ns
	{
	  	asm ("NOP");          // 62.5ns
		delay_250n(3);        // 750ns
		delay_u(49);
		delay_u(50);
		
		delay_u(50);
		delay_u(50);

		delay_u(50);
		delay_u(50);

		delay_u(50);
		delay_u(50);

		delay_u(50);
		delay_u(50);

		delay_u(50);
		delay_u(50);

		delay_u(50);
		delay_u(50);

		delay_u(50);
		delay_u(50);

		delay_u(50);
		delay_u(50);

		delay_u(50);
		delay_u(50);          // 100us X 10 = 1ms
	}

}

//----------------------------------------------------------------------------





//----------------------------------------------------------------- 서브 함수


void StatusLED (uint8 status) 
{
	if(status)PORTC |= _BV(7); // 켜짐  
	else PORTC &= ~_BV(7);  //꺼짐
}
void SelSerial(unsigned int ddata)
{
	PORTD &= 0x08;
	switch(ddata)
	{
		case 0:  // Sub Serial 1
			PORTD |= 0x10;
			break;
			
		case 1:  // Sub Serial 2
			PORTD |= 0x20;
			break;
			
		case 2:  // Main RS485 2
			PORTD |= 0x40;
			break;
			
		case 3:  //  Sub Serial 1 and 2
			PORTD |= 0x30;
			break;
			
		default: // Main RS485 2
			PORTD |= 0x40;
			break;
	}		
}
void tx_send1(unsigned char Tx_data)
{
    while((UCSR1A&0x20) == 0x00);
    UDR1=Tx_data;
}
void tx_string1(uint8 *str_data, uint16 length)
{
    while(!(length==0))
    {   
		tx_send1(*str_data);
        str_data++;
		length--;
    }
}
uint8 SUM ()
 {
 	uint16  i;
	uint8 sum = 0;
 	for(i=1;i<(Tx_buffer[3]+4);i++)sum=(Tx_buffer[i]+sum);
	return sum;
 }

//----------------------------------------------------------------------------




//----------------------------------------------------------------- 메인 함수

int main(void)
{
	uint16 i, temp;
	system_init();
    while(1)
    {
		if(TX_flag)
		{			
			Tx_buffer[0] = 0x96;  // 서보 프로토콜 헤더
			Tx_buffer[1] = Rx_buffer[0];  // 서보 ID (0x00 : 모든 서보 호출)
			Tx_buffer[2] = 0x20;  // 서보 이동속도 레지스터 어드레스
			Tx_buffer[3] = 0x02;  // 레지스터 어드레스 byte 갯수 (토크 레지스터는 2byte)
			temp = ((uint16)(Rx_buffer[1] >> 4)& 0x0007) * 512 + 511;
			Tx_buffer[4] = (uint8)(temp);   
			Tx_buffer[5] = (uint8)(temp >> 8);  // 속도셋 (min = 0 ,, max = 4095)
			Tx_buffer[6] = SUM();
			tx_string1(Tx_buffer,7);
			Servo_RX_BYTE_INTERVAL;
					

			Tx_buffer[0] = 0x96;  // 서보 프로토콜 헤더
			Tx_buffer[1] = Rx_buffer[0];  // 서보 ID (0x00 : 모든 서보 호출)
			Tx_buffer[2] = 0x1E;  // 서보 이동 레지스터 어드레스
			Tx_buffer[3] = 0x02;  // 레지스터 어드레스 byte 갯수 (이동 레지스터는 2byte)	


//----------------------------------------------------------------------[10월 15일 변경 시작점]

			if((Rx_buffer[1]&0x0f)==0)
			{
				temp = (unsigned int)(3900 + Servo_angle_correct[Rx_buffer[0]]);
				Tx_buffer[4] = (uint8)(temp);   
				Tx_buffer[5] = (uint8)(temp >> 8);  // 위치셋 (min = 0 ,, max = 4095)
				servo_move_flag[Rx_buffer[0]]=ON;				
			}
			else
			{
				temp = (unsigned int)((Rx_buffer[1]&0x0f)*253 + Servo_angle_correct[Rx_buffer[0]]);
				Tx_buffer[4] = (uint8)(temp);   
				Tx_buffer[5] = (uint8)(temp >> 8);  // 위치셋 (min = 0 ,, max = 4095)				
			}
			
//----------------------------------------------------------------------------------------
/*기존 코드
			temp = (unsigned int)(3800 + Servo_angle_correct[Rx_buffer[0]]);
			Tx_buffer[4] = (uint8)(temp);   
			Tx_buffer[5] = (uint8)(temp >> 8);  // 위치셋 (min = 0 ,, max = 4095)	
			servo_move_flag[Rx_buffer[0]]=ON;	
----------------------------------------------------------------------------------------*/

//---------------------------------------------------------------------[10월 15일 변경 마지막점]		

			
			Tx_buffer[6] = SUM();
			tx_string1(Tx_buffer,7);
			Servo_RX_BYTE_INTERVAL;
			// -----------------------------------	
			
			
			
			
			
			TX_flag=0;					
		}
		
		for(i=1;i<45;i++)
		{
			if(servo_move_flag[i]==2)
			{
				
				Tx_buffer[0] = 0x96;  // 서보 프로토콜 헤더
				Tx_buffer[1] = i;  // 서보 ID (0x00 : 모든 서보 호출)
				Tx_buffer[2] = 0x1E;
				Tx_buffer[3] = 0x02;
				//temp = (Start_position + Servo_angle_correct[i]);
				Tx_buffer[4] = (uint8)(Start_position + Servo_angle_correct[i]);
				Tx_buffer[5] = (uint8)((Start_position + Servo_angle_correct[i]) >> 8);
				Tx_buffer[6] = SUM();
				tx_string1(Tx_buffer,7);
				Servo_RX_BYTE_INTERVAL;
				servo_move_flag[i]=OFF;
			}	
		}
		
	}		
       
}




//----------------------------------------------------------------------------




//-----------------------------------------------------------------------인터럽트


SIGNAL(SIG_OUTPUT_COMPARE0)
{
	uint16 i;
	T_count++;
	for(i=1;i<45;i++)
	{
		if(servo_move_flag[i])
		{
			servo_timer_count[i]++;
			if(servo_timer_count[i]>=servo_return_delay)
			{
				servo_move_flag[i]=2;
				servo_timer_count[i]=0;
			}			
		}		
	}
	/*		
	if(T_count>=1000)
	{
		T_count=0;
		StatusLED_flag=~StatusLED_flag;
		StatusLED(StatusLED_flag);
	}
	*/	
}

SIGNAL(SIG_UART0_RECV)  // switch로 변경??
{
	uint8 rdata=0;
	rdata=UDR0;	
	
	if(Serial_input==5 && rdata==ETX)
	{
		TX_flag=ON;StatusLED_flag=~StatusLED_flag;
		StatusLED(StatusLED_flag);
		Serial_input=0;
	}		
	else if(Serial_input==5)
	{
		Serial_input=0;
		Rx_buffer[0]=0;
		Rx_buffer[1]=0;		
	}	
	if(Serial_input==4 && rdata < 128){Rx_buffer[1]=rdata;Serial_input=5;}  // 속도, 궤적 데이터		
	else if(Serial_input==4)
	{
		Serial_input=0;
		Rx_buffer[0]=0;
		Rx_buffer[1]=0;
	}
	if(Serial_input==3 && rdata < 45 && rdata > 0){Rx_buffer[0]=rdata;Serial_input=4;}  // 서보 모터 어드레스
	else if(Serial_input==3)
	{
		Serial_input=0;
		Rx_buffer[0]=0;
		Rx_buffer[1]=0;			
	}	
	if(Serial_input==2 && rdata==CPP3)Serial_input=3;  // 악기종류
	else if(Serial_input==2)Serial_input=0;	
	if(Serial_input==1 && rdata==CPP2)Serial_input=2;  // ???
	else if(Serial_input==1)Serial_input=0;	
	if(STX==rdata)Serial_input=1;		
}
