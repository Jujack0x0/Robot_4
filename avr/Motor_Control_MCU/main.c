
#include "mcu_init.h"
#include "dataType.h"


//////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define dt_current 0.0005  // 0.5ms
#define dt_velocity 0.005  // 5ms
#define dt_position 0.05   // 50ms

#define Kcp 2.827        // 전류제어기 P gain	(La*wcc) => wcc=2pi*fcc
#define Kci 2.2117e+03    // 전류제어기 I gain	(Ra/La)*Kp
#define Kac 1.209         // 전류제어기 anti wind up (1/Kcp)

#define Kvp 1.8				//  속도제어기 P gain	(J*wcv)/Kt => Trial and error로 구함
#define Kvi 50.0			//  속도제어기 I gain (B/J)*Kp => 공식을 이용해 구한 게인 값을 넣으면 과한 진동
//	I Gain이 커지면 정상상태 오차를 줄일 수 있지만, 진동수가 커져 시스템이 불안정해진다.
#define Kas 0.664         //  속도제어기 anti wind up (1/Kvp)

#define Kpp 12.5664       //  위치제어기 P gain	Kpp=wcp=wcs/10=4pi
#define Kpd 15          //  위치제어기 D gain	Kpd=wcp/wcs=1/10

#define Kt 0.0683         //  역기전력 상수

volatile int32_t g_Cnt, g_preCnt;

volatile double g_Pdes = 0.;
volatile double g_Pcur=0., g_Pre_Pcur=0.;
volatile double g_Perr=0.;
volatile double g_Perr_old = 0.;
volatile double g_Perr_det = 0.;

volatile double g_Vcur=0.;
volatile double g_Vdes = 0.0;
volatile double g_Verr=0.;
volatile double g_Vlimit;
volatile double g_Verr_sum=0.;


volatile double g_Ccur=0.;
volatile double g_Cdes=0.;
volatile double g_Cerr=0.;
volatile double g_Cerr_sum=0.;
volatile double g_Climit;

volatile double g_ADC;
volatile int g_SendFlag = 0;
volatile int g_Direction;

volatile int g_cur_control = 0;
volatile double g_vel_control=0.;
volatile double g_pos_control=0.;
volatile unsigned char g_TimerCnt;

volatile unsigned char checkSize;
volatile unsigned char g_buf[256], g_BufWriteCnt, g_BufReadCnt;

volatile Packet_t g_PacketBuffer;
volatile unsigned char g_PacketMode;
volatile unsigned char g_ID = 1;

#define DEG2RAD 0.0174533
#define RAD2DEG 57.2958

int over_check = 0;

//// SetDuty 설정 ////
void SetDutyCW(double v){
	
	while(TCNT1  == 0);

	uint16_t ocr = v * (200. / 24.) + 200;

	if(ocr > OCR_MAX)   ocr = OCR_MAX;
	else if(ocr < OCR_MIN)   ocr = OCR_MIN;
	//OCR1A = OCR1B = ocr;
	
	OCR1A = OCR3B = ocr + 8;      //1 H
	OCR1B = OCR3A = ocr - 8;      //1 L
}

//// LS7366  ////
void InitLS7366(){
	
	PORTB = 0x00;
	SPI_MasterSend(SELECT_MDR0 | WR_REG);
	SPI_MasterSend(X4_QUAD | FREE_RUN | DISABLE_INDEX | SYNCHRONOUS_INDEX |FILTER_CDF_1);
	PORTB = 0x01;
	
	PORTB = 0x00;
	SPI_MasterSend(SELECT_MDR1 | WR_REG);
	SPI_MasterSend(FOUR_BYTE_COUNT_MODE | ENABLE_COUNTING);
	PORTB = 0x01;
	
	PORTB = 0x00;
	SPI_MasterSend(SELECT_CNTR | CLR_REG);
	PORTB = 0x01;
}


//// ADC 설정 ////
int getADC(char ch){

	ADMUX = (ADMUX & 0xf0) + ch;
	ADCSRA |= 0x40;
	while(!(ADCSRA & 0x10));
	return ADC;
}


ISR(USART0_RX_vect){

	g_buf[g_BufWriteCnt++] = UDR0;
}



ISR(TIMER0_OVF_vect){
	
	TCNT0 = 256 - 125;
	
	//Read LS7366
	int32_t cnt;
	
	PORTC = 0x01;
	
	g_ADC = getADC(0);
	
	PORTB = 0x00;
	SPI_MasterSend(SELECT_OTR | LOAD_REG);
	PORTB = 0x01;
	
	PORTB = 0x00;
	SPI_MasterSend(SELECT_OTR | RD_REG);
	cnt = SPI_MasterRecv();      cnt = cnt<< 8;
	cnt |= SPI_MasterRecv();   cnt = cnt<< 8;
	cnt |= SPI_MasterRecv();   cnt = cnt<< 8;
	cnt |= SPI_MasterRecv();
	PORTB = 0x01;
	g_Cnt = -cnt;      //누적 pulse 값 반화
	
	PORTC = 0x03;
	
	g_Pcur = (double)(g_Cnt / (4096. * 81.)) * 2 * M_PI;   // Unit : [rad] 엔코더값
	
	//TO DO
	//위치제어기 0.05
	
	if((g_TimerCnt % 100) == 0){
		g_TimerCnt = 0;
		
		g_Perr = g_Pdes - g_Pcur;
		
		g_Perr_det = (g_Perr - g_Perr_old)/dt_position;
		g_Perr_old = g_Perr;
		
		// P gain, D Gain
		g_pos_control = g_Perr * Kpp + g_Perr_det * Kpd;
		
		
		// 속도 제한
		if(g_pos_control >= g_Vlimit)
		{
			g_pos_control = g_Vlimit;
		}
		else if(g_pos_control <= (-1) * g_Vlimit)
		{
			g_pos_control = (-1) * g_Vlimit;
		}
		if(g_Vlimit >= 6140.0 * 2.0 * M_PI / 60.0)
		{
			g_Vlimit = 6140.0 * 2.0 * M_PI / 60.0;
		}
		else if(g_Vlimit <= (-1) * 6140.0 * 2.0 * M_PI / 60.0)
		{
			g_Vlimit = (-1) * 6140.0 * 2.0 * M_PI / 60.0;
		}
		
	}
	
	//속도제어기 
	if((g_TimerCnt % 10) == 0){
		
		g_Vcur = (double)(g_Pcur - g_Pre_Pcur) / 0.005;
		g_Pre_Pcur = (double)g_Pcur;
		
		g_Vdes = g_pos_control;
		
		
		g_Verr = (double)(g_Vdes - g_Vcur);
		g_Verr_sum += (double)g_Verr;
		
		// P Gain, I Gain, D Gain
		// I Gain은 오차가 누적되는 값에 곱해준다.
		g_vel_control = (double)(g_Verr * Kvp + g_Verr_sum * Kvi );
		
		
		// 속도 제어기 Anti-windup
		if(g_vel_control >= g_Climit)
		{
			g_Verr_sum -= (double)((g_vel_control - g_Climit)*Kas);
			g_vel_control = g_Climit;
		}
		else if(g_vel_control <= -g_Climit)
		{
			g_Verr_sum -= (double)((g_vel_control + g_Climit)*Kas);
			g_vel_control = -g_Climit;
		}
		if(g_Climit >= 27.3)
		{
			g_Climit = 27.3;
		}
		else if(g_Climit <= -27.3)
		{
			g_Climit = -27.3;
		}
	}
	
	g_TimerCnt++;
	
	//전류제어기 0.0005
	g_Cdes = g_vel_control;     //속도 제어기 출력을 전류제어에 입력
	
	g_Ccur = -( ((g_ADC / 1024. * 5.) - 2.5) * 10.);
	g_Cerr = (double)(g_Cdes - g_Ccur);
	
	g_Cerr_sum += (double)g_Cerr;      // 누적 오차
	
	// P Gain, I Gain
	g_cur_control = (double)(g_Cerr * Kcp + g_Cerr_sum * Kci* dt_current);
	
	g_cur_control += (double)(g_Vcur * Kt);
	
	// 전류 제어기 Anti (-24[V] ~ 24[V])
	if(g_cur_control >= 24)
	{
		g_Cerr_sum -= (double)(g_cur_control - 24)*1. / 0.0827 / 3.;
		g_cur_control = 24.;
	}
	else if(g_cur_control <= -24)
	{
		g_Cerr_sum -= (double)(g_cur_control + 24)*1. / 0.0827 / 3.;
		g_cur_control = -24.;
	}
	
	
	SetDutyCW(g_cur_control);
	
	
	/////////////////////////////////////////
	
	g_SendFlag++;

}



int main(void){
	
	Packet_t packet;
	packet.data.header[0] = packet.data.header[1] = packet.data.header[2] = packet.data.header[3] = 0xFE;
	
	InitIO();
	
	//Uart
	InitUart0();
	
	//SPI
	InitSPI();
	
	//Timer
	InitTimer0();
	InitTimer1();
	InitTimer3();


	TCNT1 = TCNT3 = 0;
	SetDutyCW(0.);
	
	//ADC
	InitADC();
	
	//LS7366
	InitLS7366();
	
	TCNT0 = 256 - 125;
	sei();

	unsigned char check = 0;
	
	//// Packet 통신 설정 ////
	while (1) {
		for(;g_BufReadCnt != g_BufWriteCnt; g_BufReadCnt++){
			
			switch(g_PacketMode){
				case 0:
				
				if (g_buf[g_BufReadCnt] == 0xFF) {
					checkSize++;
					if (checkSize == 4) {
						g_PacketMode = 1;
					}
				}
				else {
					checkSize = 0;
				}
				break;
				
				case 1:

				g_PacketBuffer.buffer[checkSize++] = g_buf[g_BufReadCnt];
				
				if (checkSize == 8) {
					if(g_PacketBuffer.data.id == g_ID){

						g_PacketMode = 2;
					}
					else{
						SetDutyCW(0);
						
						g_PacketMode = 0;
						checkSize = 0;
					}
				}

				break;

				case 2:
				
				g_PacketBuffer.buffer[checkSize++] = g_buf[g_BufReadCnt];
				check += g_buf[g_BufReadCnt];
				
				if (checkSize == g_PacketBuffer.data.size) {

					if(check == g_PacketBuffer.data.check){

						switch(g_PacketBuffer.data.mode){

							case 2:
							g_Pdes = g_PacketBuffer.data.pos / 1000.;
							g_Pdes += (360*DEG2RAD) * over_check;
							g_Vlimit = g_PacketBuffer.data.velo / 1000.;
							g_Climit = g_PacketBuffer.data.cur / 1000.;
							break;
						}
					}
					
					check = 0;
					g_PacketMode = 0;
					checkSize = 0;
				}
				else if(checkSize > g_PacketBuffer.data.size || checkSize > sizeof(Packet_t)) {
					TransUart0('f');
					check = 0;
					g_PacketMode = 0;
					checkSize = 0;
				}
			}
		}

		if(g_SendFlag > 19){
			g_SendFlag = 0;
			
			packet.data.id = g_ID;
			packet.data.size = sizeof(Packet_data_t);
			packet.data.mode = 3;
			packet.data.check = 0;
			
			/*   ODE로 현재 모터 값 전송    */
			packet.data.pos = g_Pcur * 1000;
			packet.data.velo = g_Vcur * 1000;
			packet.data.cur = g_Ccur * 1000;
			
			for (int i = 8; i < sizeof(Packet_t); i++)
			packet.data.check += packet.buffer[i];
			
			
			for(int i=0; i<packet.data.size; i++){
				TransUart0(packet.buffer[i]);
			}
			
		}
	}
	
}