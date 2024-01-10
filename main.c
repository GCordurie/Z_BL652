#include "nrf.h"
#include "math.h"

/*--------------------------------------------------------------------------*/
/*									Code de test de la carte Tshirt V07_06e									*/
/*--------------------------------------------------------------------------*/
/*																																					*/
/*									Mapping des entrees sorties utilisees										*/
/*																																					*/
/*																																					*/
/*				LED					:	P0_21				output																	*/
/*																																					*/
/*				VBat				:	P0_04				analog input														*/
/*				TEMP0				:	P0_03				analog input														*/
/*				TEMP1				:	P0_02				analog input														*/
/*				Mes_Zth			:	P0_29				analog input														*/
/*				Respi_0			:	P0_31				analog input														*/
/*				Respi_1			:	P0_30				analog input														*/
/*				ECG_in			:	P0_05				analog input														*/
/*																																					*/
/*			Accélérometre LIS2DH12, Composant en SPI slave (uC en master)				*/
/*				SDI_Accel		:	P0_20				output -> MOSI du SPI										*/
/*				SDO_Accel		:	P0_22				input	 -> MISO du SPI										*/
/*				CS_Accel		:	P0_24				output																	*/
/*				CK_Accel		:	P0_17				output																	*/
/*				INT1				:	P0_13				input																		*/
/*																																					*/
/*				SDin_DAC		:	P0_18				output																	*/
/*				CK_DAC			:	P0_16				output																	*/
/*				nSync_DAC		:	P0_26				output																	*/
/*																																					*/
/*				Select_SW		:	P0_28				output																	*/
/*				SCK_SW			:	P0_11				output																	*/
/*				SData_SW		:	P0_19				output																	*/
/*				nSync_SW		:	P0_23				output																	*/
/*																																					*/
/*																																					*/
/*			Mémoire flash de 1Gbits (128 Moctets)																*/
/*				SDI_flash		:	P0_25				output																	*/
/*				SDO_flash		:	P0_15				input																		*/
/*				CK_flash		:	P0_27				output																	*/
/*				nCS_flash		:	P0_06				output																	*/
/*				nRST_flash	:	P0_12				output																	*/
/*																																					*/
/*				Cmde_off		:	P0_07				output																	*/
/*				En_SW				:	P0_08				output																	*/
/*				nChg				:	P0_09				input																		*/
/*				Gain				:	P0_10				output																	*/
/*				Qi_TS				:	P0_14				IO																			*/
/*																																					*/
/*																																					*/
/*--------------------------------------------------------------------------*/
/*																																					*/
/*																																					*/
/*																																					*/
/*--------------------------------------------------------------------------*/



/*--------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------*/

#define N_Led 21		


volatile unsigned char Flag_Timer = 0;

volatile unsigned int ADC_Result;
volatile unsigned char Flag_ADC = 0;

volatile uint16_t ADC_result[40];

volatile uint16_t Stimu[16];
volatile uint8_t index_sin = 0;

volatile unsigned char TX_data[10];

/*--------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------*/

void Prepare_DAC(uint16_t Val);
void Write_DAC(uint16_t Val);

/*--------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------*/

void Init_CPU(void)
{
	NRF_NFCT->TASKS_DISABLE = 1;
	NRF_UICR->NFCPINS = 0xfffffffe;						//Disable protection of NFC pins

	
//config carte Z_Proto
//Init LED
//Vu	
	NRF_P0->DIR |= 1<<N_Led;					//Init port pour led

//Init_DAC
//Vu
	NRF_P0->DIR |= 1<<16;				//SCK_DAC			:	P0_16				output																	
	NRF_P0->DIR |= 1<<18;				//SData_DAC		:	P0_18				output																	
	NRF_P0->DIR |= 1<<26;				//nSync_DAC		:	P0_26				output	

//Init stim_direct
	NRF_P0->DIR |= 1<<14;				//BLE_stim		:	P0_14				output																	
	
//Init Timer1
	NRF_TIMER1->TASKS_CLEAR = 1;
	
	NRF_TIMER1->MODE = 0x0000;				//Timer	mode
	NRF_TIMER1->SHORTS = 0x0001;			//Shortcut entre compare0 et clear
	NRF_TIMER1->CC[0] = 16;						//Si capture compare 0 mis a 50 -> 20 kHz
																		//Si capture compare 0 mis a 2500 -> 400 Hz
																		
	NRF_TIMER1->INTENSET = 1<<16;			//Enable IT sur compare0;
	
	NRF_TIMER1->BITMODE = 0x0000;			//16 bits mode
	NRF_TIMER1->PRESCALER = 0x04;			//Prescaler = 16 -> 1MHz
	
	NVIC_EnableIRQ(TIMER1_IRQn);			//Valide IT Timer
	//NVIC_SetPriority(TIMER1_IRQn,1);
	
	//NRF_TIMER1->TASKS_STOP = 0;
	NRF_TIMER1->TASKS_START = 1;			//Demarre Timer1

	
//Init ADC
//Vux
	NRF_SAADC->ENABLE = 0;								//Disable ADC
	
	NRF_SAADC->CH[0].PSELP = 4;						//Connect CH0p to AIN3
	NRF_SAADC->CH[0].CONFIG = (1 << 8);		//Gain de 1/5, single ended, 3us de tacq, Ref interne de 0,6V
	
	NRF_SAADC->RESOLUTION = 1;			//Resolution a 10 bits
	NRF_SAADC->SAMPLERATE = 0;			//Sample controle par Sample_task
	NRF_SAADC->INTENSET = 4;				//Valide IT sur DONE
	NRF_SAADC->RESULT.MAXCNT = 1;
	NRF_SAADC->RESULT.PTR = (uint32_t)(&ADC_Result);
	
	NVIC_EnableIRQ(SAADC_IRQn);			//Valide IT SAADC
	
	NRF_SAADC->ENABLE = 1;					//Enable ADC
	NRF_SAADC->TASKS_START = 1;
	
//Init UART
	NRF_UARTE0->PSEL.TXD = 6;				//TX sur PO.6
	NRF_UARTE0->PSEL.RXD = 8;				//RX sur PO.5
	NRF_UARTE0->BAUDRATE = UART_BAUDRATE_BAUDRATE_Baud115200;	//0x00275000UL;
	NRF_UARTE0->CONFIG = 0;						//Pas de parite ni de flow control
	NRF_UARTE0->TXD.PTR = (uint32_t)(&TX_data);
	NRF_UARTE0->TXD.MAXCNT = 3;
	
	TX_data[0] = 0x47;
	TX_data[1] = 0x43;
	TX_data[2] = 0x6f;
	
	NRF_UART0->TASKS_STARTTX = 1;				//Start transmission UART0
	
	NRF_UARTE0->ENABLE = 8;						//Enable UART
}

/*--------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------*/

void TIMER1_IRQHandler(void)
{			
	NRF_TIMER1->EVENTS_COMPARE[0] = 0;	//Clear event compare 0
	NRF_P0->OUT ^= 1<<14;
	NRF_SAADC->TASKS_SAMPLE = 1;
	
	Flag_Timer = 1;
}

/*--------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------*/

void SAADC_IRQHandler(void)
{
	NRF_SAADC->EVENTS_DONE = 0;		//Clear event done
	
	Flag_ADC = 1;
}

/*--------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------*/

void Init_data(void)
{
	unsigned int i;
	uint16_t temp;
	float f_temp;
	
//Init de la sinusoide de stimulation pour Zth	
	for(i=0;i<16;i++)
	{
		f_temp = 1 + sin(i*6.283185/16);
		f_temp = f_temp * 2046;
		temp = (uint16_t)(f_temp);
		temp &= 0x0fff;
		Stimu[i] = temp<<2;
	}
}
/*--------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------*/

void Delais(void)
{
	unsigned int i,j;
	for(i=0;i<10000;i++) for(j=0;j<1000;j++);
}

/*--------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------*/

void Send_ADC(void)
{
	TX_data[0] = (unsigned char)(ADC_Result>>8);
	TX_data[1] = (unsigned char)(ADC_Result & 0x00ff);
	
	NRF_UARTE0->TXD.MAXCNT = 2;
	NRF_UART0->TASKS_STARTTX = 1;				//Start transmission UART0
	
	Flag_ADC = 0;
}

/*--------------------------------------------------------------------------*/
//			SCK_DAC			:	P0_16				output																	
//			SData_DAC		:	P0_18				output																	
//			nSync_DAC		:	P0_26				output
/*--------------------------------------------------------------------------*/
//Complet pour l'instant, a spiltter a terme
inline void Prepare_DAC(uint16_t Val)
{
	uint8_t i;
	uint16_t V_loc;
	uint16_t mask = 0x8000;
	
	V_loc = Val & 0x3ffc;
/*	
	V_loc = V_loc<<2;
	
//Deja fait a priori	
	NRF_P0->OUT |= (1<<16);			// 1 -> SCK_SW			
	NRF_P0->OUT |= (1<<26);			// 1 -> nSync_SW		
*/
	
	NRF_P0->OUT &= ~(1<<26);		// 0 -> nSync_SW		
	
	for(i=0;i<16;i++)
	{
		if(V_loc & mask) NRF_P0->OUT |= (1<<18); else NRF_P0->OUT &= ~(1<<18);
		NRF_P0->OUT &= ~(1<<16);	// 0 -> SCK_SW	
		mask = mask>>1;
		NRF_P0->OUT |= (1<<16);		// 1 -> SCK_SW				
	}
	
	NRF_P0->OUT |= (1<<26);			// 1 -> nSync_SW			
}

/*--------------------------------------------------------------------------*/
//			SCK_DAC			:	P0_16				output																	
//			SData_DAC		:	P0_18				output																	
//			nSync_DAC		:	P0_26				output
/*--------------------------------------------------------------------------*/
//Version depliee
inline void Write_DAC(uint16_t Val)
{
	uint16_t V_loc = Val & 0x3ffc;
	uint16_t mask = 0x8000;
	
	NRF_P0->OUT &= ~(1<<26);		// 0 -> nSync_SW		
	
	if(V_loc & mask) NRF_P0->OUT |= (1<<18); else NRF_P0->OUT &= ~(1<<18);
	NRF_P0->OUT &= ~(1<<16);	// 0 -> SCK_SW	
	mask = 0x4000;
	NRF_P0->OUT |= (1<<16);		// 1 -> SCK_SW				
	
	if(V_loc & mask) NRF_P0->OUT |= (1<<18); else NRF_P0->OUT &= ~(1<<18);
	NRF_P0->OUT &= ~(1<<16);	// 0 -> SCK_SW	
	mask = 0x2000;
	NRF_P0->OUT |= (1<<16);		// 1 -> SCK_SW				
	
	if(V_loc & mask) NRF_P0->OUT |= (1<<18); else NRF_P0->OUT &= ~(1<<18);
	NRF_P0->OUT &= ~(1<<16);	// 0 -> SCK_SW	
	mask = 0x1000;
	NRF_P0->OUT |= (1<<16);		// 1 -> SCK_SW				
	
	if(V_loc & mask) NRF_P0->OUT |= (1<<18); else NRF_P0->OUT &= ~(1<<18);
	NRF_P0->OUT &= ~(1<<16);	// 0 -> SCK_SW	
	mask = 0x800;
	NRF_P0->OUT |= (1<<16);		// 1 -> SCK_SW				
	
	if(V_loc & mask) NRF_P0->OUT |= (1<<18); else NRF_P0->OUT &= ~(1<<18);
	NRF_P0->OUT &= ~(1<<16);	// 0 -> SCK_SW	
	mask = 0x400;
	NRF_P0->OUT |= (1<<16);		// 1 -> SCK_SW				
	
	if(V_loc & mask) NRF_P0->OUT |= (1<<18); else NRF_P0->OUT &= ~(1<<18);
	NRF_P0->OUT &= ~(1<<16);	// 0 -> SCK_SW	
	mask = 0x200;
	NRF_P0->OUT |= (1<<16);		// 1 -> SCK_SW				
	
	if(V_loc & mask) NRF_P0->OUT |= (1<<18); else NRF_P0->OUT &= ~(1<<18);
	NRF_P0->OUT &= ~(1<<16);	// 0 -> SCK_SW	
	mask = 0x100;
	NRF_P0->OUT |= (1<<16);		// 1 -> SCK_SW				
	
	if(V_loc & mask) NRF_P0->OUT |= (1<<18); else NRF_P0->OUT &= ~(1<<18);
	NRF_P0->OUT &= ~(1<<16);	// 0 -> SCK_SW	
	mask = 0x80;
	NRF_P0->OUT |= (1<<16);		// 1 -> SCK_SW				
	
	if(V_loc & mask) NRF_P0->OUT |= (1<<18); else NRF_P0->OUT &= ~(1<<18);
	NRF_P0->OUT &= ~(1<<16);	// 0 -> SCK_SW	
	mask = 0x40;
	NRF_P0->OUT |= (1<<16);		// 1 -> SCK_SW				
	if(V_loc & mask) NRF_P0->OUT |= (1<<18); else NRF_P0->OUT &= ~(1<<18);
	NRF_P0->OUT &= ~(1<<16);	// 0 -> SCK_SW	
	mask = 0x20;
	NRF_P0->OUT |= (1<<16);		// 1 -> SCK_SW				
	
	if(V_loc & mask) NRF_P0->OUT |= (1<<18); else NRF_P0->OUT &= ~(1<<18);
	NRF_P0->OUT &= ~(1<<16);	// 0 -> SCK_SW	
	mask = 0x10;
	NRF_P0->OUT |= (1<<16);		// 1 -> SCK_SW				
	
	if(V_loc & mask) NRF_P0->OUT |= (1<<18); else NRF_P0->OUT &= ~(1<<18);
	NRF_P0->OUT &= ~(1<<16);	// 0 -> SCK_SW	
	mask = 0x08;
	NRF_P0->OUT |= (1<<16);		// 1 -> SCK_SW				
	
	if(V_loc & mask) NRF_P0->OUT |= (1<<18); else NRF_P0->OUT &= ~(1<<18);
	NRF_P0->OUT &= ~(1<<16);	// 0 -> SCK_SW	
	mask = 0x04;
	NRF_P0->OUT |= (1<<16);		// 1 -> SCK_SW				
	
	if(V_loc & mask) NRF_P0->OUT |= (1<<18); else NRF_P0->OUT &= ~(1<<18);
	NRF_P0->OUT &= ~(1<<16);	// 0 -> SCK_SW	
	mask = 0x02;
	NRF_P0->OUT |= (1<<16);		// 1 -> SCK_SW				
	
	if(V_loc & mask) NRF_P0->OUT |= (1<<18); else NRF_P0->OUT &= ~(1<<18);
	NRF_P0->OUT &= ~(1<<16);	// 0 -> SCK_SW	
	mask = 0x1;
	NRF_P0->OUT |= (1<<16);		// 1 -> SCK_SW				
	
	if(V_loc & mask) NRF_P0->OUT |= (1<<18); else NRF_P0->OUT &= ~(1<<18);
	NRF_P0->OUT &= ~(1<<16);	// 0 -> SCK_SW	
	mask = 0x0;
	NRF_P0->OUT |= (1<<16);		// 1 -> SCK_SW				
	
	NRF_P0->OUT |= (1<<26);			// 1 -> nSync_SW			
}

/*--------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------*/

void main(void)
{
	uint16_t Cnt_IT = 0;
		
	Delais();
	
	Init_CPU();
	
	NRF_P0->OUT |= (1<<N_Led);
	
	Init_data();
	Prepare_DAC(0x800);
	
	while(1)
	{
		
		while(Flag_Timer == 0);
		//Prepare_DAC(Stimu[index_sin]);
		Write_DAC(Stimu[index_sin]);
		index_sin = (index_sin + 1)%16;
		Flag_Timer = 0;
		
		/*
		Cnt_IT++;
		Prepare_DAC(Stimu[Cnt_IT%20]);
		
		if(Cnt_IT == 2000) 
		{
			Cnt_IT = 0;
			NRF_P0->OUT ^= 1<<N_Led;
			NRF_UART0->TASKS_STARTTX = 1;				//Start transmission UART0
		}
		*/
	}
}

/*--------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------*/


