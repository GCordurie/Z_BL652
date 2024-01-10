#include "nrf.h"
#include "math.h"

/*--------------------------------------------------------------------------*/
/*									Code de test de la carte Tshirt V07_06									*/
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

volatile unsigned char index_SPI = 0;
volatile unsigned int Val_SPI = 0;
volatile unsigned int Stimu[20];
volatile unsigned char Flag_SPI = 0;

volatile unsigned char Buf_send_Accelero[3];
volatile unsigned char Buf_receive_Accelero[10];

volatile unsigned char Buf_send_Flash[200];
volatile unsigned char Buf_receive_Flash[200];
volatile unsigned char Flag_SPI1 = 0;

volatile unsigned char TX_data[100];

volatile unsigned char index_led = 0;

volatile unsigned int ADC_Result;
volatile unsigned char Flag_ADC = 0;

volatile unsigned char Toggle_ADC = 0;

/*--------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------*/

void Init_CPU(void)
{
	NRF_NFCT->TASKS_DISABLE = 1;
	NRF_UICR->NFCPINS = 0xfffffffe;						//Disable protection of NFC pins

	
//config carte Corps Europrotect
//Init LED	
	NRF_P0->DIR |= 1<<N_Led;					//Init port pour led

//Translation de niveau vers mux
	NRF_P0->DIR |= 1<<28;				//Select_SW		:	P0_28				output
	NRF_P0->DIR |= 1<<11;				//SCK_SW			:	P0_11				output																	
	NRF_P0->DIR |= 1<<19;				//SData_SW		:	P0_19				output																	
	NRF_P0->DIR |= 1<<23;				//nSync_SW		:	P0_23				output	
		
	NRF_P0->OUT &= ~(1<<28);		// 0 -> Select_SW		
	NRF_P0->OUT &= ~(1<<11);		// 0 -> SCK_SW			
	NRF_P0->OUT &= ~(1<<19);		// 0 -> SData_SW		
	NRF_P0->OUT &= ~(1<<23);		// 0 -> nSync_SW		
	
//Bits de control
	NRF_P0->DIR |= 1<<7;				//Cmde_off		:	P0_07				output
	NRF_P0->OUT |= 1<<7;				//Cmde_off -> 1 -> alim off
	
	NRF_P0->DIR |= 1<<8;				//En_SW				:	P0_08				output	
	NRF_P0->OUT &= ~(1<<8);			//En_SW -> 0 -> pas de lecture de la tension de batterie
	
/*	
//Init Cmd_off;
	NRF_P0->OUT |= 1<<19;				//Mets le bit Cmd_off a 1 avant de le déclarer en sortie
	NRF_P0->DIR |= 1<<19;				//Init port pour Cmd_off

//Init Pol+
	NRF_P0->OUT |= 1<<18;				//Mets le bit Pol+ a 1 
	NRF_P0->DIR |= 1<<18;				//Init port pour Pol+

//Init Pol-
	NRF_P0->OUT &= ~(1<<21);		//Mets le bit Pol- a 1 
	NRF_P0->DIR |= 1<<21;				//Init port pour Pol-
*/
	
/*
//config carte Bras Europrotect
//Init LED	
	NRF_P0->DIR |= 1<<N_Led;					//Init port pour led
	
//Init En_Stepup
	NRF_P0->OUT &= ~(1<<14);		//Mets le bit En_stepup a 0 avant de le déclarer en sortie
	NRF_P0->DIR |= 1<<14;				//Init port pour Led0

//Init Led0
	NRF_P0->OUT &= ~(1<<31);		//Mets le bit Led0 a 0 avant de le déclarer en sortie
	NRF_P0->DIR |= 1<<31;				//Init port pour Led0

//Init Led1
	NRF_P0->OUT &= ~(1<<27);		//Mets le bit Led1 a 0 avant de le déclarer en sortie
	NRF_P0->DIR |= 1<<27;				//Init port pour Led1

//Init Led2
	NRF_P0->OUT &= ~(1<<15);		//Mets le bit Led2 a 0 avant de le déclarer en sortie
	NRF_P0->DIR |= 1<<15;				//Init port pour Led2

//Init Led3
	NRF_P0->OUT &= ~(1<<05);		//Mets le bit Led3 a 0 avant de le déclarer en sortie
	NRF_P0->DIR |= 1<<05;				//Init port pour Led3

//Init nChg
	NRF_P0->DIR &= ~(1<<16);		//Init port en entree pour nChg
	NRF_P0->PIN_CNF[16] &= ~(1<<1);
	
//Init Pbutton
	NRF_P0->DIR &= ~(1<<12);		//Init port en entree pour nChg
	NRF_P0->PIN_CNF[12] &= ~(1<<1);
*/


/*	
//Init Timer1
	NRF_TIMER1->TASKS_CLEAR = 1;
	
	NRF_TIMER1->MODE = 0x0000;				//Timer	mode
	NRF_TIMER1->SHORTS = 0x0001;			//Shortcut entre compare0 et clear
	NRF_TIMER1->CC[0] = 1000;						//Capture compare 0 mis a 50 -> 20 kHz
	
	NRF_TIMER1->INTENSET = 1<<16;			//Enable IT sur compare0;
	
	NRF_TIMER1->BITMODE = 0x0000;			//16 bits mode
	NRF_TIMER1->PRESCALER = 0x04;			//Prescaler = 16 -> 1MHz
	
	NVIC_EnableIRQ(TIMER1_IRQn);			//Valide IT Timer
	//NVIC_SetPriority(TIMER1_IRQn,1);
	
	//NRF_TIMER1->TASKS_STOP = 0;
	NRF_TIMER1->TASKS_START = 1;			//Demarre Timer1
*/
/*
//Init UART
	NRF_UARTE0->PSEL.TXD = 26;				//TX sur PO.26
	NRF_UARTE0->PSEL.RXD = 25;				//RX sur PO.25
	NRF_UARTE0->BAUDRATE = UART_BAUDRATE_BAUDRATE_Baud115200;	//0x00275000UL;
	NRF_UARTE0->CONFIG = 0;						//Pas de parite ni de flow control
	NRF_UARTE0->TXD.PTR = (uint32_t)(&TX_data);
	NRF_UARTE0->TXD.MAXCNT = 3;
	
	NRF_UARTE0->ENABLE = 8;						//Enable UART
	
//Init IO pour DAC à la main
	NRF_P0->DIR |= 1<<11;							//nSync_DAC en sortie
	NRF_P0->DIR |= 1<<12;							//CK_DAC en sortie
	NRF_P0->DIR |= 1<<14;							//SDin_DAC en sortie
	NRF_P0->OUT &= ~(1<<12);					//Clear CK
	NRF_P0->OUT &= ~(1<<14);					//Clear SDin
	NRF_P0->OUT |= 1<<11;							//Set nSync
	
//Init SPI0 pour accéléro
	NRF_SPIM0->ENABLE = 0;						//Disable SPIM0
	
	NRF_SPIM0->PSEL.MOSI = 19;				//Routage de MOSI
	NRF_SPIM0->PSEL.MISO = 17;				//Routage de MISO
	NRF_SPIM0->PSEL.SCK = 13;					//Routage de SCK
	
	NRF_P0->DIR |= 1<<15;							//Init port pour CS SPI
	NRF_P0->OUT |= 1<<15;							//Set CS SPI
	
	NRF_SPIM0->FREQUENCY = SPIM_FREQUENCY_FREQUENCY_M1;			//SPI clock à 1 MHz
	NRF_SPIM0->TXD.PTR = (uint32_t)(&Buf_send_Accelero);
	NRF_SPIM0->TXD.MAXCNT = 3;
	NRF_SPIM0->RXD.PTR = (uint32_t)(&Buf_receive_Accelero);
	NRF_SPIM0->RXD.MAXCNT = 3;
	NRF_SPIM0->CONFIG = 6;						//MSB first, CPHA et CPOL = 1
	
	NRF_SPIM0->ORC = 0x5A;
	NRF_SPIM0->INTENSET = 1 << 6;			//Valide IT sur END event
	NVIC_EnableIRQ(SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQn);		//Valide IT SPI
	
	NRF_SPIM0->ENABLE = 7;						//Enable SPIM0
	
//Init SPI1 pour memoire flash
		//SDI_flash		:	SIO06				
		//SDO_flash		:	SIO09
		//CK_flash		:	SIO07			
		//nCS_flash		:	SIO08			
		//WP_flash		:	SIO10
		
	NRF_SPIM1->ENABLE = 0;						//Disable SPIM1
	
	
	NRF_SPIM1->PSEL.MOSI = 6;					//Routage de MOSI
	NRF_SPIM1->PSEL.MISO = 9;					//Routage de MISO
	NRF_SPIM1->PSEL.SCK = 7;					//Routage de SCK
	
	NRF_P0->DIR |= 1<<8;							//Init port pour CS SPI
	NRF_P0->OUT |= 1<<8;							//Set CS SPI
	
	NRF_P0->DIR |= 1<<10;							//Init port pour WP de la Flash
	NRF_P0->OUT |= 1<<10;							//Set WP de la Flash
	
	
	NRF_SPIM1->FREQUENCY = SPIM_FREQUENCY_FREQUENCY_K250;			//SPI clock à 1 MHz
	NRF_SPIM1->TXD.PTR = (uint32_t)(&Buf_send_Flash);
	NRF_SPIM1->TXD.MAXCNT = 10;
	NRF_SPIM1->RXD.PTR = (uint32_t)(&Buf_receive_Flash);
	NRF_SPIM1->RXD.MAXCNT = 10;
	NRF_SPIM1->CONFIG = 6;						//MSB first, CPHA et CPOL = 1
	
	NRF_SPIM1->ORC = 0xFF;
	NRF_SPIM1->INTENSET = 1 << 6;			//Valide IT sur END event
	NVIC_EnableIRQ(SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQn);		//Valide IT SPI
	
	NRF_SPIM1->ENABLE = 7;						//Enable SPIM1

//Init ADC
	NRF_SAADC->ENABLE = 0;					//Disable ADC
	
	NRF_SAADC->CH[0].PSELP = 6;			//Connect CH0p to AIN5
	NRF_SAADC->CH[0].CONFIG = (1 << 8);		//Gain de 1/5, single ended, 3us de tacq, Ref interne de 0,6V
	
	//NRF_SAADC->CH[1].PSELP = 7;			//Connect CH1p to AIN6
	NRF_SAADC->CH[1].CONFIG = (1 << 8);		//Gain de 1/5, single ended, 3us de tacq, Ref interne de 0,6V
	
	//NRF_SAADC->CH[2].PSELP = 8;			//Connect CH2p to AIN7
	NRF_SAADC->CH[2].CONFIG = (1 << 8);		//Gain de 1/5, single ended, 3us de tacq, Ref interne de 0,6V
	
	NRF_SAADC->RESOLUTION = 1;			//Resolution a 10 bits
	NRF_SAADC->SAMPLERATE = 0;			//Sample controle par Sample_task
	NRF_SAADC->INTENSET = 4;				//Valide IT sur DONE
	NRF_SAADC->RESULT.MAXCNT = 1;
	NRF_SAADC->RESULT.PTR = (uint32_t)(&ADC_Result);
	
	NVIC_EnableIRQ(SAADC_IRQn);			//Valide IT SAADC
	
	NRF_SAADC->ENABLE = 1;					//Enable ADC
	NRF_SAADC->TASKS_START = 1;
*/	
}

/*--------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------*/

void TIMER1_IRQHandler(void)
{			
	NRF_TIMER1->EVENTS_COMPARE[0] = 0;	//Clear event compare 0
	
	//NRF_P0->OUT ^= 1<<N_Led;								//Toggle led
	
	NRF_SAADC->TASKS_SAMPLE = 1;
	
	Flag_Timer = 1;
	
	//TX_data++;
	//NRF_UART0->TASKS_STARTTX = 1;				//Start transmission UART0
	/*
	NRF_P0->OUT &= ~(1<<15);						//Clear CS SPI
	NRF_SPIM0->TASKS_START = 1;					//Start transmission SPI
	*/
}

/*--------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------*/

void SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQHandler(void)
{			
	NRF_SPIM0->EVENTS_END = 0;		//Clear END event
	
	NRF_SPIM0->TASKS_STOP = 1;					//Stop transaction SPI
	NRF_P0->OUT |= 1<<15;								//Set CS SPI
	
	TX_data[0] = Buf_receive_Accelero[2];
	TX_data[1] = 0xAA;
	TX_data[2] = 0x55;
	
	//NRF_UART0->TASKS_STARTTX = 1;				//Start transmission UART0
	
	Flag_SPI = 1;
	
}

/*--------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------*/

void SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQHandler(void)
{			
	NRF_SPIM1->EVENTS_END = 0;		//Clear END event
	
	NRF_SPIM1->TASKS_STOP = 1;					//Stop transaction SPI
	NRF_P0->OUT |= 1<<8;								//Set CS SPI
	
	Flag_SPI1 = 1;
}


/*--------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------*/

void SAADC_IRQHandler(void)
{
	NRF_SAADC->EVENTS_DONE = 0;		//Clear event done
	
	NRF_SAADC->TASKS_STOP = 1;
	NRF_SAADC->ENABLE = 0;					//Disable ADC
	Toggle_ADC = (Toggle_ADC + 1) %3;
	switch(Toggle_ADC)
	{
		case 0		:		NRF_SAADC->CH[2].PSELP = 0;			//Disconnect CH2p from AIN7
									NRF_SAADC->CH[0].PSELP = 6;			//Connect CH0p to AIN5
									break;
		case 1		:		NRF_SAADC->CH[0].PSELP = 0;			//Disconnect CH0p to AIN5
									NRF_SAADC->CH[1].PSELP = 7;			//Connect CH1p to AIN6
									break;
		case 2		:		NRF_SAADC->CH[1].PSELP = 0;			//Disconnect CH1p to AIN6
									NRF_SAADC->CH[2].PSELP = 8;			//Connect CH2p to AIN7
									break;
		default		:		NRF_SAADC->CH[0].PSELP = 6;			//Connect CH0p to AIN5
									NRF_SAADC->CH[1].PSELP = 0;			//Disconnect CH1p to AIN6
									NRF_SAADC->CH[2].PSELP = 0;			//Disconnect CH2p from AIN7
									break;		
	}
	NRF_SAADC->ENABLE = 1;					//Enable ADC
	NRF_SAADC->TASKS_START = 1;
	
	Flag_ADC = 1;
	
}

/*--------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------*/

void Init_data(void)
{
	unsigned int i;
	unsigned int temp;
	float f_temp;
	
//Init de la sinusoide de stimulation pour Zth	
	for(i=0;i<20;i++)
	{
		f_temp = 1 + sin(i*6.283185/20);
		f_temp = f_temp * 2046;
		temp = (unsigned int)(f_temp);
		temp &= 0x0fff;
		Stimu[i] = temp;
	}
	
	Buf_send_Accelero[0] = 0x8F;
	Buf_send_Accelero[1] = 0x00;
	
	Buf_send_Flash[0] = 0x9f;				//Commande read ID
	for(i=1;i<0x55;i++) Buf_send_Flash[i] = 0;		//Reste de la trame à 0
	
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

void Send_val(void)
{
	unsigned char i;
	unsigned int mask = 0x2000;
	
	Val_SPI = Stimu[index_SPI];
	
	NRF_P0->OUT |= 1<<12;							//Set CK
	NRF_P0->OUT &= ~(1<<11);					//Clear nSync
	
	for(i=0;i<14;i++)
	{
		NRF_P0->OUT |= 1<<12;							//Set CK
		if(Val_SPI & mask) NRF_P0->OUT |= 1<<14; else NRF_P0->OUT &= ~(1<<14);
		mask = mask >> 1;
		NRF_P0->OUT &= ~(1<<12);					//Clear CK
	
	}
	
	NRF_P0->OUT |= 1<<12;							//Set CK
	NRF_P0->OUT &= ~(1<<14);
	NRF_P0->OUT &= ~(1<<12);					//Clear CK
	NRF_P0->OUT &= ~(1<<12);					//Clear CK
	NRF_P0->OUT &= ~(1<<12);					//Clear CK
	
	NRF_P0->OUT |= 1<<12;							//Set CK
	NRF_P0->OUT &= ~(1<<14);
	NRF_P0->OUT &= ~(1<<12);					//Clear CK
	
	NRF_P0->OUT |= 1<<11;							//Set nSync
	
	index_SPI = (index_SPI + 1)% 20;
	
	Flag_Timer = 0;
}

/*--------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------*/

void Config_Accelero_temp(void)
{
	NRF_SPIM0->TXD.MAXCNT = 2;
	NRF_SPIM0->RXD.MAXCNT = 2;
	
	Buf_send_Accelero[0] = 0x1f;				//Ecriture dans le reg 0x1f
	Buf_send_Accelero[1] = 0xc0;				//Temp_en's à 1
	NRF_P0->OUT &= ~(1<<15);						//Clear CS SPI
	NRF_SPIM0->TASKS_START = 1;					//Start transmission SPI
	while(!Flag_SPI);
	Flag_SPI = 0;
	
	Buf_send_Accelero[0] = 0x1f;				//Ecriture dans le reg 0x1f
	Buf_send_Accelero[1] = 0xc0;				//Temp_en's à 1
	NRF_P0->OUT &= ~(1<<15);						//Clear CS SPI
	NRF_SPIM0->TASKS_START = 1;					//Start transmission SPI
	while(!Flag_SPI);
	Flag_SPI = 0;
	
	Buf_send_Accelero[0] = 0x23;				//Ecriture dans le reg 0x1f
	Buf_send_Accelero[1] = 0x80;				//BDU à 1
	NRF_P0->OUT &= ~(1<<15);						//Clear CS SPI
	NRF_SPIM0->TASKS_START = 1;					//Start transmission SPI
	while(!Flag_SPI);
	Flag_SPI = 0;
	
	Buf_send_Accelero[0] = 0x20;				//Ecriture dans le reg 0x1f
	Buf_send_Accelero[1] = 0x47;				//50 Hz, enable x-y-z
	NRF_P0->OUT &= ~(1<<15);						//Clear CS SPI
	NRF_SPIM0->TASKS_START = 1;					//Start transmission SPI
	while(!Flag_SPI);
	Flag_SPI = 0;
	
//Init de la trame lecture de temperature	
	Buf_send_Accelero[0] = 0xec;					//Read multiple a partir du reg 0x0c
	Buf_send_Accelero[1] = 0x00;
	Buf_send_Accelero[2] = 0x00;
	
	NRF_SPIM0->TXD.MAXCNT = 3;
	NRF_SPIM0->RXD.MAXCNT = 3;
	
}

/*--------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------*/

void Get_Flash_ID(void)
{
	unsigned char i;
	
	/*
	NRF_SPIM1->TXD.MAXCNT = 0x56;
	NRF_SPIM1->RXD.MAXCNT = 0x56;
	*/
	
	NRF_P0->OUT &= ~(1<<8);						//Clear CS SPI
	NRF_SPIM1->TASKS_START = 1;				//Start transmission SPI
	
	while(!Flag_SPI1);
	Flag_SPI1 = 0;
	
	for(i=0;i<11;i++) TX_data[i] = Buf_receive_Flash[i];
	TX_data[0] = 0xAA;
	NRF_UARTE0->TXD.MAXCNT = 11;
	NRF_UART0->TASKS_STARTTX = 1;				//Start transmission UART0
	
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
/*--------------------------------------------------------------------------*/

void Toggle_Polar(void)
{
	NRF_P0->OUT ^= (1<<18);
	NRF_P0->OUT ^= (1<<21);
}

/*--------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------*/

void main(void)
{
	unsigned int Cnt_IT = 0;
	unsigned char Roll = 0;
	uint8_t Toggle = 0;
	uint32_t tmp1, tmp2;
	
	
	Delais();
	
	Init_CPU();
	
	//NRF_P0->OUT |= 1<<14;		//Mets le bit En_stepup a 1
	//NRF_P0->OUT |= 1<<18;
	NRF_P0->OUT |= (1<<N_Led);
		
	while(1)
	{
		/*
		while(Flag_Timer == 0);
		Flag_Timer = 0;
		Cnt_IT++;
		
		if(Cnt_IT == 200)
		{
			NRF_P0->OUT ^= 1<<N_Led;
			Cnt_IT = 0;
		}
		
		
		if(Cnt_IT == 250)
		{
			tmp1 = NRF_P0->IN;
			tmp1 &= 1<<16;
			tmp2 = NRF_P0->IN;
			tmp2 &= 1<<12;
			if((tmp1!=0) || (tmp2 == 0))	NRF_P0->OUT &= ~(1<<26);
			else NRF_P0->OUT |= (1<<26);				
			if(tmp2 == 0) Toggle ^=1;
		}
		
		if(Cnt_IT == 270)
		{
			NRF_P0->OUT |= (1<<26);	
		}
		
		if(Cnt_IT == 500)
		{
			NRF_P0->OUT &= ~(0x88008020);
			
			if(Toggle == 1)
			{
				Roll = (Roll + 1)%4;
				
				switch(Roll)
				{
					case 0	:	NRF_P0->OUT |= 1<<31;
										NRF_P0->OUT |= 1<<27;
										break;
					case 1	:	NRF_P0->OUT |= 1<<27;
										NRF_P0->OUT |= 1<<15;
										break;
					case 2	:	NRF_P0->OUT |= 1<<15;
										NRF_P0->OUT |= 1<<5;
										break;
					case 3	:	NRF_P0->OUT |= 1<<5;
										NRF_P0->OUT |= 1<<31;
										break;
					default	:	NRF_P0->OUT &= ~(0x88008020);
										break;
				}
				NRF_P0->OUT |= (1<<14);
			}
			
		}
		
		
		if(Cnt_IT == 1000) 
		{
			Cnt_IT = 0;
			//NRF_P0->OUT &= ~(1<<14);
		}
		*/
	}
}

/*--------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------*/


