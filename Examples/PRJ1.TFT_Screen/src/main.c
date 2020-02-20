#include <stm32f103xb.h>
#define y 65504

void sendSPIData(char data);
void lowSPI_CS(void);
void highSPI_CS(void);
void lowRS(void);
void highRS(void);
void lowRST(void);
void highRST(void);
void wait_ms(unsigned int time);
void alreadySend_SPIData();
unsigned int pacman[11][11]={{0,0,0,0,y,y,y,0,0,0,0},{0,0,0,y,y,y,y,y,0,0,0},{0,0,y,y,y,y,y,y,y,0,0},
                             {0,y,y,y,y,y,y,y,y,y,0},{0,0,0,y,y,y,y,y,y,y,y},{0,0,0,0,y,y,y,y,y,y,y},
                             {0,0,0,y,y,y,y,y,y,y,y},{0,y,y,y,y,y,y,y,y,y,0},{0,0,y,y,y,y,y,y,y,0,0},
                             {0,0,0,y,y,y,y,y,0,0,0},{0,0,0,0,y,y,y,0,0,0,0}};


int main(void){

	//--Inicialización interface debug---------------------------------------------------------------------

	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;	        // AFIOEN = 1, activar reloj de modulo AFIO
	AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_1;			// SWJ_CFG = 010 JTAG disabled y SW-DP enable

	//--Inicializacion de reloj externo a 72 Mhz-----------------------------------------------------------

	FLASH->ACR |=FLASH_ACR_LATENCY_1;			// Ajustar Latencia de memoria Flash en 2 (Debido al ajuste de alta frecuencia)
	RCC->CR |= RCC_CR_HSEON; 					// HSE0N = 1, Habilitar reloj externo
    while(!(RCC->CR & RCC_CR_HSERDY));   		// HSERDY = 1 ?, Esperar a que el reloj externo se estabilice
    RCC->CFGR |= RCC_CFGR_PLLSRC; 				// PLLSRC=1, Seleccionar reloj externo como fuente de reloj de PLL
    RCC->CFGR |= RCC_CFGR_PLLMULL9;				// PLLMUL=7, Seleccionar 9 como factor PLL
    RCC->CR	  |= RCC_CR_PLLON;					// PLLON=1 Encender PLL
    while(!(RCC->CR & RCC_CR_PLLRDY));			// PLLRDY=1 ? Esperar a que se ajuste el PLL
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV2; 			// PPRE1=0b100, Ajustar el prescaler de APB1, para obtener 36Mhz
    RCC->CFGR |= RCC_CFGR_SW_1;    				// SW=2, PLL seleccionado como reloj del sistema
    while(!(RCC->CFGR & RCC_CFGR_SWS_PLL)); 	// SWS=2? , Esperar a que el reloj del sistema se configure

    //--Inicialización SP1-----------------------------------------------------------------------------------

    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;         // SPI1EN=1, Activación de reloj SPI1
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;			// IOPAEN = 1, Habilitar reloj de GPIOA

    // Configuración de pines

    GPIOA->CRL   = 0;                           // Limpiar configuración de puerto A
    // Configuración del CS
    GPIOA->CRL   |= GPIO_CRL_MODE4;            	// MODE4 = 0b11, Modo salida 50 MHz
    GPIOA->CRL   |= GPIO_CRL_CNF4_1;           	// CNF4= 0b10, Modo alternado push-pull
    // Configuración de CLK
    GPIOA->CRL   |= GPIO_CRL_MODE5;            	// MODE5 = 0b11, Modo salida 50 MHz
    GPIOA->CRL   |= GPIO_CRL_CNF5_1;           	// CNF5= 0b10, Modo alternado push-pull
    // Configuración de MISO
    GPIOA->CRL   &= ~GPIO_CRL_MODE6;           	// MODE6 = 0b00, Modo de entrada
    GPIOA->CRL   |= GPIO_CRL_CNF6_1;           	// MODE6 = 0b10, Modo de entrada pull-up/pull-down
    // Configuración de MOSI
    GPIOA->CRL 	 |= GPIO_CRL_MODE7;            	// MODE7 = 0b11, Modo salida 50 MHz
    GPIOA->CRL 	 |= GPIO_CRL_CNF7_1;           	// CNF7= 0b10, Modo alternado push-pull

    // Configuración

    SPI1->CR1    &= ~SPI_CR1_BIDIMODE;        	// BIDIMODE = 0b0, Modo unidereccional
    SPI1->CR1    &= ~SPI_CR1_DFF;             	// DFF = 0b0, Trama de datos de 8 bits
    SPI1->CR1    &= ~SPI_CR1_RXONLY;          	// RXONLY = 0b0, Comunicación full duplex
    SPI1->CR1    |= SPI_CR1_SSM;              	// SSM = 0b1, CS por software
    SPI1->CR1    &= ~SPI_CR1_LSBFIRST;        	// LSBFIRST = 0b0, Bit más significativo primero
    SPI1->CR1    |= SPI_CR1_BR_2;             	// BR= 0b100, PCLK/32 = 2.250 MHz
    SPI1->CR1    |= SPI_CR1_MSTR;             	// MSRT = 0b1, Modo maestro
    SPI1->CR2    |= SPI_CR2_SSOE;             	// SS0E = 0b1, Habilitar CS
    SPI1->CR1    |= SPI_CR1_SPE;              	// SPE = 0b1, Activar SPI


    //--Configuracion Pin B0 (TFT_RS)-----------------------------------------------------------------
    GPIOB->CRL   = 0;                           // Limpiar configuración de puerto B
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;			// Habilitar reloj del Puerto B
    GPIOB->CRL   |= GPIO_CRL_MODE0;            	// MODE0 = 0b11, Modo salida 50 MHz
    GPIOB->CRL   &= ~GPIO_CRL_CNF0;           	// CNF0= 0b00, Modo General Purpose Push-pull

    //--Configuracion Pin B3 (TFT_RST)-----------------------------------------------------------------
    GPIOB->CRL   |= GPIO_CRL_MODE1;            	// MODE3 = 0b11, Modo salida 50 MHz
    GPIOB->CRL   &= ~GPIO_CRL_CNF1;           	// CNF3= 0b00, Modo General Purpose Push-pull

    //Reset por hardware

    lowSPI_CS();
    highRST();
    wait_ms(10);
    lowRST();
    wait_ms(500);
    highRST();
    wait_ms(50);

    //sendSPIData();

    // Dato basura
    lowRS();
    sendSPIData(0x00);
    sendSPIData(0x00);
    alreadySend_SPIData();


    //Power Control 1
    lowRS();
    sendSPIData(0x00);
    sendSPIData(0x10);
    alreadySend_SPIData();

    highRS();
    sendSPIData(0x00);
    sendSPIData(0x00);
    alreadySend_SPIData();

    //Power Control 2
    lowRS();
    sendSPIData(0x00);
    sendSPIData(0x11);
    alreadySend_SPIData();

    highRS();
    sendSPIData(0x00);
    sendSPIData(0x00);
    alreadySend_SPIData();

    //Power Control 3
    lowRS();
    sendSPIData(0x00);
    sendSPIData(0x12);
    alreadySend_SPIData();

    highRS();
    sendSPIData(0x00);
   	sendSPIData(0x00);
    alreadySend_SPIData();

    //Power Control 4
    lowRS();
    sendSPIData(0x00);
   	sendSPIData(0x13);
    alreadySend_SPIData();

    highRS();
    sendSPIData(0x00);
    sendSPIData(0x00);
    alreadySend_SPIData();

    //Power Control 5
    lowRS();
    sendSPIData(0x00);
    sendSPIData(0x14);
    alreadySend_SPIData();

    highRS();
    sendSPIData(0x00);
    sendSPIData(0x00);
    alreadySend_SPIData();

    //Power-on sequence----------------------------------------
    wait_ms(40);

    //Power Control 2
    lowRS();
    sendSPIData(0x00);
    sendSPIData(0x11);
    alreadySend_SPIData();

    highRS();
    sendSPIData(0x00);
    sendSPIData(0x18);
    alreadySend_SPIData();

    //Power Control 3
    lowRS();
    sendSPIData(0x00);
    sendSPIData(0x12);
    alreadySend_SPIData();

    highRS();
    sendSPIData(0x61);
    sendSPIData(0x21);
    alreadySend_SPIData();

    //Power Control 4
    lowRS();
    sendSPIData(0x00);
    sendSPIData(0x13);
    alreadySend_SPIData();

    highRS();
    sendSPIData(0x00);
    sendSPIData(0x6f);
    alreadySend_SPIData();

    //Power Control 5
    lowRS();
    sendSPIData(0x00);
    sendSPIData(0x14);
    alreadySend_SPIData();

    highRS();
    sendSPIData(0x49);
    sendSPIData(0x5f);
    alreadySend_SPIData();

    //Power Control 1
    lowRS();
    sendSPIData(0x00);
    sendSPIData(0x10);
    alreadySend_SPIData();

    highRS();
    sendSPIData(0x08);
    sendSPIData(0x00);
    alreadySend_SPIData();

    wait_ms(40);

    //Power Control 2
    lowRS();
    sendSPIData(0x00);
    sendSPIData(0x11);
    alreadySend_SPIData();

    highRS();
    sendSPIData(0x10);
    sendSPIData(0x3B);
    alreadySend_SPIData();

    wait_ms(40);

    //DRIVER_OUTPUT_CTRL
    lowRS();
    sendSPIData(0x00);
    sendSPIData(0x01);
    alreadySend_SPIData();

    highRS();
    sendSPIData(0x01);
    sendSPIData(0x1C);
    alreadySend_SPIData();

    //LCD_AC_DRIVING_CTRL
    lowRS();
    sendSPIData(0x00);
    sendSPIData(0x02);
    alreadySend_SPIData();

    highRS();
    sendSPIData(0x01);
    sendSPIData(0x00);
    alreadySend_SPIData();

    //ENTRY_MODE
    lowRS();
    sendSPIData(0x00);
    sendSPIData(0x03);
    alreadySend_SPIData();

    highRS();
    sendSPIData(0x10);
    sendSPIData(0x38);
    alreadySend_SPIData();

    //DISP_CTRL1
    lowRS();
    sendSPIData(0x00);
    sendSPIData(0x07);
    alreadySend_SPIData();

    highRS();
    sendSPIData(0x00);
    sendSPIData(0x00);
    alreadySend_SPIData();

    //BLANK_PERIOD_CTRL1
    lowRS();
    sendSPIData(0x00);
    sendSPIData(0x08);
    alreadySend_SPIData();

    highRS();
    sendSPIData(0x08);
    sendSPIData(0x08);
    alreadySend_SPIData();

    //FRAME_CYCLE_CTRL
    lowRS();
    sendSPIData(0x00);
    sendSPIData(0x0B);
    alreadySend_SPIData();

    highRS();
    sendSPIData(0x11);
    sendSPIData(0x00);
    alreadySend_SPIData();

    //INTERFACE_CTRL
    lowRS();
    sendSPIData(0x00);
    sendSPIData(0x0C);
    alreadySend_SPIData();

    highRS();
    sendSPIData(0x00);
    sendSPIData(0x00);
    alreadySend_SPIData();

    //OSC_CTRL
    lowRS();
    sendSPIData(0x00);
    sendSPIData(0x0F);
    alreadySend_SPIData();

    highRS();
    sendSPIData(0x0D);
    sendSPIData(0x01);
    alreadySend_SPIData();

    //VCI_RECYCLING
    lowRS();
    sendSPIData(0x00);
    sendSPIData(0x15);
    alreadySend_SPIData();

    highRS();
    sendSPIData(0x00);
    sendSPIData(0x20);
    alreadySend_SPIData();

    //RAM_ADDR_SET1
    lowRS();
    sendSPIData(0x00);
    sendSPIData(0x20);
    alreadySend_SPIData();

    highRS();
    sendSPIData(0x00);
    sendSPIData(0x00);
    alreadySend_SPIData();

    //RAM_ADDR_SET2
    lowRS();
    sendSPIData(0x00);
    sendSPIData(0x21);
    alreadySend_SPIData();

    highRS();
    sendSPIData(0x00);
    sendSPIData(0x00);
    alreadySend_SPIData();


    //Set GRAM area------------------------------------

    //GATE_SCAN_CTRL
    lowRS();
    sendSPIData(0x00);
    sendSPIData(0x30);
    alreadySend_SPIData();

    highRS();
    sendSPIData(0x00);
    sendSPIData(0x00);
    alreadySend_SPIData();

    //VERTICAL_SCROLL_CTRL1
    lowRS();
    sendSPIData(0x00);
    sendSPIData(0x31);
    alreadySend_SPIData();

    highRS();
    sendSPIData(0x00);
    sendSPIData(0xDB);
    alreadySend_SPIData();

    //VERTICAL_SCROLL_CTRL2
    lowRS();
    sendSPIData(0x00);
    sendSPIData(0x32);
    alreadySend_SPIData();

    highRS();
    sendSPIData(0x00);
    sendSPIData(0x00);
    alreadySend_SPIData();

    //VERTICAL_SCROLL_CTRL3
    lowRS();
    sendSPIData(0x00);
    sendSPIData(0x33);
    alreadySend_SPIData();

    highRS();
    sendSPIData(0x00);
    sendSPIData(0x00);
    alreadySend_SPIData();

    //PARTIAL_DRIVING_POS1
    lowRS();
    sendSPIData(0x00);
    sendSPIData(0x34);
    alreadySend_SPIData();

    highRS();
    sendSPIData(0x00);
    sendSPIData(0xDB);
    alreadySend_SPIData();

    //PARTIAL_DRIVING_POS2
    lowRS();
    sendSPIData(0x00);
    sendSPIData(0x35);
    alreadySend_SPIData();

    highRS();
    sendSPIData(0x00);
    sendSPIData(0x00);
    alreadySend_SPIData();

    //HORIZONTAL_WINDOW_ADDR1
    lowRS();
    sendSPIData(0x00);
    sendSPIData(0x36);
    alreadySend_SPIData();

    highRS();
    sendSPIData(0x00);
    sendSPIData(0xAF);
    alreadySend_SPIData();

    //HORIZONTAL_WINDOW_ADDR2
    lowRS();
    sendSPIData(0x00);
    sendSPIData(0x37);
    alreadySend_SPIData();

    highRS();
    sendSPIData(0x00);
    sendSPIData(0x00);
    alreadySend_SPIData();

    //VERTICAL_WINDOW_ADDR1
    lowRS();
    sendSPIData(0x00);
    sendSPIData(0x38);
    alreadySend_SPIData();

    highRS();
    sendSPIData(0x00);
    sendSPIData(0xDB);
    alreadySend_SPIData();

    //VERTICAL_WINDOW_ADDR2
    lowRS();
    sendSPIData(0x00);
    sendSPIData(0x39);
    alreadySend_SPIData();

    highRS();
    sendSPIData(0x00);
    sendSPIData(0x00);
    alreadySend_SPIData();

    // Set GAMMA curve-------------------------------------
    //GAMMA_CTRL1
    lowRS();
    sendSPIData(0x00);
    sendSPIData(0x50);
    alreadySend_SPIData();

    highRS();
    sendSPIData(0x00);
    sendSPIData(0x00);

    //GAMMA_CTRL2
    lowRS();
    sendSPIData(0x00);
    sendSPIData(0x51);
    alreadySend_SPIData();

    highRS();
    sendSPIData(0x08);
    sendSPIData(0x08);
    alreadySend_SPIData();

    //GAMMA_CTRL3
    lowRS();
    sendSPIData(0x00);
    sendSPIData(0x52);
    alreadySend_SPIData();

    highRS();
    sendSPIData(0x08);
    sendSPIData(0x0A);
    alreadySend_SPIData();

    //GAMMA_CTRL4
    lowRS();
    sendSPIData(0x00);
    sendSPIData(0x53);
    alreadySend_SPIData();

    highRS();
    sendSPIData(0x00);
    sendSPIData(0x0A);
    alreadySend_SPIData();

    //GAMMA_CTRL5
    lowRS();
    sendSPIData(0x00);
    sendSPIData(0x54);
    alreadySend_SPIData();

    highRS();
    sendSPIData(0x0A);
    sendSPIData(0x08);
    alreadySend_SPIData();

    //GAMMA_CTRL6
    lowRS();
    sendSPIData(0x00);
    sendSPIData(0x55);
    alreadySend_SPIData();

    highRS();
    sendSPIData(0x08);
    sendSPIData(0x08);
    alreadySend_SPIData();

    //GAMMA_CTRL7
    lowRS();
    sendSPIData(0x00);
    sendSPIData(0x56);
    alreadySend_SPIData();

    highRS();
    sendSPIData(0x00);
    sendSPIData(0x00);
    alreadySend_SPIData();

    //GAMMA_CTRL8
    lowRS();
    sendSPIData(0x00);
    sendSPIData(0x57);
    alreadySend_SPIData();

    highRS();
    sendSPIData(0x0A);
    sendSPIData(0x00);
    alreadySend_SPIData();

    //GAMMA_CTRL9
    lowRS();
    sendSPIData(0x00);
    sendSPIData(0x58);
    alreadySend_SPIData();

    highRS();
    sendSPIData(0x07);
    sendSPIData(0x10);
    alreadySend_SPIData();

    //GAMMA_CTRL10
    lowRS();
    sendSPIData(0x00);
    sendSPIData(0x59);
    alreadySend_SPIData();

    highRS();
    sendSPIData(0x07);
    sendSPIData(0x10);
    alreadySend_SPIData();

    //-----------------------------------
    //DISP_CTRL1
    lowRS();
    sendSPIData(0x00);
    sendSPIData(0x07);
    alreadySend_SPIData();

    highRS();
    sendSPIData(0x00);
    sendSPIData(0x12);
    alreadySend_SPIData();

    wait_ms(50);

    //DISP_CTRL1
    lowRS();
    sendSPIData(0x00);
    sendSPIData(0x07);
    alreadySend_SPIData();

    highRS();
    sendSPIData(0x10);
    sendSPIData(0x17);
    alreadySend_SPIData();

    // Turn on backlight
    //setBacklight(true);
    //setOrientation(0);

    // Initialize variables
    //setBackgroundColor( COLOR_BLACK );

    //ENTRY MODE
    lowRS();
    sendSPIData(0x00);
    sendSPIData(0x03);
    alreadySend_SPIData();

    highRS();
    sendSPIData(0x10);
    sendSPIData(0x18);
    alreadySend_SPIData();

    //HORIZONTAL_WINDOW_ADDR1
    lowRS();
    sendSPIData(0x00);
    sendSPIData(0x36);
    alreadySend_SPIData();

    highRS();
    sendSPIData(0x00);
    sendSPIData(0xAF);
    alreadySend_SPIData();

    //HORIZONTAL_WINDOW_ADDR2
    lowRS();
    sendSPIData(0x00);
    sendSPIData(0x37);
    alreadySend_SPIData();

    highRS();
    sendSPIData(0x00);
    sendSPIData(0x00);
    alreadySend_SPIData();

    //VERTICAL_WINDOW_ADDR1
    lowRS();
    sendSPIData(0x00);
    sendSPIData(0x38);
    alreadySend_SPIData();

    highRS();
    sendSPIData(0x00);
    sendSPIData(0xDB);
    alreadySend_SPIData();

    //VERTICAL_WINDOW_ADDR2
    lowRS();
    sendSPIData(0x00);
    sendSPIData(0x39);
    alreadySend_SPIData();

    highRS();
    sendSPIData(0x00);
    sendSPIData(0x00);
    alreadySend_SPIData();

    //RAM_ADDR_SET1
    lowRS();
    sendSPIData(0x00);
    sendSPIData(0x20);
    alreadySend_SPIData();

    highRS();
    sendSPIData(0x00);
    sendSPIData(0x00);
    alreadySend_SPIData();

    //RAM_ADDR_SET2
    lowRS();
    sendSPIData(0x00);
    sendSPIData(0x21);
    alreadySend_SPIData();

    highRS();
    sendSPIData(0x00);
    sendSPIData(0x00);
    alreadySend_SPIData();

    //Inicio de Transferencia de datos
    //WRITE DATA TO GRAM
    lowRS();
    sendSPIData(0x00);
    sendSPIData(0x22);
    alreadySend_SPIData();

    highRS();
//    while (1){
//    	sendSPIData(0xFF);
//    	sendSPIData(0xFF);
//    }

    unsigned int aux;

    while (1){
         for(int i=0; i<176; i++){
    		for(int j=0; j<220; j++ ){
    			if(j<220/2 && i<176/2){
    				if((j>=(220/2-5)) && (j<(220/2+6)) && (i>=(176/2-5)) && (i<(176/2+6))){
    				    aux=pacman[j-(220/2)-5] [i-(176/2)-5];
    				    sendSPIData(aux>>8);
    				    sendSPIData(aux);
    				}else{
    					sendSPIData(0x00);
    					sendSPIData(0x00);
    				}
    			}else if(j<220/2 && i>=176/2){
    				if((j>=(220/2-5)) && (j<(220/2+6)) && (i>=(176/2-5)) && (i<(176/2+6))){
    				    aux=pacman[j-(220/2)-5] [i-(176/2)-5];
    				    sendSPIData(aux>>8);
    				    sendSPIData(aux);
    				}else{
						sendSPIData(0xFF);
						sendSPIData(0xFF);
    				}
    			}else if (j>=220/2 && i<176/2){
    				if((j>=(220/2-5)) && (j<(220/2+6)) && (i>=(176/2-5)) && (i<(176/2+6))){
    				    aux=pacman[j-(220/2)-5] [i-(176/2)-5];
    				    sendSPIData(aux>>8);
    				    sendSPIData(aux);
    				}else{
						sendSPIData(0x00);
						sendSPIData(0x1F);
    				}
    			}else if (j>=220/2 && i>=176/2){
    				if((j>=(220/2-5)) && (j<(220/2+6)) && (i>=(176/2-5)) && (i<(176/2+6))){
    				    aux=pacman[j-(220/2)-5] [i-(176/2)-5];
    				    sendSPIData(aux>>8);
    				    sendSPIData(aux);
    				}else{
						sendSPIData(0x86);
						sendSPIData(0x7d);
    				}
    			}
    		}
        }
    }
}

void sendSPIData(char data){
	while (!(SPI1->SR & SPI_SR_TXE));      	// TXE = 1?, Esperar que el buffer Tx esté vacío
	SPI1->DR = data;						// Enviar Datos SPI
}
void lowSPI_CS(void){
	SPI1->CR1 &= ~SPI_CR1_SSI;           	// SSI = 0b0, Activar CS
}
void highSPI_CS(void){
	SPI1->CR1 |= SPI_CR1_SSI;           	// SSI = 0b0, Desactivar CS
}
void lowRS(void){
	GPIOB->ODR &= ~GPIO_ODR_ODR0;			// Poner PB0 en 0
}
void highRS(void){
	GPIOB->ODR |= GPIO_ODR_ODR0;			// Poner PB0 en 1
}
void lowRST(void){
	GPIOB->ODR &= ~GPIO_ODR_ODR1;			// Poner PB1 en 0
}
void highRST(void){
	GPIOB->ODR |= GPIO_ODR_ODR1;			// Poner PB1 en 1
}


void wait_ms(unsigned int time){
	SysTick->LOAD =(time*9000)-1; 							// Load=Time*8-1, ajustar valor del contador
	SysTick->CTRL |=SysTick_CTRL_ENABLE_Msk;				// Enable=1, Habilitar contador
	while(!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk));	// COUNTFLAG=1?,Preguntar si ya se alcanzo 0
	SysTick->CTRL &=~SysTick_CTRL_ENABLE_Msk;				// Enable=0, Deshabilitar el contador
}
void alreadySend_SPIData(){
	while((SPI1->SR & SPI_SR_BSY));	                       // BUSY=0?,Preguntar si está ocupado
}
