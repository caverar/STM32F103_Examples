#include <stm32f103xb.h>

void sendSPIData(char data);
void lowSPI_CS(void);
void highSPI_CS(void);

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



    while(1){
    	sendSPIData('H');
    	sendSPIData('o');
    	sendSPIData('l');
    	sendSPIData('a');
    }
}

void sendSPIData(char data){
	while (!(SPI1->SR & SPI_SR_TXE));      	// TXE = 1?, Esperar que el buffer Tx esté vacío
	SPI1->DR = data;						// Enviar Datos SPI
}

