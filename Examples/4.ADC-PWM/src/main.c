#include <stm32f103xb.h>



int main(void){

	//--Inicialización interface debug---------------------------------------------------------------------

    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN; 		// AFIOEN = 1, activar reloj de modulo AFIO
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

	//--Inicialización ADC1--------------------------------------------------------------------------------

    RCC->CFGR |= RCC_CFGR_ADCPRE_1;					// ADCPRE=2, Prescaler de reloj ADC = 6, ADCCLK= 12Mhz
    RCC->APB2ENR |=RCC_APB2ENR_ADC1EN;				// ADC1EN=1, Habilitar reloj del ADC1
    RCC->APB2ENR |=RCC_APB2ENR_IOPAEN;				// IOPAEN=1, Habilitar reloj del Puerto A
	GPIOA->CRL |= ;
    ADC1->CR2 |= ADC_CR2_ADON;						// ADON=1, Encendido del conversor
	for(unsigned int i=0; i<12;i++);				// Espera de 2 ciclos de reloj de ADC (12 de sistema) para calibracion
    ADC1->CR2 |= ADC_CR2_CAL;						// CAL=1, Iniciar Calibracion
    while(ADC1->CR2 & ADC_CR2_CAL);					// CAL=0?, Esperar a que termine la calibracion
    ADC1->CR2 |= ADC_CR2_CONT;						// CONT=1, Activar modo de conversion continua
	ADC1->SQR1 &= ~ADC_SQR1_L;						// L=0, Seleccion de un solo canal para secuencia de conversion regular
    ADC1->SQR3 &= ~ADC_SQR3_SQ1;					// SQ1=0, Seleccion de primer(unico^) canal de secuencia de conversion regular


	//	Inicializacion de pines (GPIOA)

	//RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;			// IOPAEN = 1, Habilitar reloj de GPIOA
	//GPIOA->CRH |= GPIO_CRH_MODE9_0;    			// MODE9 = 01, PA9 como salida a 10 MHz
	//GPIOA->CRH |= GPIO_CRH_CNF9_1;     			// CNF9 = 10, PA9 como salida push pull alternada (salida de periferico)
	//GPIOA->CRH &= ~GPIO_CRH_MODE10;    			// MODE10 = 00, PA10 como entrada
	//GPIOA->CRH |= GPIO_CRH_CNF10_1;    			// CNF10 = 10, PA10 como entrada push pull

	//	Baudios 9600, Baudios = 72MHz/16*USARTDIV, USARTDIV = 468,75

	//USART1->BRR = (0x1D4C); 					// Parte entera y decimal del preescaler de Baudios
	//USART1->CR1 |= USART_CR1_TE;				// TE=1, Habilitar transmisor

	//	Primera transmision (vacia)

	//USART1->DR = (0x04);						// Datos TX UART1
	//while(!(USART1->SR & USART_SR_TC)); 		// TC=1? Esperar a que se complete la transmision

	//-----------------------------------------------------------------------------------------------------



	while(1){
		//USART1->DR ='H';						// Datos TX UART1
		//while(!(USART1->SR & USART_SR_TC)); 	// TC=1? Esperar a que se complete la transmision
	    //USART1->DR = 'O';						// Datos TX UART1
	    //while(!(USART1->SR & USART_SR_TC)); 	// TC=1? Esperar a que se complete la transmision
	    //USART1->DR = 'L';						// Datos TX UART1
	    //while(!(USART1->SR & USART_SR_TC)); 	// TC=1? Esperar a que se complete la transmision
	    //USART1->DR = 'A';						// Datos TX UART1
	    //while(!(USART1->SR & USART_SR_TC)); 	// TC=1? Esperar a que se complete la transmision
	    //for(int i=0; i<50000; i++);			// wait por polling
	}
}
