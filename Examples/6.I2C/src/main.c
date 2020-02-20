#include <stm32f103xb.h>


volatile unsigned char I2C_rxBuffer[12];
unsigned int I2C_Aux;										// Variable auxiliar

void readI2C(char address, char size);
void writeI2C(char address, char size, char data[size]);

void init_VL53L0X(void);

void wait_us(unsigned int time);
unsigned short getData_VL53L0X();
int main(void){

	//--Inicialización interface debug---------------------------------------------------------------------

    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN; 					// AFIOEN = 1, activar reloj de modulo AFIO
	AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_1;						// SWJ_CFG = 010 JTAG disabled y SW-DP enable

	//--Inicializacion de reloj externo a 72 Mhz-----------------------------------------------------------

	FLASH->ACR |=FLASH_ACR_LATENCY_1;						// Ajustar Latencia de memoria Flash en 2 (Debido al ajuste de alta frecuencia)
	RCC->CR |= RCC_CR_HSEON; 								// HSE0N = 1, Habilitar reloj externo
    while(!(RCC->CR & RCC_CR_HSERDY));   					// HSERDY = 1 ?, Esperar a que el reloj externo se estabilice
    RCC->CFGR |= RCC_CFGR_PLLSRC; 							// PLLSRC=1, Seleccionar reloj externo como fuente de reloj de PLL
    RCC->CFGR |= RCC_CFGR_PLLMULL9;							// PLLMUL=7, Seleccionar 9 como factor PLL
    RCC->CR	  |= RCC_CR_PLLON;								// PLLON=1 Encender PLL
    while(!(RCC->CR & RCC_CR_PLLRDY));						// PLLRDY=1 ? Esperar a que se ajuste el PLL
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV2; 						// PPRE1=0b100, Ajustar el prescaler de APB1, para obtener 36Mhz
    RCC->CFGR |= RCC_CFGR_SW_1;    							// SW=2, PLL seleccionado como reloj del sistema
    while(!(RCC->CFGR & RCC_CFGR_SWS_PLL)); 				// SWS=2? , Esperar a que el reloj del sistema se configure

	//--Inicialización ADC1--------------------------------------------------------------------------------

	RCC->CFGR |= RCC_CFGR_ADCPRE_1;							// ADCPRE=2, Prescaler de reloj ADC = 6, ADCCLK= 12Mhz
	RCC->APB2ENR |=RCC_APB2ENR_ADC1EN;						// ADC1EN=1, Habilitar reloj del ADC1

	//  Inicilizacion de Pines GPIOA

	RCC->APB2ENR |=RCC_APB2ENR_IOPAEN;						// IOPAEN=1, Habilitar reloj del Puerto A
	GPIOA->CRH = 0;											// Limpiar el puerto A
	GPIOA->CRL &= ~GPIO_CRL_MODE0;							// MODE0=0, PA0 en modo de entrada
	GPIOA->CRL &= ~GPIO_CRL_CNF0;							// CNF0=0, PA0 en modo de entrada analoga

	// Configuracion

	ADC1->CR2 |= ADC_CR2_ADON;								// ADON=1, Encendido del conversor
	for(unsigned int i=0; i<12;i++);						// Espera de 2 ciclos de reloj de ADC (12 de sistema) para calibracion
	ADC1->CR2 |= ADC_CR2_CAL;								// CAL=1, Iniciar Calibracion
	while(ADC1->CR2 & ADC_CR2_CAL);							// CAL=0?, Esperar a que termine la calibracion
	ADC1->CR2 |= ADC_CR2_CONT;								// CONT=1, Activar modo de conversion continua
	ADC1->SQR1 &= ~ADC_SQR1_L;								// L=0, Seleccion de un solo canal para secuencia de conversion regular
	ADC1->SQR3 &= ~ADC_SQR3_SQ1;							// SQ1=0, Seleccion de primer(unico^) canal de secuencia de conversion regular
	ADC1->CR2 |= ADC_CR2_ADON;								// ADON=1, Iniciar conversion continua

	//--Inicializacion PWM (Timer 3)-----------------------------------------------------------------------
	RCC->APB1ENR |=RCC_APB1ENR_TIM3EN;						// Habilitar reloj de Timer 3

	// Ajuste de pines (pin B1-TIM3-Canal4)

	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;						// Habilitar de puerto B
	GPIOB->CRL = 0;											// Limpiar el registro
	GPIOB->CRL |= GPIO_CRL_MODE1;							// MODE1=0b11, PB1 Modo salida 50 MHz
	GPIOB->CRL |= GPIO_CRL_CNF1_1;							// CNF1=0b10, PB1 Modo alternado	 push-pull


	// COnfiguracion
	TIM3->CCER |= TIM_CCER_CC4E;							// Habilitar canal 4
	TIM3->CR1 |= TIM_CR1_ARPE;								// Habilitar Auto recarga de valor precarga
	TIM3->CCMR2|=TIM_CCMR2_OC4M_2+TIM_CCMR2_OC4M_1;			// AJustar modo PWM1 en canal 4
	TIM3->CCMR2 |= TIM_CCMR2_OC4PE;							// Habilitar precarga del ciclo de trabajo  en canal 4, del registro sombra al real
	TIM3->PSC = 7;											// Ajustar prescaler en 4
	TIM3->ARR = 1024;										// Ajustar frecuencia f=Fmax/PSC/ARR-> f=8,789Khz
	TIM3->CCR4 = 512;										// Ajustar ciclo de trabajo al 50% en canal 4
	TIM3->EGR |= TIM_EGR_UG;								// Reinicializar contador y actualizar registros
	TIM3->CR1 |= TIM_CR1_CEN;								// Habilitar contador

	//--Inicialización UART1-------------------------------------------------------------------------------

	RCC->APB2ENR |= RCC_APB2ENR_USART1EN; 					// USART1EN=1, Activación de reloj UART1
	USART1->CR1 |= USART_CR1_UE;       						// UE=1, Activación UART1
	USART1->CR1 &= ~USART_CR1_M;  							// M=0, Tamaño de palabra 8 bits
	USART1->CR2 &= ~USART_CR2_STOP;    						// STOP=0, Seleccionar el número de bits de parada

	//	Inicializacion de pines (GPIOA)

	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;						// IOPAEN = 1, Habilitar reloj de GPIOA
	GPIOA->CRH |= GPIO_CRH_MODE9_0;    						// MODE9 = 01, PA9 como salida a 10 MHz
	GPIOA->CRH |= GPIO_CRH_CNF9_1;     						// CNF9 = 10, PA9 como salida push pull alternada (salida de periferico)
	GPIOA->CRH &= ~GPIO_CRH_MODE10;    						// MODE10 = 00, PA10 como entrada
	GPIOA->CRH |= GPIO_CRH_CNF10_1;    						// CNF10 = 10, PA10 como entrada push pull

	//	Baudios 9600, Baudios = 72MHz/16*USARTDIV, USARTDIV = 468,75

	USART1->BRR = (0x1D4C); 								// Parte entera y decimal del preescaler de Baudios
	USART1->CR1 |= USART_CR1_TE;							// TE=1, Habilitar transmisor

	//	Primera transmision (vacia)

	USART1->DR = (0x04);									// Datos TX UART1
	while(!(USART1->SR & USART_SR_TC)); 					// TC=1? Esperar a que se complete la transmision
	//--Inicializacion I2C---------------------------------------------------------------------------------

	RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;						// Habilitar reloj I2C2


	// Ajuste de pines (pin B11-SDA, B10-SCL)

	GPIOB->CRH |= GPIO_CRH_MODE10;							// MODE10=0b11, PB10 Modo salida 50 MHz
	GPIOB->CRH |= GPIO_CRH_CNF10;							// CNF10=0b10, PB10 Modo alternado open drain
	GPIOB->ODR |= GPIO_ODR_ODR10;							// ODR10=1 Habiliar resistencia de PullUp
	GPIOB->CRH |= GPIO_CRH_MODE11;							// MODE11=0b11, PB11 Modo salida 50 MHz
	GPIOB->CRH |= GPIO_CRH_CNF11;							// CNF11=0b10, PB11 Modo alternado open drain
	GPIOB->ODR |= GPIO_ODR_ODR11;							// ODR11=1 Habiliar resistencia de PullUp

	// Configuracion de DMA
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;						// HAbilitar reloj DMA
	//DMA1_Channel5->CMAR = (int)I2C_rxBuffer;				// Direccion de memoria
	//DMA1_Channel5->CPAR = (int)&I2C2->DR;					// Direccion de periferico
	//DMA1_Channel5->CNDTR = 3;								// Numero de Bytes a ser transferidos
	//DMA1_Channel5->CCR |= DMA_CCR_TCIE;						// Habilitar interrupcion de transferencia completada
	//DMA1_Channel5->CCR |= DMA_CCR_MINC;						// Activar incremento de memoria CMAR, cada vez que se realize una transferencia
	//DMA1_Channel5->CCR |= DMA_CCR_CIRC;						// Activar modo circular
	//DMA1_Channel5->CCR |= DMA_CCR_EN;						// Habilitar canal DMA



	// Configuracion de periferico
	//I2C2->CCR |= I2C_CCR_FS;								// ACtivar modo I2C de alta velocidad
	I2C2->CR2 |= 36;										// Frecuencia de Bus APB1
	I2C2->CCR |= 180;										// Ajuste de frecuencia i2c a 400 khz
	I2C2->TRISE |= 37;										// y otros ajustes de velocidad

	I2C2->CR1 |= I2C_CR1_ACK;								// Enable ACK
	I2C2->CR2 |= I2C_CR2_DMAEN;								// Habilitar llamado a DMA
	//I2C2->CR2 |= I2C_CR2_LAST;								// Habilitar NACK, cuando se llena el buffer DMA
	I2C2->CR1 |= I2C_CR1_PE;								// Habilitar periferico

//-----------------------------------------------------------------------------------------------------

	init_VL53L0X();
	unsigned short value=0;

	while(1){
		value=getData_VL53L0X();

		USART1->DR ='L';										// Datos TX UART1
		while(!(USART1->SR & USART_SR_TC)); 					// TC=1? Esperar a que se complete la transmision
		USART1->DR =':';										// Datos TX UART1
		while(!(USART1->SR & USART_SR_TC)); 					// TC=1? Esperar a que se complete la transmision
		USART1->DR =' ';										// Datos TX UART1
		while(!(USART1->SR & USART_SR_TC)); 					// TC=1? Esperar a que se complete la transmision

		USART1->DR = (value/1000)+48;							// Datos TX UART1
		while(!(USART1->SR & USART_SR_TC)); 					// TC=1? Esperar a que se complete la transmision
		USART1->DR = ((value%1000)/100)+48;						// Datos TX UART1
		while(!(USART1->SR & USART_SR_TC)); 					// TC=1? Esperar a que se complete la transmision
		USART1->DR = (((value%1000)%100)/10)+48;				// Datos TX UART1
		while(!(USART1->SR & USART_SR_TC)); 					// TC=1? Esperar a que se complete la transmision
		USART1->DR = (((value%1000)%100)%10)+48;				// Datos TX UART1
		while(!(USART1->SR & USART_SR_TC)); 					// TC=1? Esperar a que se complete la transmision


		USART1->DR =13;											// Datos TX UART1
		while(!(USART1->SR & USART_SR_TC)); 					// TC=1? Esperar a que se complete la transmision
		USART1->DR =10;											// Datos TX UART1
		while(!(USART1->SR & USART_SR_TC)); 					// TC=1? Esperar a que se complete la transmision

		//while(!(ADC1->SR & ADC_SR_EOC));						// EOC=1? Preguntar si ya termino la conversion
		//value=ADC1->DR & ADC_DR_DATA;							// :DATA, Almacenar los datos de la conversion
		//TIM3->CCR4 = value/4;
		//readI2C(0x00,3);										// Lectura I2C de datos del sensor
		//readI2C(0x01,2);
		//USART1->DR =I2C_rxBuffer[0];							// Datos TX UART1
		//while(!(USART1->SR & USART_SR_TC)); 					// TC=1? Esperar a que se complete la transmision

	}
}
char txI2C_Data[10];
void init_VL53L0X(void){

	// Proceso de inicializacion 1
	readI2C(0x89, 2); 	//1
	txI2C_Data[0]=0x01;
	writeI2C(0x89, 1, txI2C_Data);
	txI2C_Data[0]=0x00;
	writeI2C(0x88, 1, txI2C_Data);
	readI2C(0xC0, 2);	//1
	readI2C(0xF8, 2);
	readI2C(0x04, 4);
	readI2C(0x20, 2);
	readI2C(0x28, 2);
	readI2C(0x44, 2);
	readI2C(0x64, 2);
	readI2C(0x64, 2);
	readI2C(0x01, 2);	//1
	readI2C(0x01, 2);	//1
	readI2C(0x50, 2);	//1
	readI2C(0x46, 2);	//1

	wait_us(160);
	wait_us(5);

	readI2C(0x50, 2);	//1
	readI2C(0x50, 2);	//1
	readI2C(0x51, 2);

	wait_us(180);
	wait_us(5);

	readI2C(0x01, 2);	//1
	readI2C(0x50, 2);	//1
	readI2C(0x51, 2);
	readI2C(0x70, 2);	//1
	readI2C(0x71, 2);	//

	wait_us(200);
	wait_us(5);

	txI2C_Data[0]=0x01;
	writeI2C(0x80, 1, txI2C_Data);
	txI2C_Data[0]=0x01;
	writeI2C(0xFF, 1, txI2C_Data);
	txI2C_Data[0]=0x00;
	writeI2C(0x00, 1, txI2C_Data);
	readI2C(0x91, 2);	//1
	txI2C_Data[0]=0x01;
	writeI2C(0x00, 1, txI2C_Data);
	txI2C_Data[0]=0x00;
	writeI2C(0xFF, 1, txI2C_Data);
	txI2C_Data[0]=0x00;
	writeI2C(0x80, 1, txI2C_Data);
	txI2C_Data[0]=0x00;
	txI2C_Data[1]=0x20;
	writeI2C(0x44, 2, txI2C_Data);
	readI2C(0x60, 2);	//1
	txI2C_Data[0]=0x00;
	writeI2C(0x60, 1, txI2C_Data);
	readI2C(0x60, 2);	//1
	txI2C_Data[0]=0x00;
	writeI2C(0x60, 1, txI2C_Data);
	readI2C(0x60, 2);	//1
	txI2C_Data[0]=0x02;
	writeI2C(0x60, 1, txI2C_Data);
	readI2C(0x60, 2);	//1
	txI2C_Data[0]=0x12;
	writeI2C(0x60, 1, txI2C_Data);
	txI2C_Data[0]=0x00;
	txI2C_Data[1]=0x20;
	writeI2C(0x44, 2, txI2C_Data);
	txI2C_Data[0]=0xFF;
	writeI2C(0x01, 1, txI2C_Data);
	txI2C_Data[0]=0x29;
	writeI2C(0x8A, 1, txI2C_Data);

	for(int i =0; i<100; i++){
		wait_us(100);
	}
	wait_us(5);

	// Proceso de inicializacion 2

	txI2C_Data[0]=0x01;
	writeI2C(0x80, 1, txI2C_Data);
	txI2C_Data[0]=0x01;
	writeI2C(0xFF, 1, txI2C_Data);
	txI2C_Data[0]=0x00;
	writeI2C(0x00, 1, txI2C_Data);
	txI2C_Data[0]=0x06;
	writeI2C(0xFF, 1, txI2C_Data);
	readI2C(0x83, 2);	//1
	txI2C_Data[0]=0x05;
	writeI2C(0x83, 1, txI2C_Data);
	txI2C_Data[0]=0x07;
	writeI2C(0xFF, 1, txI2C_Data);
	txI2C_Data[0]=0x01;
	writeI2C(0x81, 1, txI2C_Data);

	for(int i =0; i<6; i++){
		wait_us(100);
	}
	wait_us(5);

	txI2C_Data[0]=0x01;
	writeI2C(0x80, 1, txI2C_Data);
	txI2C_Data[0]=0x02;
	writeI2C(0x94, 1, txI2C_Data);
	txI2C_Data[0]=0x00;
	writeI2C(0x83, 1, txI2C_Data);
	readI2C(0x83, 2);	//1
	txI2C_Data[0]=0x01;
	writeI2C(0x83, 1, txI2C_Data);
	readI2C(0x90, 2);	//1
	txI2C_Data[0]=0x7B;
	writeI2C(0x94, 1, txI2C_Data);
	txI2C_Data[0]=0x00;
	writeI2C(0x83, 1, txI2C_Data);
	readI2C(0x83, 2);	//1
	txI2C_Data[0]=0x01;
	writeI2C(0x83, 1, txI2C_Data);
	readI2C(0x90, 2);	//1
	txI2C_Data[0]=0x77;
	writeI2C(0x94, 1, txI2C_Data);
	txI2C_Data[0]=0x00;
	writeI2C(0x83, 1, txI2C_Data);
	readI2C(0x83, 2);	//1
	txI2C_Data[0]=0x01;
	writeI2C(0x83, 1, txI2C_Data);
	readI2C(0x90, 4);

	wait_us(75);
	wait_us(5);

	txI2C_Data[0]=0x78;
	writeI2C(0x94, 1, txI2C_Data);
	txI2C_Data[0]=0x00;
	writeI2C(0x83, 1, txI2C_Data);
	readI2C(0x83, 2);	//1
	txI2C_Data[0]=0x01;
	writeI2C(0x83, 1, txI2C_Data);
	readI2C(0x90, 4);

	wait_us(75);
	wait_us(5);
	txI2C_Data[0]=0x79;
	writeI2C(0x94, 1, txI2C_Data);
	txI2C_Data[0]=0x00;
	writeI2C(0x83, 1, txI2C_Data);
	readI2C(0x83, 2);	//1
	txI2C_Data[0]=0x01;
	writeI2C(0x83, 1, txI2C_Data);
	readI2C(0x90, 4);
	txI2C_Data[0]=0x74;
	writeI2C(0x94, 1, txI2C_Data);
	txI2C_Data[0]=0x00;
	writeI2C(0x83, 1, txI2C_Data);
	readI2C(0x83, 2);	//1
	txI2C_Data[0]=0x01;
	writeI2C(0x83, 1, txI2C_Data);
	readI2C(0x90, 4);

	wait_us(75);
	wait_us(5);

	txI2C_Data[0]=0x00;
	writeI2C(0x81, 1, txI2C_Data);
	txI2C_Data[0]=0x06;
	writeI2C(0xFF, 1, txI2C_Data);
	readI2C(0x83, 2);	//1
	txI2C_Data[0]=0x01;
	writeI2C(0x83, 1, txI2C_Data);
	txI2C_Data[0]=0x01;
	writeI2C(0xFF, 1, txI2C_Data);
	txI2C_Data[0]=0x01;
	writeI2C(0x00, 1, txI2C_Data);
	txI2C_Data[0]=0x00;
	writeI2C(0xFF, 1, txI2C_Data);
	txI2C_Data[0]=0x00;
	writeI2C(0x80, 1, txI2C_Data);
	readI2C(0xC0, 2);	//1
	readI2C(0xC2, 2);	//1

	wait_us(75);
	wait_us(5);

	txI2C_Data[0]=0x01;
	writeI2C(0x80, 1, txI2C_Data);
	txI2C_Data[0]=0x01;
	writeI2C(0xFF, 1, txI2C_Data);
	txI2C_Data[0]=0x00;
	writeI2C(0x00, 1, txI2C_Data);
	txI2C_Data[0]=0x06;
	writeI2C(0xFF, 1, txI2C_Data);
	readI2C(0x83, 2);	//1
	txI2C_Data[0]=0x05;
	writeI2C(0x83, 1, txI2C_Data);
	txI2C_Data[0]=0x07;
	writeI2C(0xFF, 1, txI2C_Data);
	txI2C_Data[0]=0x01;
	writeI2C(0x81, 1, txI2C_Data);


	for(int i =0; i<6; i++){
		wait_us(100);
	}
	wait_us(5);

	txI2C_Data[0]=0x01;
	writeI2C(0x80, 1, txI2C_Data);
	txI2C_Data[0]=0x6B;
	writeI2C(0x94, 1, txI2C_Data);
	txI2C_Data[0]=0x00;
	writeI2C(0x83, 1, txI2C_Data);
	readI2C(0x83, 2);	//1
	txI2C_Data[0]=0x01;
	writeI2C(0x83, 1, txI2C_Data);
	readI2C(0x90, 4);
	txI2C_Data[0]=0x24;
	writeI2C(0x94, 1, txI2C_Data);
	txI2C_Data[0]=0x00;
	writeI2C(0x83, 1, txI2C_Data);
	readI2C(0x83, 2);	//1
	txI2C_Data[0]=0x01;
	writeI2C(0x83, 1, txI2C_Data);
	readI2C(0x90, 4);
	txI2C_Data[0]=0x25;
	writeI2C(0x94, 1, txI2C_Data);
	txI2C_Data[0]=0x00;
	writeI2C(0x83, 1, txI2C_Data);
	readI2C(0x83, 2);	//1
	txI2C_Data[0]=0x01;
	writeI2C(0x83, 1, txI2C_Data);
	readI2C(0x90, 4);
	txI2C_Data[0]=0x00;
	writeI2C(0x81, 1, txI2C_Data);
	txI2C_Data[0]=0x06;
	writeI2C(0xFF, 1, txI2C_Data);
	readI2C(0x83, 2);	//1
	txI2C_Data[0]=0x01;
	writeI2C(0x83, 1, txI2C_Data);
	txI2C_Data[0]=0x01;
	writeI2C(0xFF, 1, txI2C_Data);
	txI2C_Data[0]=0x01;
	writeI2C(0x00, 1, txI2C_Data);
	txI2C_Data[0]=0x00;
	writeI2C(0xFF, 1, txI2C_Data);
	txI2C_Data[0]=0x00;
	writeI2C(0x80, 1, txI2C_Data);
	txI2C_Data[0]=0x01;
	writeI2C(0xFF, 1, txI2C_Data);
	txI2C_Data[0]=0x00;
	writeI2C(0x4F, 1, txI2C_Data);
	txI2C_Data[0]=0x2C;
	writeI2C(0x4E, 1, txI2C_Data);
	txI2C_Data[0]=0x00;
	writeI2C(0xFF, 1, txI2C_Data);
	txI2C_Data[0]=0xB4;
	writeI2C(0xB6, 1, txI2C_Data);

	for(int i =0; i<2; i++){
		wait_us(100);
	}
	wait_us(5);
	txI2C_Data[0]=0x3F;
	txI2C_Data[1]=0x00;
	txI2C_Data[2]=0x00;
	txI2C_Data[3]=0x00;
	txI2C_Data[4]=0x00;
	txI2C_Data[5]=0x00;
	writeI2C(0xB0, 6, txI2C_Data);
	readI2C(0xB0, 6);
	txI2C_Data[0]=0x01;
	writeI2C(0xFF, 1, txI2C_Data);
	txI2C_Data[0]=0x00;
	writeI2C(0x00, 1, txI2C_Data); 	//1638150us
	txI2C_Data[0]=0x00;
	writeI2C(0xFF, 1, txI2C_Data);
	txI2C_Data[0]=0x00;
	writeI2C(0x09, 1, txI2C_Data);
	txI2C_Data[0]=0x00;
	writeI2C(0x10, 1, txI2C_Data);
	txI2C_Data[0]=0x00;
	writeI2C(0x11, 1, txI2C_Data);
	txI2C_Data[0]=0x01;
	writeI2C(0x24, 1, txI2C_Data);
	txI2C_Data[0]=0xFF;
	writeI2C(0x25, 1, txI2C_Data);
	txI2C_Data[0]=0x00;
	writeI2C(0x75, 1, txI2C_Data);
	txI2C_Data[0]=0x01;
	writeI2C(0xFF, 1, txI2C_Data);
	txI2C_Data[0]=0x2C;
	writeI2C(0x4E, 1, txI2C_Data);
	txI2C_Data[0]=0x00;
	writeI2C(0x48, 1, txI2C_Data);
	txI2C_Data[0]=0x20;
	writeI2C(0x30, 1, txI2C_Data);
	txI2C_Data[0]=0x00;
	writeI2C(0xFF, 1, txI2C_Data);
	txI2C_Data[0]=0x09;
	writeI2C(0x30, 1, txI2C_Data);
	txI2C_Data[0]=0x00;
	writeI2C(0x54, 1, txI2C_Data);
	txI2C_Data[0]=0x04;
	writeI2C(0x31, 1, txI2C_Data);
	txI2C_Data[0]=0x03;
	writeI2C(0x32, 1, txI2C_Data);
	txI2C_Data[0]=0x83;
	writeI2C(0x40, 1, txI2C_Data);
	txI2C_Data[0]=0x25;
	writeI2C(0x46, 1, txI2C_Data);
	txI2C_Data[0]=0x00;
	writeI2C(0x60, 1, txI2C_Data);
	txI2C_Data[0]=0x00;
	writeI2C(0x27, 1, txI2C_Data);
	txI2C_Data[0]=0x06;
	writeI2C(0x50, 1, txI2C_Data);
	txI2C_Data[0]=0x00;
	writeI2C(0x51, 1, txI2C_Data);
	txI2C_Data[0]=0x96;
	writeI2C(0x52, 1, txI2C_Data);
	txI2C_Data[0]=0x08;
	writeI2C(0x56, 1, txI2C_Data);
	txI2C_Data[0]=0x30;
	writeI2C(0x57, 1, txI2C_Data);
	txI2C_Data[0]=0x00;
	writeI2C(0x61, 1, txI2C_Data);
	txI2C_Data[0]=0x00;
	writeI2C(0x62, 1, txI2C_Data);
	txI2C_Data[0]=0x00;
	writeI2C(0x64, 1, txI2C_Data);
	txI2C_Data[0]=0x00;
	writeI2C(0x65, 1, txI2C_Data);
	txI2C_Data[0]=0xA0;
	writeI2C(0x66, 1, txI2C_Data);
	txI2C_Data[0]=0x01;
	writeI2C(0xFF, 1, txI2C_Data);
	txI2C_Data[0]=0x32;
	writeI2C(0x22, 1, txI2C_Data);
	txI2C_Data[0]=0x14;
	writeI2C(0x47, 1, txI2C_Data);
	txI2C_Data[0]=0xFF;
	writeI2C(0x49, 1, txI2C_Data);
	txI2C_Data[0]=0x00;
	writeI2C(0x4A, 1, txI2C_Data);
	txI2C_Data[0]=0x00;
	writeI2C(0xFF, 1, txI2C_Data);
	txI2C_Data[0]=0x0A;
	writeI2C(0x7A, 1, txI2C_Data);
	txI2C_Data[0]=0x00;
	writeI2C(0x7B, 1, txI2C_Data);	//1650800
	txI2C_Data[0]=0x21;
	writeI2C(0x78, 1, txI2C_Data);
	txI2C_Data[0]=0x01;
	writeI2C(0xFF, 1, txI2C_Data);
	txI2C_Data[0]=0x34;
	writeI2C(0x23, 1, txI2C_Data);
	txI2C_Data[0]=0x00;
	writeI2C(0x42, 1, txI2C_Data);
	txI2C_Data[0]=0xFF;
	writeI2C(0x44, 1, txI2C_Data);
	txI2C_Data[0]=0x26;
	writeI2C(0x45, 1, txI2C_Data);
	txI2C_Data[0]=0x05;
	writeI2C(0x46, 1, txI2C_Data);
	txI2C_Data[0]=0x40;
	writeI2C(0x40, 1, txI2C_Data);
	txI2C_Data[0]=0x06;
	writeI2C(0x0E, 1, txI2C_Data);
	txI2C_Data[0]=0x1A;
	writeI2C(0x20, 1, txI2C_Data);
	txI2C_Data[0]=0x40;
	writeI2C(0x43, 1, txI2C_Data);
	txI2C_Data[0]=0x00;
	writeI2C(0xFF, 1, txI2C_Data);
	txI2C_Data[0]=0x03;
	writeI2C(0x34, 1, txI2C_Data);
	txI2C_Data[0]=0x44;
	writeI2C(0x35, 1, txI2C_Data);
	txI2C_Data[0]=0x01;
	writeI2C(0xFF, 1, txI2C_Data);
	txI2C_Data[0]=0x04;
	writeI2C(0x31, 1, txI2C_Data);
	txI2C_Data[0]=0x09;
	writeI2C(0x4B, 1, txI2C_Data);
	txI2C_Data[0]=0x05;
	writeI2C(0x4C, 1, txI2C_Data);
	txI2C_Data[0]=0x04;
	writeI2C(0x4D, 1, txI2C_Data);
	txI2C_Data[0]=0x00;
	writeI2C(0xFF, 1, txI2C_Data);
	txI2C_Data[0]=0x00;
	writeI2C(0x44, 1, txI2C_Data);
	txI2C_Data[0]=0x20;
	writeI2C(0x45, 1, txI2C_Data);
	txI2C_Data[0]=0x08;
	writeI2C(0x47, 1, txI2C_Data);
	txI2C_Data[0]=0x28;
	writeI2C(0x48, 1, txI2C_Data);
	txI2C_Data[0]=0x00;
	writeI2C(0x67, 1, txI2C_Data);
	txI2C_Data[0]=0x04;
	writeI2C(0x70, 1, txI2C_Data);
	txI2C_Data[0]=0x01;
	writeI2C(0x71, 1, txI2C_Data);
	txI2C_Data[0]=0xFE;
	writeI2C(0x72, 1, txI2C_Data);
	txI2C_Data[0]=0x00;
	writeI2C(0x76, 1, txI2C_Data);
	txI2C_Data[0]=0x00;
	writeI2C(0x77, 1, txI2C_Data);
	txI2C_Data[0]=0x01;
	writeI2C(0xFF, 1, txI2C_Data);
	txI2C_Data[0]=0x01;
	writeI2C(0x0D, 1, txI2C_Data);
	txI2C_Data[0]=0x00;
	writeI2C(0xFF, 1, txI2C_Data);
	txI2C_Data[0]=0x01;
	writeI2C(0x80, 1, txI2C_Data);
	txI2C_Data[0]=0xF8;
	writeI2C(0x01, 1, txI2C_Data);
	txI2C_Data[0]=0x01;
	writeI2C(0xFF, 1, txI2C_Data);
	txI2C_Data[0]=0x01;
	writeI2C(0x8E, 1, txI2C_Data);
	txI2C_Data[0]=0x01;
	writeI2C(0x00, 1, txI2C_Data);
	txI2C_Data[0]=0x00;
	writeI2C(0xFF, 1, txI2C_Data);
	txI2C_Data[0]=0x00;
	writeI2C(0x80, 1, txI2C_Data);
	txI2C_Data[0]=0x04;
	writeI2C(0x0A, 1, txI2C_Data);
	readI2C(0x84, 2);	//1
	txI2C_Data[0]=0x01;
	writeI2C(0x84, 1, txI2C_Data);
	txI2C_Data[0]=0x01;
	writeI2C(0x0B, 1, txI2C_Data);
	txI2C_Data[0]=0x00;
	writeI2C(0x0B, 1, txI2C_Data);
	readI2C(0x13, 2);	//1
	txI2C_Data[0]=0x01;
	writeI2C(0xFF, 1, txI2C_Data);
	readI2C(0x84, 2);
	txI2C_Data[0]=0x00;
	writeI2C(0xFF, 1, txI2C_Data);
	readI2C(0xF8, 2);
	readI2C(0x04, 4);

	wait_us(90);
	wait_us(5);

	readI2C(0x20, 2);
	readI2C(0x28, 2);
	readI2C(0x44, 2);
	readI2C(0x64, 2);
	readI2C(0x64, 2);
	readI2C(0x01, 2);	//1
	readI2C(0x01, 2);	//1
	readI2C(0x50, 2);	//1
	readI2C(0x46, 2);	//1

	wait_us(180);
	wait_us(5);

	readI2C(0x50, 2);	//1
	readI2C(0x50, 2);	//1
	readI2C(0x51, 2);

	wait_us(180);
	wait_us(5);

	readI2C(0x01, 2);	//1
	readI2C(0x50, 2);	//1
	readI2C(0x51, 2);
	readI2C(0x70, 2);	//1
	readI2C(0x71, 2);

	wait_us(180);
	wait_us(5);

	readI2C(0x09, 2);	//1
	readI2C(0x01, 2);	//1
	readI2C(0x01, 2);	//1
	txI2C_Data[0]=0xE8;
	writeI2C(0x01, 1, txI2C_Data);
	readI2C(0x01, 2);	//1
	readI2C(0x50, 2);	//1
	readI2C(0x46, 2);	//1

	wait_us(180);
	wait_us(5);

	readI2C(0x50, 2);	//1
	readI2C(0x50, 2);	//1
	readI2C(0x51, 2);

	wait_us(180);
	wait_us(5);

	readI2C(0x01, 2);	//1
	readI2C(0x50, 2);	//1
	readI2C(0x51, 2);
	readI2C(0x70, 2);	//1

	wait_us(180);
	wait_us(5);

	txI2C_Data[0]=0x02;
	txI2C_Data[0]=0x94;
	writeI2C(0x71, 2, txI2C_Data);
	readI2C(0x01, 2);	//1
	readI2C(0x50, 2);	//1
	readI2C(0x70, 2);	//1
	readI2C(0x50, 2);	//1
	readI2C(0x50, 2);	//1
	readI2C(0x51, 2);

	wait_us(180);
	wait_us(5);

	readI2C(0x01, 2);	//1
	readI2C(0x50, 2);	//1
	readI2C(0x51, 2);
	readI2C(0x70, 2);	//1
	readI2C(0x71, 2);

	wait_us(260);
	wait_us(5);

	txI2C_Data[0]=0x01;
	writeI2C(0xFF, 1, txI2C_Data);
	txI2C_Data[0]=0x00;
	writeI2C(0x4F, 1, txI2C_Data);
	txI2C_Data[0]=0x2C;
	writeI2C(0x4E, 1, txI2C_Data);
	txI2C_Data[0]=0x00;
	writeI2C(0xFF, 1, txI2C_Data);
	txI2C_Data[0]=0xB4;
	writeI2C(0xB6, 1, txI2C_Data);
	txI2C_Data[0]=0x00;
	writeI2C(0x80, 1, txI2C_Data);
	txI2C_Data[0]=0x01;
	writeI2C(0x01, 1, txI2C_Data);
	txI2C_Data[0]=0x41;
	writeI2C(0x00, 1, txI2C_Data);

	readI2C(0x13, 2);	//1
	for(int i =0; i<6; i++) wait_us(100);
	wait_us(5);

	while(I2C_rxBuffer[0]==0x40){								// Preguntar si es 0x44
		for(int i =0; i<6; i++) wait_us(100);
		wait_us(5);
		readI2C(0x13, 2);	//1
	}

	txI2C_Data[0]=0x01;
	writeI2C(0x0B, 1, txI2C_Data);
	txI2C_Data[0]=0x00;
	writeI2C(0x0B, 1, txI2C_Data);
	readI2C(0x13, 2);	//1
	txI2C_Data[0]=0x00;
	writeI2C(0x00, 1, txI2C_Data);
	txI2C_Data[0]=0x02;
	writeI2C(0x01, 1, txI2C_Data);
	txI2C_Data[0]=0x01;
	writeI2C(0x00, 1, txI2C_Data);

	readI2C(0x13, 2);	//1
	for(int i =0; i<6; i++) wait_us(100);
	wait_us(5);

	while(I2C_rxBuffer[0]==0x40){								// Preguntar si es 0x44
		for(int i =0; i<6; i++) wait_us(100);
		wait_us(5);
		readI2C(0x13, 2);	//1
	}

	txI2C_Data[0]=0x01;
	writeI2C(0x0B, 1, txI2C_Data);
	txI2C_Data[0]=0x00;
	writeI2C(0x0B, 1, txI2C_Data);
	readI2C(0x13, 2);	//1
	txI2C_Data[0]=0x00;
	writeI2C(0x00, 1, txI2C_Data);
	txI2C_Data[0]=0xE8;
	writeI2C(0x01, 1, txI2C_Data);

	for(int i =0; i<2; i++) wait_us(100);
	wait_us(5);

	txI2C_Data[0]=0x07;
	txI2C_Data[1]=0x00;
	txI2C_Data[2]=0x00;
	txI2C_Data[3]=0x00;
	txI2C_Data[4]=0x00;
	txI2C_Data[5]=0x00;
	writeI2C(0xB0, 6, txI2C_Data);
	readI2C(0xB0, 6);
	txI2C_Data[0]=0xC0;
	writeI2C(0x01, 1, txI2C_Data);
	txI2C_Data[0]=0x01;
	writeI2C(0x80, 1, txI2C_Data);
	txI2C_Data[0]=0x01;
	writeI2C(0xFF, 1, txI2C_Data);
	txI2C_Data[0]=0x00;
	writeI2C(0x00, 1, txI2C_Data);
	txI2C_Data[0]=0x3C;
	writeI2C(0x91, 1, txI2C_Data);
	txI2C_Data[0]=0x01;
	writeI2C(0x00, 1, txI2C_Data);
	txI2C_Data[0]=0x00;
	writeI2C(0xFF, 1, txI2C_Data);
	txI2C_Data[0]=0x00;
	writeI2C(0x80, 1, txI2C_Data);
	txI2C_Data[0]=0x01;
	writeI2C(0x00, 1, txI2C_Data);
	readI2C(0x00, 2);	//1

	readI2C(0x13, 2);	//1
	for(int i =0; i<6; i++) wait_us(100);
	wait_us(5);

	while(I2C_rxBuffer[0]==0x40){								// Preguntar si es 0x44
		for(int i =0; i<6; i++) wait_us(100);
		wait_us(5);
		readI2C(0x13, 2);	//1
	}

	readI2C(0x14, 12);
	txI2C_Data[0]=0x01;
	writeI2C(0xFF, 1, txI2C_Data);
	readI2C(0xB6, 2);
	txI2C_Data[0]=0x00;
	writeI2C(0xFF, 1, txI2C_Data);
	txI2C_Data[0]=0x01;
	writeI2C(0x0B, 1, txI2C_Data);
	txI2C_Data[0]=0x00;
	writeI2C(0x0B, 1, txI2C_Data);
	readI2C(0x13, 2);	//1
	txI2C_Data[0]=0x01;
	writeI2C(0xFF, 1, txI2C_Data);
	readI2C(0xB6, 2);
	txI2C_Data[0]=0x00;
	writeI2C(0xFF, 1, txI2C_Data);
	txI2C_Data[0]=0xE8;
	writeI2C(0x01, 1, txI2C_Data);
	txI2C_Data[0]=0x0F;
	txI2C_Data[1]=0x00;
	txI2C_Data[2]=0x00;
	txI2C_Data[3]=0x00;
	txI2C_Data[4]=0x00;
	txI2C_Data[5]=0x00;
	writeI2C(0xB0, 6, txI2C_Data);
	txI2C_Data[0]=0xC0;
	writeI2C(0x01, 1, txI2C_Data);
	txI2C_Data[0]=0x01;
	writeI2C(0x80, 1, txI2C_Data);
	txI2C_Data[0]=0x01;
	writeI2C(0xFF, 1, txI2C_Data);
	txI2C_Data[0]=0x00;
	writeI2C(0x00, 1, txI2C_Data);
	txI2C_Data[0]=0x3C;
	writeI2C(0x91, 1, txI2C_Data);
	txI2C_Data[0]=0x01;
	writeI2C(0x00, 1, txI2C_Data);
	txI2C_Data[0]=0x00;
	writeI2C(0xFF, 1, txI2C_Data);
	txI2C_Data[0]=0x00;
	writeI2C(0x80, 1, txI2C_Data);
	txI2C_Data[0]=0x01;
	writeI2C(0x00, 1, txI2C_Data);
	readI2C(0x00, 2);	//1

	readI2C(0x13, 2);	//1
	for(int i =0; i<6; i++) wait_us(100);
	wait_us(5);

	while(I2C_rxBuffer[0]==0x40){								// Preguntar si es 0x44
		for(int i =0; i<6; i++) wait_us(100);
		wait_us(5);
		readI2C(0x13, 2);	//1
	}

	readI2C(0x14, 12);	//1788850
	txI2C_Data[0]=0x01;
	writeI2C(0xFF, 1, txI2C_Data);
	readI2C(0xB6, 2);
	txI2C_Data[0]=0x00;
	writeI2C(0xFF, 1, txI2C_Data);
	txI2C_Data[0]=0x01;
	writeI2C(0x0B, 1, txI2C_Data);
	txI2C_Data[0]=0x00;
	writeI2C(0x0B, 1, txI2C_Data);
	readI2C(0x13, 2);	//1
	txI2C_Data[0]=0x01;
	writeI2C(0xFF, 1, txI2C_Data);
	readI2C(0xB6, 2);
	txI2C_Data[0]=0x00;
	writeI2C(0xFF, 1, txI2C_Data);
	txI2C_Data[0]=0xE8;
	writeI2C(0x01, 1, txI2C_Data);
	txI2C_Data[0]=0x01;
	writeI2C(0x01, 1, txI2C_Data);
	txI2C_Data[0]=0x41;
	writeI2C(0x00, 1, txI2C_Data);

	readI2C(0x13, 2);	//1
	for(int i =0; i<6; i++) wait_us(100);
	wait_us(5);

	while(I2C_rxBuffer[0]==0x40){								// Preguntar si es 0x44
		for(int i =0; i<6; i++) wait_us(100);
		wait_us(5);
		readI2C(0x13, 2);	//1
	}

	txI2C_Data[0]=0x01;
	writeI2C(0x0B, 1, txI2C_Data);
	txI2C_Data[0]=0x00;
	writeI2C(0x0B, 1, txI2C_Data);
	readI2C(0x13, 2);	//1
	txI2C_Data[0]=0x00;
	writeI2C(0x00, 1, txI2C_Data);
	txI2C_Data[0]=0x01;
	writeI2C(0xFF, 1, txI2C_Data);
	txI2C_Data[0]=0x00;
	writeI2C(0x00, 1, txI2C_Data);
	txI2C_Data[0]=0x00;
	writeI2C(0xFF, 1, txI2C_Data);
	readI2C(0xCB, 2);	//1
	txI2C_Data[0]=0x01;
	writeI2C(0xFF, 1, txI2C_Data);
	txI2C_Data[0]=0x01;
	writeI2C(0x00, 1, txI2C_Data);
	txI2C_Data[0]=0x00;
	writeI2C(0xFF, 1, txI2C_Data);
	txI2C_Data[0]=0x02;
	writeI2C(0x01, 1, txI2C_Data);
	txI2C_Data[0]=0x01;
	writeI2C(0x00, 1, txI2C_Data);

	readI2C(0x13, 2);	//1
	for(int i =0; i<6; i++) wait_us(100);
	wait_us(5);

	while(I2C_rxBuffer[0]==0x40){								// Preguntar si es 0x44
		for(int i =0; i<6; i++) wait_us(100);
		wait_us(5);
		readI2C(0x13, 2);	//1
	}

	txI2C_Data[0]=0x01;
	writeI2C(0x0B, 1, txI2C_Data);
	txI2C_Data[0]=0x00;
	writeI2C(0x0B, 1, txI2C_Data);
	readI2C(0x13, 2);	//1
	txI2C_Data[0]=0x00;
	writeI2C(0x00, 1, txI2C_Data);
	txI2C_Data[0]=0x01;
	writeI2C(0xFF, 1, txI2C_Data);
	txI2C_Data[0]=0x00;
	writeI2C(0x00, 1, txI2C_Data);
	txI2C_Data[0]=0x00;
	writeI2C(0xFF, 1, txI2C_Data);
	readI2C(0xEE, 2);	//1
	txI2C_Data[0]=0x01;
	writeI2C(0xFF, 1, txI2C_Data);
	txI2C_Data[0]=0x01;
	writeI2C(0x00, 1, txI2C_Data);
	txI2C_Data[0]=0x00;
	writeI2C(0xFF, 1, txI2C_Data);
	txI2C_Data[0]=0xE8;
	writeI2C(0x01, 1, txI2C_Data);
	txI2C_Data[0]=0x00;
	txI2C_Data[1]=0x20;
	writeI2C(0x44, 2, txI2C_Data);

}
unsigned short getData_VL53L0X(){
	txI2C_Data[0]=0x01;
	writeI2C(0x80, 1, txI2C_Data);
	txI2C_Data[0]=0x01;
	writeI2C(0xFF, 1, txI2C_Data);
	txI2C_Data[0]=0x00;
	writeI2C(0x00, 1, txI2C_Data);
	txI2C_Data[0]=0x3C;
	writeI2C(0x91, 1, txI2C_Data);
	txI2C_Data[0]=0x01;
	writeI2C(0x00, 1, txI2C_Data);
	txI2C_Data[0]=0x00;
	writeI2C(0xFF, 1, txI2C_Data);
	txI2C_Data[0]=0x00;
	writeI2C(0x80, 1, txI2C_Data);
	txI2C_Data[0]=0x01;
	writeI2C(0x00, 1, txI2C_Data);

	readI2C(0x00, 2);	//1
	readI2C(0x13, 2);	//1

	for(int i =0; i<6; i++) wait_us(100);
	wait_us(5);

	while(I2C_rxBuffer[0]==0x40){								// Preguntar si es 0x44
		for(int i =0; i<6; i++) wait_us(100);
		wait_us(5);
		readI2C(0x13, 2);	//1
	}

	readI2C(0x14, 12);
	short returnValue=(I2C_rxBuffer[10]<<8) + I2C_rxBuffer[11];
	txI2C_Data[0]=0x01;
	writeI2C(0xFF, 1, txI2C_Data);
	readI2C(0xB6, 2);
	txI2C_Data[0]=0x00;
	writeI2C(0xFF, 1, txI2C_Data);
	txI2C_Data[0]=0x01;
	writeI2C(0x0B, 1, txI2C_Data);
	txI2C_Data[0]=0x00;
	writeI2C(0x0B, 1, txI2C_Data);
	readI2C(0x13, 2);	//1
	return returnValue;
}

void wait_us(unsigned int time){ //455 valor maximo
	SysTick->LOAD =(time*9)-1; 								// Load=Time*8-1, ajustar valor del contador
	SysTick->CTRL |=SysTick_CTRL_ENABLE_Msk;				// Enable=1, Habilitar contador
	while(!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk));	// COUNTFLAG=1?,Preguntar si ya se alcanzo 0
	SysTick->CTRL &=~SysTick_CTRL_ENABLE_Msk;				// Enable=0, Deshabilitar el contador
}

void writeI2C(char address, char size, char data[size]){
	I2C2->CR1 |= I2C_CR1_START;								// Secuencia de inicio
	while(!(I2C2->SR1 & I2C_SR1_SB));						// SB=1?, Esperar a que se complete la secuencia de inicio
	I2C2->DR = 0x52;										// Escribir Direccion de escritura de périferico
	while(!(I2C2->SR1 & I2C_SR1_ADDR));						// ADDR=1?, ACK de direccion de periferico
	I2C_Aux= I2C2->SR2;										// Lectura basura para limpiar ADDR
	I2C2->DR = address;										// Escribir Direccion de registro de périferico
	while(!(I2C2->SR1 & I2C_SR1_TXE));						// TX=1?, Esperar a que el registro de datos TX este vacio (transmision)

	for(int i=0; i<size;i++){
		I2C2->DR = data[i];									// Escribir dato
		while(!(I2C2->SR1 & I2C_SR1_TXE));					// TX=1?, Esperar a que el registro de datos TX este vacio (transmision)
	}
	I2C2->CR1 |= I2C_CR1_STOP;								// Secuencia de parada
	wait_us(4);												// Espera de 4 us
}

void readI2C(char address, char size){

	if(size==1){

		I2C2->CR1 |= I2C_CR1_START;							// Secuencia de inicio
		while(!(I2C2->SR1 & I2C_SR1_SB));					// SB=1?, Esperar a que se complete la secuencia de inicio
		I2C2->DR = 0x52;									// Escribir Direccion de escritura de périferico
		while(!(I2C2->SR1 & I2C_SR1_ADDR));					// ADDR=1?, ACK de direccion de periferico
		I2C_Aux= I2C2->SR2;									// Lectura basura para limpiar ADDR
		I2C2->DR = address;									// Escribir Direccion de registro de périferico
		while(!(I2C2->SR1 & I2C_SR1_TXE));					// TX=1?, Esperar a que el registro de datos TX este vacio (transmision)
		I2C2->CR1 |= I2C_CR1_STOP;							// Secuencia de parada
		wait_us(4);											// Espera de 4 us

		// Reajuste DMA
		DMA1_Channel5->CMAR = (int)I2C_rxBuffer;			// Direccion de memoria
		DMA1_Channel5->CPAR = (int)&I2C2->DR;				// Direccion de periferico
		DMA1_Channel5->CNDTR = size;						// Numero de Bytes a ser transferidos
		DMA1_Channel5->CCR |= DMA_CCR_TCIE;					// Habilitar interrupcion de transferencia completada
		//DMA1_Channel5->CCR &= !DMA_CCR_MINC;				// Activar incremento de memoria CMAR, cada vez que se realize una transferencia
		//DMA1_Channel5->CCR |= DMA_CCR_CIRC;				// Activar modo circular
		DMA1_Channel5->CCR |= DMA_CCR_EN;					// Habilitar canal DMA

		// Mandar instruccion de lectura y leectura de datos
		I2C2->CR1 &= !I2C_CR1_ACK;							// Disable ACK
		//I2C2->CR2 |= I2C_CR2_LAST;							// Habilitar NACK, cuando se llena el buffer DMA
		I2C2->CR1 |= I2C_CR1_START;							// Secuencia de inicio
		while(!(I2C2->SR1 & I2C_SR1_SB));					// SB=1?, Esperar a que se complete la secuencia de inicio
		I2C2->DR = 0x53;									// Escribir Direccion de lectura de périferico
		while(!(I2C2->SR1 & I2C_SR1_ADDR));					// ADDR=1?, ACK de direccion de periferico
		I2C_Aux = I2C2->SR2;								// Lectura basura para limpiar ADDR
		while(!(DMA1->ISR & DMA_ISR_TCIF5));				// TCIF5=1?, Esperar a completar la transferencia de datos por DMA
		DMA1->IFCR |= DMA_IFCR_CTCIF5;						// TCIF5=1, Limpiar bandera de transferencia de datos por DMA completada
		//I2C2->CR1 &= !I2C_CR1_ACK;						// Disable ACK
		//I2C2->CR2 &= !I2C_CR2_LAST;						// Deshabilitar NACK, cuando se llena el buffer DMA
		I2C2->CR1 |= I2C_CR1_STOP;							// Secuencia de parada
		DMA1_Channel5->CCR &= !DMA_CCR_EN;					// Deshabilitar canal DMA
		wait_us(4);											// Espera de 4 us
	}else{

		// Mandar instruccion de escritura y direccion de lectura

		//I2C2->CR2 |= I2C_CR2_LAST;						// Habilitar NACK, cuando se llena el buffer DMA
		I2C2->CR1 |= I2C_CR1_START;							// Secuencia de inicio
		while(!(I2C2->SR1 & I2C_SR1_SB));					// SB=1?, Esperar a que se complete la secuencia de inicio
		I2C2->DR = 0x52;									// Escribir Direccion de escritura de périferico
		while(!(I2C2->SR1 & I2C_SR1_ADDR));					// ADDR=1?, ACK de direccion de periferico
		I2C_Aux= I2C2->SR2;									// Lectura basura para limpiar ADDR
		I2C2->DR = address;									// Escribir Direccion de registro de périferico
		while(!(I2C2->SR1 & I2C_SR1_TXE));					// TX=1?, Esperar a que el registro de datos TX este vacio (transmision)
		I2C2->CR1 |= I2C_CR1_STOP;							// Secuencia de parada
		wait_us(4);											// Espera de 4 us

		// Reajuste DMA
		DMA1_Channel5->CMAR = (int)I2C_rxBuffer;			// Direccion de memoria
		DMA1_Channel5->CPAR = (int)&I2C2->DR;				// Direccion de periferico
		DMA1_Channel5->CNDTR = size;						// Numero de Bytes a ser transferidos
		DMA1_Channel5->CCR |= DMA_CCR_TCIE;					// Habilitar interrupcion de transferencia completada
		DMA1_Channel5->CCR |= DMA_CCR_MINC;					// Activar incremento de memoria CMAR, cada vez que se realize una transferencia
		//DMA1_Channel5->CCR |= DMA_CCR_CIRC;				// Activar modo circular
		DMA1_Channel5->CCR |= DMA_CCR_EN;					// Habilitar canal DMA

		// Mandar instruccion de lectura y leectura de datos
		I2C2->CR1 |= I2C_CR1_ACK;							// Enable ACK
		I2C2->CR2 |= I2C_CR2_LAST;							// Habilitar NACK, cuando se llena el buffer DMA
		I2C2->CR1 |= I2C_CR1_START;							// Secuencia de inicio
		while(!(I2C2->SR1 & I2C_SR1_SB));					// SB=1?, Esperar a que se complete la secuencia de inicio
		I2C2->DR = 0x53;									// Escribir Direccion de lectura de périferico
		while(!(I2C2->SR1 & I2C_SR1_ADDR));					// ADDR=1?, ACK de direccion de periferico
		I2C_Aux = I2C2->SR2;								// Lectura basura para limpiar ADDR
		while(!(DMA1->ISR & DMA_ISR_TCIF5));				// TCIF5=1?, Esperar a completar la transferencia de datos por DMA
		DMA1->IFCR |= DMA_IFCR_CTCIF5;						// TCIF5=1, Limpiar bandera de transferencia de datos por DMA completada
		//I2C2->CR1 &= !I2C_CR1_ACK;						// Disable ACK
		//I2C2->CR2 &= !I2C_CR2_LAST;						// Deshabilitar NACK, cuando se llena el buffer DMA
		I2C2->CR1 |= I2C_CR1_STOP;							// Secuencia de parada
		DMA1_Channel5->CCR &= !DMA_CCR_EN;					// Deshabilitar canal DMA
		wait_us(4);											// Espera de 4 us
	}
}


