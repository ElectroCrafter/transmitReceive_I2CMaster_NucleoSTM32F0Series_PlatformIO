#include "stm32f0xx.h"
#include <stdlib.h>
#define _OPEN_SYS_ITOA_EXT
#define ledPin GPIO_PIN_5
#define SLAVE_ADDRESS 0x08  // I2C address of the Arduino
#define I2C_TIMING 0x00303D5B  // Timing configuration for 100 kHz I2C speed (example)

volatile uint8_t bufferReady = 0;
const int idx = 2;
volatile uint32_t millis_count = 0;
volatile uint32_t captureValue = 0,cnt0=0,cnt1=0;
volatile uint8_t captureFlag = 0;
volatile uint8_t gap = 0;   //the laps computing on timer 3 Ch1 interrupt

/*
Objective   : 
Button Press Detection
    1. LCD 16x2 I2C                                             (on hold)
    2. Receives data from slave to master                       (passed)
    3. Transmits data from master to slave                      (passed)
    4. Receive data from slave and print it to LCD 16x2 I2C     (on hold)
    5. Transmit and receive I2C using two MCU STM32 and arudino (on progress)
 
Bugs        : 
    1. On the LCD, the data cannot pass properly after the system is running few minutes.
    Solutions
    a. skip first on the LCD because it is the low priority

    2. On the transmit and receive data on I2C, arduino receive the data as well as ussual. However,
    the stm32 has few bugs on the data reading from arduino.
    Solutions
    a. deep dive more into i2c features on the stm32, find the method similar with serial 
    available() or limitation of the I2C buffer (it doesn't solve)
    b. the data conflicted on the receiving and transmiting data. We need to find method
    to solve that because if we integrate i2c with another  peripheral, it will will complicate
    situation.
    c.If we've apllied i2c only reading mode, it has worked well.
    d.Use ring buffer or circular buffer to solve this problem.
    e.Simulate the bms communication:
        -only few events request a data in critical situation
        -parameter data of voltage, temperature, and current are transfered continuously

    

Notes       :
    - this section need several external cirucuits, please check on notion
*/

typedef enum {
    INT,
    FLOAT,
    UINT32,
    UINT16,
    UINT8,
    CHR,
    STRING
} DataType;
void GPIOinit(){
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;    // enable GPIOA clock 
    GPIOA->MODER |= GPIO_MODER_MODER5_0 ; //general purpose output mode on pin A5

}
void USART2_Init(void) {
    // Enable USART2 clock
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    // Configure GPIOA pins for USART2
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN; // Enable GPIOA clock
    GPIOA->MODER |= GPIO_MODER_MODER2_1 | GPIO_MODER_MODER3_1; // Alternate function mode for PA2 and PA3
    GPIOA->AFR[0] |= (1 << (2 * 4)) | (1 << (3 * 4)); // AF1 for PA2 and PA3

    // Configure USART2
    USART2->CR1 &= ~USART_CR1_UE; // Disable USART2
    USART2->BRR = SystemCoreClock / 9600; // Set baud rate to 38400
    USART2->CR1 |= USART_CR1_TE | USART_CR1_RE; // Enable transmitter and receiver
    USART2->CR1 |= USART_CR1_UE; // Enable USART2
}

void USART2_SendChar(char ch) {
    // Wait until transmit data register is empty
    while (!(USART2->ISR & USART_ISR_TXE))
        ;
    USART2->TDR = (ch & 0xFF);
}

void USART2_SendString(const char *str) {
    while (*str) {
        USART2_SendChar(*str++);
    }
}
void serialPrint(void* data, DataType type) {
    char buffer[100];  // Adjust size as needed

    switch (type) {
        /*integer converts syntax*/
        /*
        IntToChar(*(int*)data, buffer,10);

        1. data: A pointer to the data passed to the SerialPrint function.
        2. (int*)data: Casts the void* pointer data to an int* (pointer to an integer).
        3. *(int*)data: Dereferences the int* pointer, accessing the integer value stored at that memory location.
        4. IntToChar(): A function that converts an integer to a string and stores the result in buffer.
        5. buffer: A character array where the converted string is stored.
        6. 10 -> decimal ; 2 -> binary ; 16 -> hexa
        */
        case INT:
            itoa(*(int*)data,buffer,10); 
            break;
        case UINT32:
            itoa(*(uint32_t*)data,buffer,10); 
            break;
        case UINT16:
            itoa(*(uint16_t*)data,buffer,10); 
            break;
        case UINT8:
            itoa(*(uint8_t*)data,buffer,10); 
            break;
        case CHR:
            // Cast to char* and assign the character
            buffer[0] = *(char*)data;

            // Null-terminate the string
            buffer[1] = '\0';
            break;
        // case FLOAT:
        //     gcvt((float*)data,4,buffer);  // error find the true casting to collect val in the buffer
        //     break;
        case STRING:
            strcpy(buffer, (char*)data);  // Copy string
            break;
        default:
            return;  // Unsupported data type
    }
    for (int i = 0; buffer[i] != '\0'; i++) {
        USART2->TDR = buffer[i];  // Load character into UART data register
        while (!(USART2->ISR & USART_ISR_TXE));  // Wait until transmission is complete
    }
}

void confTimPol(uint16_t tDelay){
/*
    Timer without interrupt using an internal clock 8 Mhz
  1. Enable timer 2
  2. enable GPIOA at GPIOinit function
  3. Select PA1 as an alternate pin TIM2_CH2 || Bit Masking Technique: Bit Clearing and Setting
  4. Set the PA1 as an alternate GPIO TIM2_CH2 on the low register AFSEL1 (AFRL) using AF2
  5. TIM prescaler register Address offset: 0x28
  6. Setting auto-reload timer2 on the 1000mS (limit count up the counter)
  7. Send an update event to reset the timer and apply settings.
  8. Enable the hardware interrupt.
  9. Start the timer
  */
    RCC ->APB1ENR |= RCC_APB1ENR_TIM2EN;   //(1)
    GPIOA ->MODER = (GPIOA->MODER &~(GPIO_MODER_MODER1))
    |(GPIO_MODER_MODER1_1);               //(3)
    GPIOA ->AFR[0] |= (0b10 <<4);         //(4)
    //time-base unit
    TIM2 ->PSC = 8000-1;            //(5)
    TIM2 ->ARR = 4000-1;            //(6)

    /*Detail Calculation*/
    /*
    1. Clock Configuration
    Assume the timer clock (CK_INT) is running at 8 MHz.
    
    2.Set the prescaler value to 7999. This will divide the timer clock by 8000.
    TIM2->PSC = 7999; // Divides the timer clock by 8000
    - CK_PSC = CK_INT / (PSC + 1)
    - CK_PSC = 8 MHz / (7999 + 1) = 1 kHz -> 1000 pulse per detik || 1 pulse per mS
    if u want create uS
    8Mhz/8 = 1Mhz -> 8*10^6 pulse per detik || 1 pulse per uS

    3.Counter Register (CNT)
    Configure the counter to count up to a value (e.g., 1000) before generating an update event.
    TIM2->ARR = 1000 - 1; // Auto-reload value
    The timer will count from 0 to 999, generating an update event at 1 kHz / 1000 = 1 Hz (once every second).
    */
    
    TIM2 ->EGR  |= TIM_EGR_UG;      //(7)
    TIM2 ->DIER |= TIM_DIER_UIE;    //(8)
    TIM2 -> CR1 |= TIM_CR1_CEN;     //(9)
}
void delay(uint32_t milliseconds) {
    uint32_t i;
    for (i = 0; i < (milliseconds * 800); i++) {
        // Adjust the delay loop according to your system clock frequency
        __NOP();
    }
}

void I2C1_Init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;  // Enable I2C1 clock
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;   // Enable GPIOB clock

    // Configure PB6 and PB7 for I2C1 (SCL and SDA)
    GPIOB->MODER &= ~(GPIO_MODER_MODER6 | GPIO_MODER_MODER7);
    GPIOB->MODER |= (GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1);  // Alternate function
    GPIOB->PUPDR |= (GPIO_PUPDR_PUPDR6_0 | GPIO_PUPDR_PUPDR7_0);  // Pull-up
    GPIOB->AFR[0] |= (1 << GPIO_AFRL_AFRL6_Pos) | (1 << GPIO_AFRL_AFRL7_Pos);  // AF1 for I2C1

    // I2C1->TIMINGR = 0x2000090E;  // Set I2C timing
    I2C1->TIMINGR = (0x1 << I2C_TIMINGR_PRESC_Pos)|(0x4<<I2C_TIMINGR_SCLDEL_Pos)||(0x2 << I2C_TIMINGR_SDADEL_Pos)
    |(0xF << I2C_TIMINGR_SCLH_Pos)|0x13;
    I2C1->CR1 |= I2C_CR1_PE;  // Enable I2C1x
    // I2C_CR1_SWRST
}

void I2C1_Write(uint8_t slaveAddr, uint8_t* data, uint8_t len) {
    while (I2C1->ISR & I2C_ISR_BUSY);
    I2C1->CR2 = (slaveAddr << 1);  // Set slave address (left shift by 1 for write operation)
    I2C1->CR2 |= (len << I2C_CR2_NBYTES_Pos) | I2C_CR2_START | I2C_CR2_AUTOEND;  // Start transmission

    for (int i = 0; i < len; i++) {
        while (!(I2C1->ISR & I2C_ISR_TXIS));  // Wait until TXIS is set
        I2C1->TXDR = data[i];  // Send data byte
    }

    while (!(I2C1->ISR & I2C_ISR_STOPF));  // Wait until STOPF is set
    I2C1->ICR = I2C_ICR_STOPCF;  // Clear STOPF flag
}

void I2C1_Read(uint8_t slaveAddr, char* buffer, uint8_t len) {
    while (I2C1->ISR & I2C_ISR_BUSY);
    // I2C1->CR2 = (slaveAddr << 1) | I2C_CR2_RD_WRN;  // Set slave address for read operation
    // I2C1->CR2 |= (len << I2C_CR2_NBYTES_Pos) | I2C_CR2_START | I2C_CR2_AUTOEND;  // Start transmission
    I2C1->CR2 = (slaveAddr << 1) | I2C_CR2_RD_WRN | (len << I2C_CR2_NBYTES_Pos) | I2C_CR2_AUTOEND | I2C_CR2_START;
    int i=0;
    for (i = 0; i < len; i++) {
        while (!(I2C1->ISR & I2C_ISR_RXNE));    // Wait until RXNE is set (Receive Buffer Not Empty)
        buffer[i] = I2C1->RXDR;             // Read data from the RXDR register 
        // if (I2C1->RXDR>=21 && I2C1->RXDR<=126){ // the only data 21-126 dec Ascii is passed
        //     buffer[i] = I2C1->RXDR;             // Read data from the RXDR register  
        // }
        
        // serialPrint(&buffer[i],CHR);
    }
    // bufferReady=i;
    // if (i==len-1){
    //     bufferReady = 1;
    // } 


    while (!(I2C1->ISR & I2C_ISR_STOPF));  // Wait until STOPF is set (Stop detection)
    I2C1->ICR = I2C_ICR_STOPCF;  // Clear STOPF flag
}

void I2CreadChr(uint8_t slaveAddr, void* buffer){
    I2C1->CR2 = (slaveAddr << 1) | I2C_CR2_RD_WRN;  // Set slave address for read operation
    I2C1->CR2 = (uint32_t)(1<<16) | I2C_CR2_AUTOEND|I2C_CR2_START;  // Start transmission
 
   if ((I2C2->ISR & I2C_ISR_TXE) == I2C_ISR_TXE)  // Wait until RXNE is set (Receive Buffer Not Empty)
    {
        *(char*) buffer = I2C1->RXDR;  // Read data from the RXDR register 
    }
     

    // while (!(I2C1->ISR & I2C_ISR_STOPF));  // Wait until STOPF is set (Stop detection)
    // I2C1->ICR = I2C_ICR_STOPCF;  // Clear STOPF flag
}

int main(void) {
    // Initialize system clock
    SystemInit();
    // Initialize GPIO led A5
    GPIOinit();
    //Initialize UART communication
    USART2_Init();
    //initialze timer 2
    confTimPol(0);
    //initialze I2C
    I2C1_Init();
    //millis variables
    uint32_t prevMillis = 0;            // Read current counter value;
    uint32_t interval =700;
    volatile uint32_t currentMillis = 0;
    uint32_t curr_counter  = 0;
    uint32_t prev_counter = 0;
    uint8_t delta;
    // Buffer to store received data

    char receivedData[idx];  
    //send data to arduino
    uint8_t message[] = "Hi Arduino!";
    char symblChr = '\t';

    while (1) {
        curr_counter = TIM2->CNT;
        // Timer using interrupt or blinking without delay or delay generation

        //clear the update flag and reset counter value
        if (TIM2->SR & TIM_SR_UIF) {    // is not zero
        TIM2->SR &= ~(TIM_SR_UIF);      // reset update interrupt flag
        }

        // Calculate elapsed time in milliseconds
        if (curr_counter  >= prev_counter) {
            delta = curr_counter - prev_counter;
        } else {
            // Handle counter overflow
            delta = (4000 - prev_counter) + curr_counter + 1;
            // serialPrint(&delta,UINT8);
            // USART2_SendChar('\t');
            // serialPrint(&prev_counter,UINT32);
            // // USART2_SendChar('\t');
            // serialPrint(&curr_counter,UINT32);
            // USART2_SendChar('\n');
        }

        currentMillis += delta; // Update global millis count
        prev_counter = curr_counter; // Update previous counter value

        //millis
        if (currentMillis-prevMillis>=interval){
            prevMillis = currentMillis;
            //send data to arduino using I2C
            // I2C1_Write(SLAVE_ADDRESS, message, sizeof(message) - 1);  // Send message to Arduino
            
            //read data 
            I2C1_Read(SLAVE_ADDRESS, receivedData, sizeof(receivedData) - 1);         
            if (!(I2C1->ISR & I2C_ISR_RXNE)){
                // for(uint8_t i=0;i<idx;i++){
                //     if (receivedData[i]=='\0'){
                //         break;
                //     }
                //     serialPrint(&receivedData[i],CHR);
                // }
                serialPrint(&receivedData[0],CHR);
                // serialPrint(&symblChr,CHR);
                // serialPrint(&bufferReady,UINT8);
                // serialPrint(&symblChr,CHR);
                char enter = '\n';
                serialPrint(&enter,CHR);
                bufferReady=0; //reset buffer
            }
            GPIOA -> ODR ^= ledPin; 
        }
    }
}
