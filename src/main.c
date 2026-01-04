#include "stm32f4xx.h"
#include <stdio.h> 

#define PCF8574_ADDRESS (0x27 << 1)

volatile uint8_t game_locked = 0;

void System_Init(void);
void RCC_Init(void);
void GPIO_Init(void);
void TIM2_PWM_Init(void);
void I2C1_Init(void);
void EXTI_Init(void);
void UART1_Init(void); 

void Delay_ms(uint32_t ms);
void Simple_Delay(int count);

void UART1_Write(char ch);
void UART1_Print(char *str); 

void I2C_Write(uint8_t data);
void LCD_Send_I2C(uint8_t data, uint8_t flags);
void LCD_SendCommand(uint8_t cmd);
void LCD_SendData(uint8_t data);
void LCD_Init(void);
void LCD_String(char *str);
void LCD_Clear(void);
void LCD_SetCursor(uint8_t row, uint8_t col);

void Buzzer_Tone(uint16_t period);
void Buzzer_Stop(void);

int main(void) {
    System_Init(); 

    LCD_Clear();
    LCD_SetCursor(0, 0); LCD_String(" Menunggu Input ");
    LCD_SetCursor(1, 0); LCD_String(" Pencet Tombol! ");

    UART1_Print("\r\n--------------------\r\n");
    UART1_Print(" Menunggu Input \r\n");
    UART1_Print(" Pencet Tombol! \r\n");
    UART1_Print("--------------------\r\n");

    while (1) {
        __WFI(); 
    }
}

void System_Init(void) {
    RCC_Init();
    GPIO_Init();
    TIM2_PWM_Init();
    I2C1_Init();
    UART1_Init(); 
    EXTI_Init();
    
    Delay_ms(500); 
    LCD_Init(); 
}

void RCC_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN | RCC_APB1ENR_I2C1EN;
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN | RCC_APB2ENR_SYSCFGEN;
}

void GPIO_Init(void) {
    GPIOA->MODER |= (1U << (1 * 2)) | (1U << (2 * 2));
    
    GPIOA->MODER &= ~(3U << (0 * 2)); 
    GPIOA->PUPDR |=  (1U << (0 * 2)); 
    GPIOB->MODER &= ~((3U << (3 * 2)) | (3U << (4 * 2))); 
    GPIOB->PUPDR |=  ((1U << (3 * 2)) | (1U << (4 * 2))); 
}

void UART1_Init(void) {
    GPIOA->MODER &= ~((3U << (9 * 2)) | (3U << (10 * 2)));
    GPIOA->MODER |=  ((2U << (9 * 2)) | (2U << (10 * 2)));
    GPIOA->AFR[1] |= (7U << ((9 - 8) * 4)) | (7U << ((10 - 8) * 4));

    USART1->BRR = 0x683;
    USART1->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}

void TIM2_PWM_Init(void) {
    GPIOA->MODER &= ~(3U << (3 * 2));
    GPIOA->MODER |=  (2U << (3 * 2)); 
    GPIOA->AFR[0] |= (1U << 12); 
    TIM2->PSC = 16 - 1;   
    TIM2->ARR = 1000 - 1; 
    TIM2->CCMR2 |= (6U << 12);    
    TIM2->CCER  |= TIM_CCER_CC4E; 
    TIM2->CR1   |= TIM_CR1_CEN;
}

void I2C1_Init(void) {
    GPIOB->MODER &= ~((3U << (6 * 2)) | (3U << (7 * 2)));
    GPIOB->MODER |=  ((2U << (6 * 2)) | (2U << (7 * 2)));
    GPIOB->OTYPER |= (1U << 6) | (1U << 7); //open drain
    GPIOB->AFR[0] |= (4U << 24) | (4U << 28); 
    
    I2C1->CR1 |= I2C_CR1_SWRST;
    I2C1->CR1 &= ~I2C_CR1_SWRST;
    I2C1->CR2 = 16;  
    I2C1->CCR = 80;  
    I2C1->TRISE = 17;
    I2C1->CR1 |= I2C_CR1_PE;
}

void EXTI_Init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    SYSCFG->EXTICR[0] &= ~(0xF << 0);
    SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI3_PB;
    SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI4_PB;
    EXTI->IMR |= (1 << 0) | (1 << 3) | (1 << 4);
    EXTI->FTSR |= (1 << 0) | (1 << 3) | (1 << 4);
    NVIC_EnableIRQ(EXTI0_IRQn);
    NVIC_EnableIRQ(EXTI3_IRQn);
    NVIC_EnableIRQ(EXTI4_IRQn);
}

void EXTI0_IRQHandler(void) {
    if (EXTI->PR & (1 << 0)) {
        EXTI->PR = (1 << 0); 
        game_locked = 0;
        GPIOA->ODR &= ~((1 << 1) | (1 << 2)); 
        Buzzer_Stop();
        LCD_Clear();
        LCD_SetCursor(0, 0); LCD_String(" Menunggu Input ");
        LCD_SetCursor(1, 0); LCD_String(" Pencet Tombol! ");
        UART1_Print("\r\n--------------------\r\n");
        UART1_Print(" Menunggu Input \r\n");
        UART1_Print(" Pencet Tombol! \r\n");
        UART1_Print("--------------------\r\n");
    }
}

void EXTI3_IRQHandler(void) {
    if (EXTI->PR & (1 << 3)) {
        EXTI->PR = (1 << 3); 
        if (!game_locked) {
            game_locked = 1;
            GPIOA->ODR |= (1 << 1); 
            LCD_Clear();
            LCD_SetCursor(0, 0); LCD_String("   PLAYER A:    ");
            LCD_SetCursor(1, 0); LCD_String(" SILAKAN JAWAB! ");
            UART1_Print("\r\n====================\r\n");
            UART1_Print("   PLAYER A:    \r\n");
            UART1_Print(" SILAKAN JAWAB! \r\n");
            UART1_Print("====================\r\n");
            Buzzer_Tone(1000); 
            Delay_ms(1000);    
            Buzzer_Stop();
        } 
    }
}

void EXTI4_IRQHandler(void) {
    if (EXTI->PR & (1 << 4)) {
        EXTI->PR = (1 << 4); 
        if (!game_locked) {
            game_locked = 1;
            GPIOA->ODR |= (1 << 2); 
            LCD_Clear();
            LCD_SetCursor(0, 0); LCD_String("   PLAYER B:    ");
            LCD_SetCursor(1, 0); LCD_String(" SILAKAN JAWAB! ");
            UART1_Print("\r\n====================\r\n");
            UART1_Print("   PLAYER B:    \r\n");
            UART1_Print(" SILAKAN JAWAB! \r\n");
            UART1_Print("====================\r\n");
            Buzzer_Tone(1500); 
            Delay_ms(1000);    
            Buzzer_Stop();
        } 
    }
}

void Buzzer_Tone(uint16_t period) {
    TIM2->CNT = 0;
    TIM2->ARR = period - 1;
    TIM2->CCR4 = period / 2; 
}

void Buzzer_Stop(void) {
    TIM2->CCR4 = 0;
}

void I2C_Write(uint8_t data) {
    I2C1->CR1 |= I2C_CR1_START;
    while (!(I2C1->SR1 & I2C_SR1_SB));
    I2C1->DR = PCF8574_ADDRESS;
    while (!(I2C1->SR1 & I2C_SR1_ADDR));
    (void)I2C1->SR2;
    I2C1->DR = data;
    while (!(I2C1->SR1 & I2C_SR1_BTF));
    I2C1->CR1 |= I2C_CR1_STOP;
}

void LCD_Send_I2C(uint8_t data, uint8_t flags) {
    uint8_t up = (data & 0xF0) | flags | 0x08; 
    uint8_t lo = ((data << 4) & 0xF0) | flags | 0x08;
    I2C_Write(up | 0x04); Simple_Delay(1000); I2C_Write(up & ~0x04);
    I2C_Write(lo | 0x04); Simple_Delay(1000); I2C_Write(lo & ~0x04);
}

void LCD_Init(void) {
    Simple_Delay(1000000);
    LCD_SendCommand(0x33); Simple_Delay(50000);
    LCD_SendCommand(0x32); Simple_Delay(50000);
    LCD_SendCommand(0x28); LCD_SendCommand(0x0C);
    LCD_SendCommand(0x06); LCD_Clear();
}

void LCD_SendCommand(uint8_t cmd) { LCD_Send_I2C(cmd, 0); }
void LCD_SendData(uint8_t data)  { LCD_Send_I2C(data, 1); }
void LCD_String(char *str)       { while (*str) LCD_SendData(*str++); }
void LCD_Clear(void)             { LCD_SendCommand(0x01); Delay_ms(2); }
void LCD_SetCursor(uint8_t row, uint8_t col) {
    LCD_SendCommand((row == 0 ? 0x80 : 0xC0) + col);
}

void Delay_ms(uint32_t ms) {
    SysTick->LOAD = (16000000 / 1000) - 1; 
    SysTick->VAL = 0;
    SysTick->CTRL = 5;
    for (uint32_t i = 0; i < ms; i++) {
        while (!(SysTick->CTRL & 0x10000));
    }
    SysTick->CTRL = 0;
}

void Simple_Delay(int count) {
    for (volatile int i = 0; i < count; i++);
}

void UART1_Write(char ch) {
    while (!(USART1->SR & USART_SR_TXE));
    USART1->DR = (ch & 0xFF);
}

void UART1_Print(char *str) {
    while (*str) {
        UART1_Write(*str++);
    }
}   