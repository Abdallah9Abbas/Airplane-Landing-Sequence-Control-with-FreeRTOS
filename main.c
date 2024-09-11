#include "TM4C123.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include <stdbool.h>
#include <stdint.h>
#include "task.h"
#include "semphr.h"

// Definitions for PORTF registers
#define GPIO_PORTF_DATA_R       (*((volatile unsigned long *)0x400253FC))
#define GPIO_PORTF_DIR_R        (*((volatile unsigned long *)0x40025400))
#define GPIO_PORTF_PUR_R        (*((volatile unsigned long *)0x40025510))
#define GPIO_PORTF_DEN_R        (*((volatile unsigned long *)0x4002551C))
#define GPIO_PORTF_CR_R         (*((volatile unsigned long *)0x40025524))
#define GPIO_PORTF_AMSEL_R      (*((volatile unsigned long *)0x40025528))
#define GPIO_PORTF_PCTL_R       (*((volatile unsigned long *)0x4002552C))
#define GPIO_PORTF_DR2R_R       (*((volatile unsigned long *)0x40025500))
#define GPIO_PORTF_LOCK_R       (*((volatile unsigned long *)0x40025520))
#define GPIO_PORTF_AFSEL_R      (*((volatile unsigned long *)0x40025420))

// Definitions for PORTB registers
#define GPIO_PORTB_DATA_R (*((volatile unsigned long *)0x400053FC))
#define GPIO_PORTB_DIR_R (*((volatile unsigned long *)0x40005400))
#define GPIO_PORTB_DEN_R (*((volatile unsigned long *)0x4000551C))
#define GPIO_PORTB_Pin0_7 0xFF

// Definitions for system control
#define SYSCTL_RCGC2_R (*((volatile unsigned long *)0x400FE108))
#define SYSCTL_RCGC2_GPIOBF 0x22

#define SYSCTL_RCGCGPIO_R       (*((volatile unsigned long *)0x400FE608))
#define SYSCTL_RCGCTIMER_R      (*((volatile unsigned long *)0x400FE604))
#define NVIC_EN0_R              (*((volatile unsigned long *)0xE000E100))
#define GPIO_LOCK_KEY           0x4C4F434B

// Semaphore and queue handles
SemaphoreHandle_t xSemaphoreSW1 = NULL;
SemaphoreHandle_t xSemaphoreSW2 = NULL;
QueueHandle_t xQueue;

// Variables for switch states
int SW1, SW2;
#define DEBOUNCE_DELAY_MS 50



// Function to initialize PORTB
void PORTB_Init(void) {
    SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOBF;
    GPIO_PORTB_DEN_R |= GPIO_PORTB_Pin0_7;
    GPIO_PORTB_DIR_R |= GPIO_PORTB_Pin0_7;
}

// Function to initialize PORTF
void PortF_Init(void) {
    volatile unsigned long delay;
    SYSCTL_RCGCGPIO_R |= 0x00000020;
    delay = SYSCTL_RCGCGPIO_R;
    GPIO_PORTF_LOCK_R = GPIO_LOCK_KEY;
    GPIO_PORTF_CR_R = 0x1F;
    GPIO_PORTF_AMSEL_R = 0x00;
    GPIO_PORTF_PCTL_R = 0x00000000;
    GPIO_PORTF_DIR_R = 0x0E;
    GPIO_PORTF_AFSEL_R = 0x00;

    GPIO_PORTF_PUR_R = 0x11;
    GPIO_PORTF_DEN_R = 0x1F;

    SYSCTL_RCGCTIMER_R |= (1 << 0);
    TIMER0->CTL &= ~(1 << 0);
    TIMER0->CFG = 0x00000000;
    TIMER0->TAMR |= (0x2 << 0);

    GPIOF->IS &= ~((1 << 0) | (1 << 4));
    GPIOF->IBE &= ~((1 << 0) | (1 << 4));
    GPIOF->IEV &= ~((1 << 0) | (1 << 4));
    GPIOF->IM |= ((1 << 0) | (1 << 4));
    GPIOF->ICR |= ((1 << 0) | (1 << 4));

    NVIC_EN0_R |= (1 << 30);
    NVIC->ISER[0] |= (1 << 30);
    NVIC->IP[30] = 0x01;
}

// Interrupt handler for PORTF
void GPIOF_Handler(void) {
    if (GPIO_PORTF_DATA_R & 0x02) {
        GPIOF->ICR |= (1 << 4) | (1 << 0); //Clear flags
    } else {
        if (GPIOF->RIS & (1 << 4)) {
            GPIO_PORTF_DATA_R &= 0xF1; //Turn off the LED
            GPIO_PORTF_DATA_R |= 0x08; // Green LED
            GPIOB->DATA = 0x5B; //2 on 7 segment 
            GPIOF->ICR |= (1 << 4); //Clear flag
        }

        if (GPIOF->RIS & (1 << 0)) {
            GPIO_PORTF_DATA_R &= 0xF1; //Turn off the LED
            GPIO_PORTF_DATA_R |= 0x02; //Red LED
            GPIOB->DATA = 0x06; // 1 on 7 segment
            GPIOF->ICR |= (1 << 0); //Clear flag
        }
    }
}

// Sender task to send values to the queue
void vSenderTask(void *pvParameters) {
    int32_t ValueToBeSent;
    BaseType_t xStatus;
    ValueToBeSent = (int32_t)pvParameters;
    for (;;) {
        xStatus = xQueueSend(xQueue, &ValueToBeSent, (TickType_t)1000);
        if (xStatus != pdPASS) {
            // Error handling code
        }
        taskYIELD();
    }
}

// Receiver task to receive values from the queue
void vReceiverTask(void *pvParameters) {
    int32_t ReceivedValue;
    BaseType_t xStatus;

    for (;;) {
        xStatus = xQueueReceive(xQueue, &ReceivedValue, 0);
        SW1 = GPIOF->DATA & 0x10;
        SW2 = GPIOF->DATA & 0x01;
        if (!(SW1 || SW2)) {
            // Handle case when neither SW1 nor SW2 is pressed
        } else if ((GPIO_PORTF_DATA_R & 0x02) && SW2 && (!SW1)) {
            GPIO_PORTF_DATA_R &= 0xF1; //Turn off the LED
            GPIO_PORTF_DATA_R |= 0x08; //Green Light
            GPIOB->DATA = 0x5B;  // 2 on 7 segment
            GPIOF->ICR |= (1 << 4);
        } else if (SW1 && SW2) {
            if (xStatus == pdPASS) {
                if (ReceivedValue == 0x100) {
                    GPIO_PORTF_DATA_R &= 0xF1;  //Turn off the LED
                    GPIO_PORTF_DATA_R |= 0x0A; // yellow   RG-    0x0A
                    GPIOB->DATA = 0x4F; //3 on the 7 segments
                    vTaskDelay(1000);
                } else if (ReceivedValue == 0x200) {
                    GPIO_PORTF_DATA_R &= 0xF1; //Turn off the LED
                    GPIO_PORTF_DATA_R |= 0x06; // pink     R-B    0x06
                    GPIOB->DATA = 0x4F; //3 on the 7 segments
                    vTaskDelay(1000);
                } else if (ReceivedValue == 0x300) {
                    GPIO_PORTF_DATA_R &= 0xF1;  //Turn off the LED
                    GPIO_PORTF_DATA_R |= 0x0C; // sky blue -GB    0x0C
                    GPIOB->DATA = 0x4F; //3 on the 7 segments
                    vTaskDelay(1000);
                } else {
                    GPIO_PORTF_DATA_R &= 0xF1;
                    GPIOB->DATA = 0x00;
                }
            }
        }
    }
}

int main(void) {
    PortF_Init();
    PORTB_Init();

    xQueue = xQueueCreate(10, sizeof(int32_t));
    xTaskCreate(vSenderTask, "sender 1", 100, (void *)0x100, 3, NULL); //Yellow
    xTaskCreate(vSenderTask, "sender 2", 100, (void *)0x200, 3, NULL); //Pink
    xTaskCreate(vSenderTask, "sender 3", 100, (void *)0x300, 3, NULL); //Sky blue
    xTaskCreate(vReceiverTask, "receiver", 100, NULL, 2, NULL);

    xSemaphoreSW1 = xSemaphoreCreateBinary();
    xSemaphoreSW2 = xSemaphoreCreateBinary();

    vTaskStartScheduler();

    while (1) {
        // Main loop (should not be reached)
    }

    return 0;
}
// Color    LED(s) PortF
// dark     ---    0
// red      R--    0x02
// blue     --B    0x04
// green    -G-    0x08
// yellow   RG-    0x0A
// sky blue -GB    0x0C
// white    RGB    0x0E
// pink     R-B    0x06
