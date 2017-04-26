//*****************************************************************************
//
// MSP432 main.c template - Empty main
//
//****************************************************************************

#include <ti/devices/msp432p4xx/inc/msp.h>
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

#define INIT_STATE 1    // nothing received so far
#define AA_RECEIVED 2    // one aa received
#define AA55_RECEIVED 3    // aa55 received
#define AA55AA_RECEIVED 4    // aa55aa received
#define AA55AA55_RECEIVED 5    // aa55aa55 received

volatile uint8_t currentState = 0;

//volatile uint8_t txData = 0x01;
volatile uint8_t rxData[12];
volatile uint8_t index = 0;

//volatile uint8_t endWords[4];
//volatile uint8_t wordIndex = 0;

volatile uint16_t xpos;
volatile uint16_t ypos;

const uint16_t RIGHT_THRESHOLD = 213;
const uint16_t LEFT_THRESHOLD = 106;

/* received object format
    0, 1     y              sync: 0xaa55=normal object, 0xaa56=color code object
    2, 3     y              checksum (sum of all 16-bit words 2-6, that is, bytes 4-13)
    4, 5     y              signature number
    6, 7     y              x center of object  // 0-319
    8, 9     y              y center of object  // 0-199
    10, 11   y              width of object     // 1-320
    12, 13   y              height of object    // 1-200
 */

// Configuration for clock source 1MHz and required baud rate = 9600 baud/s
const eUSCI_UART_Config uartConfig =
{
     EUSCI_A_UART_CLOCKSOURCE_SMCLK,                // clockSource
     6,                                             // clockPrescalar
     8,                                             // firstModReg
     17,                                            // secondModReg
     EUSCI_A_UART_NO_PARITY,                        // No parity
     EUSCI_A_UART_LSB_FIRST,                        // LSB first transmission
     EUSCI_A_UART_ONE_STOP_BIT,                     // One stop bit
     EUSCI_A_UART_MODE,                             // UART mode
     EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION  // Over-sampling on
};

void main(void)
{
    // Stop the watch dog timer
    MAP_WDT_A_holdTimer();

    // Set DCO frequency to 1MHz
    MAP_CS_setDCOFrequency(CS_1MHZ);

    // Set DCO as source for MCLK and SMCLK
    MAP_CS_initClockSignal(CS_MCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
    MAP_CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);

    // Selecting P2.2 (UCA1RXD) and P2.3 (UCA1TXD) in UART mode
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P2, GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);

    /* Configuring UART Module */
    MAP_UART_initModule(EUSCI_A1_BASE, &uartConfig);

    /* Enable UART module */
    MAP_UART_enableModule(EUSCI_A1_BASE);

    /* Enabling interrupts */
    MAP_UART_enableInterrupt(EUSCI_A1_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
    MAP_Interrupt_enableInterrupt(INT_EUSCIA1);
    MAP_Interrupt_enableSleepOnIsrExit();
    MAP_Interrupt_enableMaster();

    MAP_PCM_gotoLPM0();
    __no_operation();
}

/* ISR for UART receive */
void EUSCIA1_IRQHandler(void)
{
    // interrupt status
    uint_fast8_t status = MAP_UART_getEnabledInterruptStatus(EUSCI_A1_BASE);

    if (status & EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG)   // check if receive flag is raised
    {
        uint8_t data = MAP_UART_receiveData(EUSCI_A1_BASE);

        if (currentState==INIT_STATE && data==0xaa) // nothing received yet
        {
            currentState = AA_RECEIVED;
        }
        else if (currentState==AA_RECEIVED && data==0xa55)    // 'aa' already received
        {
            currentState = AA55_RECEIVED;
        }
        else if (currentState==AA55_RECEIVED && data==0xaa)    // 'aa55' already received
        {
            currentState = AA55AA_RECEIVED;
        }
        else if (currentState==AA55AA_RECEIVED && data==0x55)
        {
            currentState = AA55AA55_RECEIVED;
        }
        else if (currentState==AA55AA55_RECEIVED)
        {
            rxData[++index] = data;
            if (index==12)
            {
                currentState = INIT_STATE;
            }
        }


////        if (data)
//        {
//            if ((endWords[0]==0xaa && endWords[1]==0x55 && endWords[2]==0xaa && endWords[3]==0x55))
//            {
//                if (index==12)
//                {
////                    wordIndex = 0;
//                    endWords[0] = data;
//                    endWords[1] = 0;
//                    endWords[2] = 0;
//                    endWords[3] = 0;
//                }
//                else
//                {
//                    rxData[index++] = data;
//                }
//            }
//            else
//            {
//                if (wordIndex==4) wordIndex = 0;
//                endWords[wordIndex++] = data;
//            }
//
//        }

    }
}
