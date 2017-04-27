//*****************************************************************************
//
// MSP432 main.c template - Empty main
//
//****************************************************************************

#include <ti/devices/msp432p4xx/inc/msp.h>
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

#define RIGHT_THRESHOLD 213
#define LEFT_THRESHOLD 106

#define INIT_STATE 1
#define STATE_55_RECEIVED 2
#define STATE_55AA_RECEIVED 3
#define STATE_55AA55_RECEIVED 4
#define STATE_55AA55AA_RECEIVED 5

volatile uint8_t currentState = INIT_STATE;

//volatile uint8_t values[100];

//volatile uint8_t txData = 0x01;
volatile uint8_t rxData[12];
volatile uint8_t index = 0;

//volatile uint8_t endWords[4];
//volatile uint8_t wordIndex = 0;

volatile uint16_t checksum;
volatile uint16_t signatureNumber;
volatile uint16_t xCenter;
volatile uint16_t yCenter;
volatile uint16_t width;
volatile uint16_t height;

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
     52,                                            // clockPrescalar
     1,                                             // firstModReg
     73,                                            // secondModReg
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

    // Set DCO frequency to 8MHz
    MAP_CS_setDCOFrequency(CS_8MHZ);

    // Set DCO as source for MCLK and SMCLK
    MAP_CS_initClockSignal(CS_MCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
    MAP_CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);

    // Selecting P2.2 (UCA1RXD) and P2.3 (UCA1TXD) in UART mode
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P2, GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);

    // Configuring UART Module
    MAP_UART_initModule(EUSCI_A1_BASE, &uartConfig);

    // Enable UART module
    MAP_UART_enableModule(EUSCI_A1_BASE);

    // Enabling interrupts
    MAP_UART_enableInterrupt(EUSCI_A1_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
    MAP_Interrupt_enableInterrupt(INT_EUSCIA1);
    MAP_Interrupt_disableSleepOnIsrExit();
    MAP_Interrupt_enableMaster();

    // go to sleep
    while(1)
    {
        MAP_PCM_gotoLPM0();
        __no_operation();
    }

}

/* ISR for UART receive */
void EUSCIA1_IRQHandler(void)
{
    // interrupt status
    uint_fast8_t status = MAP_UART_getEnabledInterruptStatus(EUSCI_A1_BASE);

    if (status & EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG)   // check if receive flag is raised
    {
        uint8_t data = MAP_UART_receiveData(EUSCI_A1_BASE);



//        values[index] = data;
////        index = index+1 == 100 ? 0 : index++;
//        index++;
//        if (index==100)
//        {
//            index = 0;
//        }




        if (currentState==INIT_STATE) // nothing received yet
        {
            if (data==0x55)
                currentState = STATE_55_RECEIVED;
        }
        else if (currentState==STATE_55_RECEIVED)
        {
            if (data==0xaa)
                currentState = STATE_55AA_RECEIVED;
            else
                currentState = INIT_STATE;
        }
        else if (currentState==STATE_55AA_RECEIVED)
        {
            if (data==0x55)
                currentState = STATE_55AA55_RECEIVED;
            else
                currentState = INIT_STATE;
        }
        else if (currentState==STATE_55AA55_RECEIVED)
        {
            if (data==0xaa)
                currentState = STATE_55AA55AA_RECEIVED;
            else
                currentState = INIT_STATE;
        }
        else if (currentState==STATE_55AA55AA_RECEIVED)
        {
            rxData[index++] = data;
            if (index==12)
            {
                checksum = rxData[0] + 256*rxData[1];
                signatureNumber = rxData[2] + 256*rxData[3];
                xCenter = rxData[4] + 256*rxData[5];
                yCenter = rxData[6] + 256*rxData[7];
                width = rxData[8] + 256*rxData[9];
                height = rxData[10] + 256*rxData[11];

                currentState = INIT_STATE;
                index = 0;
            }
        }
    }
}
