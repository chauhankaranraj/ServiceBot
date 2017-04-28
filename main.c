//*****************************************************************************
//
// MSP432 main.c template - Empty main
//
//****************************************************************************

/*
 *  TODO:
 *  1. change 55 to 56
 *  2. interrupt-ception??
 */
#include <stdio.h>
#include <ti/devices/msp432p4xx/inc/msp.h>
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

#define RIGHT_THRESHOLD 200
#define LEFT_THRESHOLD 100
#define WIDTH_THRESHOLD 170
#define HEIGHT_THRESHOLD 70
#define ANGLE_HIGH_THRESHOLD 110
#define ANGLE_LOW_THRESHOLD 70

#define SIGNATURE_123 5201

#define INIT_STATE 1
#define STATE_55_RECEIVED 2
#define STATE_55AA_RECEIVED 3
#define STATE_55AA56_RECEIVED 4
#define STATE_55AA56AA_RECEIVED 5

volatile uint8_t currentState = INIT_STATE;

volatile uint8_t data;
volatile int rxDataArray[12];
volatile uint8_t index = 0;

volatile uint16_t checksum;
volatile uint16_t signatureNumber;
volatile uint16_t xCenter;
volatile uint16_t yCenter;
volatile uint16_t width;
volatile uint16_t height;
volatile uint16_t angle;

//volatile int xInt, yInt, zInt;

volatile float x = 0,y = 0,z = 0;
// X -- 4 3 2 1
// Y -- 8 7 6 5
// Z -- 12 11 10 9

typedef union {
  int intForm;
  float floatForm;
} acc;

acc xAcc, yAcc, zAcc;



/* received object format
    0, 1     y              sync: 0xaa55=normal object, 0xaa56=color code object
    2, 3     y              checksum (sum of all 16-bit words 2-6, that is, bytes 4-13)
    4, 5     y              signature number
    6, 7     y              x center of object  // 0-319
    8, 9     y              y center of object  // 0-199
    10, 11   y              width of object     // 1-320
    12, 13   y              height of object    // 1-200
 */

/********************************************************************
 *  STATE MACHINE FUNCTION
 ********************************************************************/
uint8_t didReceiveSyncWords(void)
{
    if (currentState==INIT_STATE)
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
       if (data==0x56)
           currentState = STATE_55AA56_RECEIVED;
       else
           currentState = INIT_STATE;
    }
    else if (currentState==STATE_55AA56_RECEIVED)
    {
       if (data==0xaa)
           currentState = STATE_55AA56AA_RECEIVED;
       else
           currentState = INIT_STATE;
    }
    else if (currentState==STATE_55AA56AA_RECEIVED)
    {
        return 1;
    }

    return 0;
}

/********************************************************************
 *  CAR MOVEMENT FUNCTION
 ********************************************************************/

void moveCar(void)
{
    // angle
    if (angle>ANGLE_LOW_THRESHOLD && angle<ANGLE_HIGH_THRESHOLD)
    {
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN6);
    }
    else
    {
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN6);
    }

    // forward backward
    if (width < WIDTH_THRESHOLD)
    {
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN1);
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN6);
    }
    else
    {
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN1);
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN6);
    }

    // right left
    if (xCenter > RIGHT_THRESHOLD)
    {
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN1);
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN6);
    }
    else if (xCenter < LEFT_THRESHOLD)
    {
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN1);
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN6);
    }
    else
    {
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN6);
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN1);
    }


}


/********************************************************************
 *  UART CONFIGURATION AND INTERRUPT HANDLER
 ********************************************************************/

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

// ISR for UART receive
void EUSCIA1_IRQHandler(void)
{
    // interrupt status
    uint_fast8_t status = MAP_UART_getEnabledInterruptStatus(EUSCI_A1_BASE);

    if (status & EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG)   // check if receive flag is raised
    {
        data = MAP_UART_receiveData(EUSCI_A1_BASE);

        if (currentState==1)
        {
            if (data==33)
                currentState = 2;
            else
                currentState = 1;
        }
        else if (currentState==2)
        {
            if (data==65)
                currentState = 3;
            else
                currentState = 1;
        }
        else if (currentState==3)
        {
            rxDataArray[index++] = data;
            if (index==12)
            {
                index = 0;

//                test2 = (rxDataArray[4]) | (rxDataArray[3]<<8) | (rxDataArray[2]<<16) | (rxDataArray[1]<<24);
//                xLower = (((int)rxDataArray[3])<<8);

//                x = (int)rxDataArray[1] + (int)rxDataArray[2]*256 + rxDataArray[3]*65536 + rxDataArray[4]*16777216);

                // PERFECTLY WORKING
//                xLower = (((int)rxDataArray[4]) << 24)
//                  | (((int)rxDataArray[3]) << 16)
//                  | (((int)rxDataArray[2]) << 8)
//                  | (((int)rxDataArray[1]) << 0);

                xAcc.intForm = ((((int)rxDataArray[3]) << 24)
                  | (((int)rxDataArray[2]) << 16)
                  | (((int)rxDataArray[1]) << 8)
                  | (((int)rxDataArray[0]) << 0));

                x = xAcc.floatForm;

                yAcc.intForm = ((((int)rxDataArray[7]) << 24)
                  | (((int)rxDataArray[6]) << 16)
                  | (((int)rxDataArray[5]) << 8)
                  | (((int)rxDataArray[4]) << 0));

                y = yAcc.floatForm;

                zAcc.intForm = ((((int)rxDataArray[11]) << 24)
                  | (((int)rxDataArray[10]) << 16)
                  | (((int)rxDataArray[9]) << 8)
                  | (((int)rxDataArray[8]) << 0));

                z = zAcc.floatForm;

//                x = (float)((((int)rxDataArray[3]) << 24)
//                        | (((int)rxDataArray[2]) << 16)
//                        | (((int)rxDataArray[1]) << 8)
//                        | (((int)rxDataArray[0]) << 0));
//
//                y = (float)((((int)rxDataArray[7]) << 24)
//                        | (((int)rxDataArray[6]) << 16)
//                        | (((int)rxDataArray[5]) << 8)
//                        | (((int)rxDataArray[4]) << 0));
//
//                z = (float)((((int)rxDataArray[11]) << 24)
//                        | (((int)rxDataArray[10]) << 16)
//                        | (((int)rxDataArray[9]) << 8)
//                        | (((int)rxDataArray[8]) << 0));

                if (y<-4)   // turn left
                {
                    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN6);
                    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN1);
                }
                else if (y>4)   // turn right
                {
                    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN6);
                    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN1);
                }
                else if (z<-2) // turn on both motors in reverse
                {
                    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN6);
                    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN1);
                }
                else if (z>6)   // turn on both motors in forward
                {
                    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN6);
                    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN1);
                }
                else
                {
                    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN6);
                    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN1);
                }

                currentState = 1;
            }
        }






//        if (didReceiveSyncWords())
//        {
//            rxDataArray[index++] = data;
//            if (index==14)
//            {
//                checksum = rxDataArray[0] + 256*rxDataArray[1];
//                signatureNumber = rxDataArray[2] + 256*rxDataArray[3];
//                xCenter = rxDataArray[4] + 256*rxDataArray[5];
//                yCenter = rxDataArray[6] + 256*rxDataArray[7];
//                width = rxDataArray[8] + 256*rxDataArray[9];
//                height = rxDataArray[10] + 256*rxDataArray[11];
//                angle = rxDataArray[12] + 256*rxDataArray[13];
//
//                if (signatureNumber==SIGNATURE_123)
//                {
//                    moveCar();
//                }
//
//                currentState = INIT_STATE;
//                index = 0;
//            }
//        }
    }
}

/********************************************************************
 *  MAIN FUNCTION
 ********************************************************************/

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

    // motor pins 4.1, 1.6, fall 4.6
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN1 | GPIO_PIN6);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN6);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN1 | GPIO_PIN6);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN6);

    // Configuring UART Module
    MAP_UART_initModule(EUSCI_A1_BASE, &uartConfig);

    // Enable UART module
    MAP_UART_enableModule(EUSCI_A1_BASE);

    // Enabling interrupts
    MAP_UART_enableInterrupt(EUSCI_A1_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
    MAP_Interrupt_enableInterrupt(INT_EUSCIA1);
    MAP_Interrupt_disableSleepOnIsrExit();
    MAP_Interrupt_enableMaster();

    while(1)
    {
        // go to sleep
        MAP_PCM_gotoLPM0();
        __no_operation();
    }

}
