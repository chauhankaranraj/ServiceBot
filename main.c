//*****************************************************************************
//
// MSP432 main.c template - Empty main
//
//****************************************************************************

/*
 *  TODO:
 *  1. change 55 to 56
 *  2. interrupt-ception??
 *  3. change output port from timer
 */
#include <stdio.h>
#include <ti/devices/msp432p4xx/inc/msp.h>
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

#define LEFT_MOTOR_PORT GPIO_PORT_P4
#define LEFT_MOTOR_PIN GPIO_PIN1
#define RIGHT_MOTOR_PORT GPIO_PORT_P1
#define RIGHT_MOTOR_PIN GPIO_PIN6

#define FALL_SIGNAL_PORT GPIO_PORT_P4
#define FALL_SIGNAL_PIN GPIO_PIN6

#define US_TRIGGER_PORT GPIO_PORT_P2
#define US_TRIGGER_PIN GPIO_PIN7
#define US_ECHO_PORT GPIO_PORT_P5
#define US_ECHO_PIN GPIO_PIN1

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

#define INITIAL_HALF_PERIOD 2500

volatile uint8_t currentState = INIT_STATE;

volatile uint8_t data;
volatile uint8_t rxDataArray[14];
volatile uint8_t index = 0;

volatile uint16_t checksum;
volatile uint16_t signatureNumber;
volatile uint16_t xCenter;
volatile uint16_t yCenter;
volatile uint16_t width;
volatile uint16_t height;
volatile int16_t angle;

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
 *  ULTRASONIC SENSOR TIMER_A PWM
 ********************************************************************/
// Port mapping configuration register
const uint8_t portMapping[] =
{
    //Port P2:
    PM_NONE, PM_NONE, PM_NONE, PM_NONE, PM_NONE, PM_NONE, PM_NONE,
    PM_TA0CCR0A
};

// Timer_A UpMode Configuration Parameter
const Timer_A_UpModeConfig upConfig =
{
        TIMER_A_CLOCKSOURCE_SMCLK,              // SMCLK Clock Source
        TIMER_A_CLOCKSOURCE_DIVIDER_8,          // SMCLK/8 = 1MHz
        INITIAL_HALF_PERIOD,                    // 100 tick period
        TIMER_A_TAIE_INTERRUPT_DISABLE,         // Disable Timer interrupt
        TIMER_A_CCIE_CCR0_INTERRUPT_DISABLE ,   // Disable CCR0 interrupt
        TIMER_A_DO_CLEAR                        // Clear value
};

const Timer_A_CompareModeConfig compareConfig =
{
        TIMER_A_CAPTURECOMPARE_REGISTER_0,
        TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE,
        TIMER_A_OUTPUTMODE_TOGGLE,
        INITIAL_HALF_PERIOD
};

void initTimer(void) // initialization and start of timer
{
    /* Configuring Timer_A1 for Up Mode */
    MAP_Timer_A_configureUpMode(TIMER_A0_BASE, &upConfig);
    MAP_Timer_A_initCompare(TIMER_A0_BASE, &compareConfig);

    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(US_TRIGGER_PORT,
                                                    US_TRIGGER_PIN,
                                                    GPIO_PRIMARY_MODULE_FUNCTION);
    MAP_Timer_A_startCounter(TIMER_A0_BASE, TIMER_A_UP_MODE);   // start TA0 in up mode
}

/********************************************************************
 *  STATE MACHINE
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
 *  CAR MOVEMENT
 ********************************************************************/
void moveCar(void)
{
    // angle
    if (angle>ANGLE_LOW_THRESHOLD && angle<ANGLE_HIGH_THRESHOLD)
    {
        MAP_GPIO_setOutputHighOnPin(FALL_SIGNAL_PORT, FALL_SIGNAL_PIN);
    }
    else
    {
        MAP_GPIO_setOutputLowOnPin(FALL_SIGNAL_PORT, FALL_SIGNAL_PIN);
    }

    // check if it is ok to move forward. If not, move backward
    if (!MAP_GPIO_getInputPinValue(US_ECHO_PORT, US_ECHO_PIN))
    {
        MAP_GPIO_setOutputLowOnPin(RIGHT_MOTOR_PORT, RIGHT_MOTOR_PIN);
        MAP_GPIO_setOutputLowOnPin(LEFT_MOTOR_PORT, LEFT_MOTOR_PIN);
        return;
    }

    // forward backward
    if (width < WIDTH_THRESHOLD)
    {
        MAP_GPIO_setOutputHighOnPin(LEFT_MOTOR_PORT, LEFT_MOTOR_PIN);
        MAP_GPIO_setOutputHighOnPin(RIGHT_MOTOR_PORT, RIGHT_MOTOR_PIN);
    }
    else
    {
        MAP_GPIO_setOutputLowOnPin(LEFT_MOTOR_PORT, LEFT_MOTOR_PIN);
        MAP_GPIO_setOutputLowOnPin(RIGHT_MOTOR_PORT, RIGHT_MOTOR_PIN);
    }

    // right left
    if (xCenter > RIGHT_THRESHOLD)
    {
        MAP_GPIO_setOutputHighOnPin(LEFT_MOTOR_PORT, LEFT_MOTOR_PIN);
        MAP_GPIO_setOutputLowOnPin(RIGHT_MOTOR_PORT, RIGHT_MOTOR_PIN);
    }
    else if (xCenter < LEFT_THRESHOLD)
    {
        MAP_GPIO_setOutputLowOnPin(LEFT_MOTOR_PORT, LEFT_MOTOR_PIN);
        MAP_GPIO_setOutputHighOnPin(RIGHT_MOTOR_PORT, RIGHT_MOTOR_PIN);
    }
    else
    {
        MAP_GPIO_setOutputLowOnPin(RIGHT_MOTOR_PORT, RIGHT_MOTOR_PIN);
        MAP_GPIO_setOutputLowOnPin(LEFT_MOTOR_PORT, LEFT_MOTOR_PIN);
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

        if (didReceiveSyncWords())
        {
            rxDataArray[index++] = data;
            if (index==14)
            {
                checksum = rxDataArray[0] + 256*rxDataArray[1];
                signatureNumber = rxDataArray[2] + 256*rxDataArray[3];
                xCenter = rxDataArray[4] + 256*rxDataArray[5];
                yCenter = rxDataArray[6] + 256*rxDataArray[7];
                width = rxDataArray[8] + 256*rxDataArray[9];
                height = rxDataArray[10] + 256*rxDataArray[11];
                angle = rxDataArray[12] + 256*rxDataArray[13];

                if (signatureNumber==SIGNATURE_123)
                {
                    moveCar();
                }

                currentState = INIT_STATE;
                index = 0;
            }
        }
    }
}


/********************************************************************
 *  MAIN
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

    // motor pins 4.1 (LEFT), 1.6 (RIGHT)
    MAP_GPIO_setAsOutputPin(LEFT_MOTOR_PORT, LEFT_MOTOR_PIN);
    MAP_GPIO_setAsOutputPin(RIGHT_MOTOR_PORT, RIGHT_MOTOR_PIN);
    MAP_GPIO_setOutputLowOnPin(LEFT_MOTOR_PORT, LEFT_MOTOR_PIN);
    MAP_GPIO_setOutputLowOnPin(RIGHT_MOTOR_PORT, RIGHT_MOTOR_PIN);

    // fall pin 4.6
    MAP_GPIO_setAsOutputPin(FALL_SIGNAL_PORT, FALL_SIGNAL_PIN);
    MAP_GPIO_setOutputLowOnPin(FALL_SIGNAL_PORT, FALL_SIGNAL_PIN);

    // ultrasonic sensor echo pin 5.1
    MAP_GPIO_setAsInputPin(US_ECHO_PORT, US_ECHO_PIN);

    // Selecting P2.2 (UCA1RXD) and P2.3 (UCA1TXD) in UART mode
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P2, GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);

    // map ports
    MAP_PMAP_configurePorts(portMapping, PMAP_P2MAP, 1, PMAP_DISABLE_RECONFIGURATION);
    initTimer();

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
