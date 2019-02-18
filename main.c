/*
 * Lab 7 Assignment: Implements forward/back style rover controls with both tracks reversible at arbitrary PWM settings but controlled together.
 * Can be either run indefinitely at a specific PWM, or given a setpoint distance which it approaches via PID. 1 Quad encoder (both position and speed), 1 Ultrasound distance sensor (20Hz sensing cycle),
 *  and 1 ADC input current sensor. Data Gathering Toggleable via 'g'. Includes Live PID Tuning mode which alternates between setpoints on a regular basis while
 *  allowing modification of Kp, Kd, and Ki.
 *
 *  All variables are necessary for a functional implementation are present already;
 *  search for LOOK_HERE to find relevant variables and TODO to find remaining tasks for the Lab.
 *  Compiles with 7 anticipated, but irrelevant warnings D:
 *
 * PID COMMANDS:
 * 's#!' sets a new setpoint distance to maintain; # is potentially-multidigit and measured in mm
 * 'L' enters Live Tuning mode (enables changing Kp/Kd/Ki via live tuning commands, and alternates the setpoint between two setpoints using Timer_A1)
 * 'u' cycles which K is being tuned between Kp (Red), Kd (green), and Ki (blue)
 * 'r' current K -= 100 (only while Live Tuning)
 * 't' current K -= 10  (only while Live Tuning)
 * 'y' current K -= 1   (only while Live Tuning)
 * 'i' current K += 1   (only while Live Tuning)
 * 'o' current K += 10  (only while Live Tuning)
 * 'p' current K += 100 (only while Live Tuning)
 *
 * GENERAL COMMANDS:
 * 'g' toggles data gathering/printing: See packageDataOntoBuffer for format
 * 'z' zeroes out the quad encoders
 *
 * DIRECT DRIVE COMMANDS:
 * 'F' sets all 4 motor control signals to full PWM, forward, indefinitely (100% duty cycle PWM)
 * 'x' stops all 4 motor control signals, indefinitely (sets them to 0% duty cycle PWM)
 * 'D+XX!' sets left and right motor control signals to XX% duty cycle PWM, indefinitely.
 *             '+'s can be '-' to indicate reverse direction as well. XX is 2 digit value, always.
 *
 *
 * All characters are echoed, unrecognized characters aside from /n or /r are replied to with ???
 *
 * Uses all available timers on an MSP432 except for Timer32_0
 *
 * P5.5 = A0 analog input for Right Front Motor Current
 * P2.4 to 2.7 = Motor Control PWM signal outputs (2.4 & 2.5 Left, 2.6 & 2.7 Right) (all 4 are same)
 * P4.1 to P4.4 = Motor direction control outputs (4.1 & 4.2 Left, 4.3 & 4.4 Right) (all 4 are same)
 * P5.1 & P5.2 = Right Quad Encoder A & B signal inputs (left is ignored)
 * P6.0 = Ultrasound Distance Sensor Echo (output from HC-SR04, thru voltage divider, Input into 432)
 * P5.6 = Ultrasound Distance Sensor Trigger (output from 432, input into HC-SR04)
 */


/* DriverLib Includes */
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include <ti/devices/msp432p4xx/inc/msp.h>

/* Standard Includes */
#include <stdint.h>
#include <stdbool.h>
#include "printf.h"

// Comms Vars
static volatile bool chasing_setpoint, gathering, parsingSetpoint, driveCommand_forward;
static volatile uint8_t driveCommand_parseIndex;
static volatile uint16_t parsedValue;

// Quad Encoder Vars LOOK_HERE
static volatile enum enc_state_t{enc_O, enc_A, enc_AB, enc_B} enc_state;
static volatile uint32_t enc_lastInterruptTime;
static volatile int32_t enc_lastPulseDuration;
static volatile int32_t enc_curspeed; // Tells us instantaneous angular velocity; sign = direction, magnitude = 1/(Timer32 pulses that past since last quad encoder interrupt)
static volatile int32_t enc_val; // Tells us encoder position.

// Distance Sensor Vars LOOK_HERE
#define DISTANCE_SLOPE_RISE 279 //Calibrate to your own sensor! See Port6 IRQH to see how these are used.
#define DISTANCE_SLOPE_RUN 208
#define DISTANCE_OFFSET -208
static volatile uint16_t distance_mm; //from distance sensor output
static volatile uint16_t distance_riseTimestamp; //for measuring Echo signal high time
static volatile uint16_t distance_highTime; //from Echo signal; used to calibrate our sensor interpretation easily
static const float distance_filter_factor=0.5; //for part two

// PID Controller Variables LOOK_HERE
#define INITIAL_KP 0
#define INITIAL_KD 0
#define INITIAL_KI 0
static volatile int16_t k_p, k_d, k_i, error, PWM;
static volatile float integrator_val; // I term value, updated whenever PID control signal updates
static volatile float proportional_val; // P term
static volatile float differential_val; // D term
static volatile float PreSatOut; // pre-saturated output
static volatile float distance_mm_intoPI; // output of distance-sensor-filter for part 2, is just distance_mm in part 1
static volatile uint16_t setpoint; // the goal or target
static const float i_decay_rate=0.5; //for the i term's decay //TODO: tune, up to you
static const float control_pwm_scaling=100.0; //to convert control signal into a PWM setting // TODO: tune, up to you

// Live-Tuning Vars LOOK_Here (automatically toggles between two setpoints on a timer, while allowing you to modify your KP KI KD values
static volatile enum tuning_whichK_t{tuning_Kp, tuning_Kd, tuning_Ki} tuning_whichK;
static volatile bool tuning_mode;
static volatile uint16_t tuning_AlternateSetpoint; //(specifies which target you're not approaching; currently approaching setpoint (defined above))
#define TUNING_SETPOINT_A 300 // TODO Choose livetuning setpoints that make sense for your available space
#define TUNING_SETPOINT_B 600
#define TUNING_TIME_PER_STEP 5 //in seconds, max is 32sec.

// Current Measurement Vars
static volatile int16_t current;

// Circular Buffer & Data Packets
typedef struct {
    uint32_t timeStamp;
    int32_t encPos;
    int32_t encCurSpeed;
    uint16_t distance_mm;
    uint16_t current;
    int16_t pwm;
} packet_t;

#define BUFF_LENGTH 100
volatile packet_t circBuff[BUFF_LENGTH];
volatile uint8_t index_WriteNext;
volatile uint8_t index_TransmitNext;

// 115200 Baud rate UART on EUSCI_A0
const eUSCI_UART_Config uartConfig =
{
        EUSCI_A_UART_CLOCKSOURCE_SMCLK,          // SMCLK Clock Source
        8,                                     // BRDIV = 6
        11,                                       // UCxBRF = 8
        0,                                       // UCxBRS = 32
        EUSCI_A_UART_NO_PARITY,                  // No Parity
        EUSCI_A_UART_LSB_FIRST,                  // LSB First
        EUSCI_A_UART_ONE_STOP_BIT,               // One stop bit
        EUSCI_A_UART_MODE,                       // UART mode
        EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION  // Oversampling
};

void configClocks(){
     CS_setDCOFrequency(16000000);
     CS_initClockSignal(CS_MCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
     CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
     CS_setReferenceOscillatorFrequency(CS_REFO_128KHZ);
     CS_initClockSignal(CS_ACLK, CS_REFOCLK_SELECT, CS_CLOCK_DIVIDER_1);
}

uint16_t* currentK(){
    switch (tuning_whichK) {
        case tuning_Kp:
            return &k_p;
        case tuning_Kd:
            return &k_d;
        default: // tuning_Ki:
            return &k_i;
    }
}

void packageDataOntoBuffer(){
    //Populate next package
    volatile packet_t* p = &(circBuff[index_WriteNext]);
    p->timeStamp = Timer32_getValue(TIMER32_1_BASE);
    p->encPos = enc_val; // units: # of encoder pulses offset from start point
    p->encCurSpeed = enc_curspeed; // units: inverse of # of Timer32_1 pulses between most recent two enc interrupts (16000000/256 per second)
                                         // +/- sign corresponds to direction of rotation; note curspeed varies linearly as wheel spin speed.
                                    // we can skip converting this to "real" units, like degrees/sec, because all of those are constant linear factors,
                                    // which we can include into our K_d simply by tuning it like we would have to do anyway.
    p->distance_mm = distance_mm; // units: interpolated distance in millemeters measured by distance sensor
    p->current = current; // Raw 14bit ADC reading relative to 0V to 3.3V range;
                            // current sensor output voltage in Volts correlates 1:1 with relevant motor current in Amps


    if(P4->OUT & GPIO_PIN1){ //If Pin 4.1 output is high, aka if Front Left Motor is going Forward:
        p->pwm = TIMER_A0->CCR[1]; // High time proportion Out of 99; 99/99 corresponds to 100% duty cycle.
    } else {
        p->pwm = TIMER_A0->CCR[1]*-1; // High time proportion Out of 99; 99/99 corresponds to 100% duty cycle.
                                       //Negative PWM indicates reverse direction polarity.
    }


    // Add to buffer
    index_WriteNext++;
    index_WriteNext %= BUFF_LENGTH;
}

void configTimerA3PWMForADC(){
        TIMER_A3->CTL = TIMER_A_CTL_SSEL__ACLK // Sources Timer_A3 from the A Clock ...
                            | TIMER_A_CTL_ID__1 // ... with an input divider of 1 ...
                            | TIMER_A_CTL_MC__UP; // accumulating in Up mode (ccr0 stores period)

        TIMER_A3->CCR[0] = 1280; // 100Hz cycle rate

        //Set CCR for Compare mode with Toggle/Set output mode for PWM generation
        TIMER_A3->CCTL[1] = TIMER_A_CCTLN_OUTMOD_6;

        TIMER_A3->CCR[1]  = 1; //~7us high time
}

void configUART(){
    UART_initModule(EUSCI_A0_BASE, &uartConfig);

    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1,
                GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);

    UART_enableModule(EUSCI_A0_BASE);

    UART_enableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
    Interrupt_enableInterrupt(INT_EUSCIA0);
}

void configADC(){
    ADC14->CTL0 = 0; //disables module first, so that it can be reconfigured
    ADC14->CTL0 = ADC14_CTL0_ON | // turns on ADC
            ADC14_CTL0_CONSEQ_3 | // sequence of sources, repeat sample mode
            ADC14_CTL0_SSEL__SMCLK | //ADC clock sourced from SM clock...
            ADC14_CTL0_DIV__1 | //with divider 1 ...
            ADC14_CTL0_PDIV__1 | //and predivider 1.
            ADC14_CTL0_SHS_7; //Sample-And-Hold (SHI) Signal sourced from TA3 CCR1's output


    ADC14->CTL1 = ADC14_CTL1_RES__14BIT | //14bit resolution
            (0 << ADC14_CTL1_CSTARTADD_OFS); //samples starting at ADCMEM[0]

    ADC14->MCTL[0] = ADC14_MCTLN_INCH_0 | //A0 sample/converts into ADCMEM[0] FOR RIGHT MOTOR CURRENTS
            ADC14_MCTLN_EOS | //ADCMEM[0] is the last in the sequence
            ADC14_MCTLN_VRSEL_0; //using reference voltages of ground and 3.3V

    ADC14->CLRIFGR0 = ADC14_IER0_IE0; // Clears interrupt flag to avoid spurious interrupt
    ADC14->IER0 = ADC14_IER0_IE0; //Enables interrupt from ADCMEM[1] finishing conversion.

    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P5, GPIO_PIN5, GPIO_TERTIARY_MODULE_FUNCTION);
    //A0 is analog input on P5.5

    Interrupt_enableInterrupt(INT_ADC14);
}
//Configures Timer_A1 for use in Live Tuning mode as the regularly occuring 5second timer
void configTA1forLiveTuning(){
    TIMER_A1->CTL = TIMER_A_CTL_SSEL__ACLK //Sourced from AClock
                    | TIMER_A_CTL_ID__8 // input divider of 8
                    | TIMER_A_CTL_CLR //Clear interrupts
                    | TIMER_A_CTL_IE; // enable interrupt

    TIMER_A1->EX0 = TIMER_A_EX0_IDEX__8; //cumulative predivider of 64; counts at 2000 pulses per second

    TIMER_A1->CCTL[0] = TIMER_A_CCTLN_CCIE; // Enables CCR0's Interrupt (For TA2_0 interrupt)
    TIMER_A1->CCR[0] = TUNING_TIME_PER_STEP * 2000; // interrupt every [insert number from define] seconds

    Interrupt_enableInterrupt(INT_TA1_0);

    tuning_mode = false;
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2);
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2);
    tuning_whichK = tuning_Kp;
    //LOOK_HERE XD
    k_p = INITIAL_KP;
    k_d = INITIAL_KD;
    k_i = INITIAL_KI;
}

// Configures Timer_A2 to both trigger the pulsing of and time the output from the Ultrasound distance sensor.
// Actual time sampling draws from GPIO input interrupts on P6.0, in contrast to lecture.
// Measures distance at 20Hz update cycle; triggers Timer_A2 CCR0's interrupt (TA2_0_IRQHandler, not TA2_N_IRQHandler) for transmission
void configTA2andGPIOsForUltrasoundDistSensor(){
    TIMER_A2->CTL = TIMER_A_CTL_SSEL__ACLK // Sources Timer_A2 from the A Clock ...
                    | TIMER_A_CTL_ID__1 // ... with an input divider of 1 ...
                    | TIMER_A_CTL_CLR // Clear Timer_A2 interrupt flag to avoid spurious interrupts
                    | TIMER_A_CTL_IE    // enable Timer_A2's Interrupts at all
                    | TIMER_A_CTL_MC__UP; // accumulating in Up mode (ccr0 stores period)

    TIMER_A2->CCTL[0] = TIMER_A_CCTLN_CCIE; // Enables CCR0's Interrupt (For TA2_0 interrupt)
    TIMER_A2->CCR[0] = 6400; // 20Hz cycle rate

    //Set CCR for Compare mode with Toggle/Set output mode for PWM generation
    TIMER_A2->CCTL[1] = TIMER_A_CCTLN_OUTMOD_6;
    TIMER_A2->CCR[1]  = 2; //15.625us high time (for >=10us high time TRIGGER line)

    Interrupt_enableInterrupt(INT_TA2_0);

    distance_mm = 0;
    distance_riseTimestamp = 0;

    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P5, GPIO_PIN6, GPIO_PRIMARY_MODULE_FUNCTION);

    // Reading the Echo via GPIO pin 6.0
    GPIO_setAsInputPin(GPIO_PORT_P6, GPIO_PIN0); //no pullup/pulldown resistor needed; signal output from distance sensor
    GPIO_interruptEdgeSelect(GPIO_PORT_P6, GPIO_PIN0, GPIO_LOW_TO_HIGH_TRANSITION); // watching for rising edge
    GPIO_enableInterrupt(GPIO_PORT_P6, GPIO_PIN0);
    Interrupt_enableInterrupt(INT_PORT6);
}

// Sets GPIO pins Outputs, defaults to low
void configGPIOOutput(uint_fast8_t port, uint_fast16_t pins){
    GPIO_setAsOutputPin(port, pins);
    GPIO_setOutputLowOnPin(port, pins);
}

void configTA0forMotorPWM(){
    TIMER_A0->CTL = TIMER_A_CTL_SSEL__SMCLK // Sources Timer_A0 from the SM Clock ...
                        | TIMER_A_CTL_ID__8 // ... with an input divider of 8 ...
                        | TIMER_A_CTL_MC__UP; // accumulating in Up mode (ccr0 stores period)

    TIMER_A0->EX0 = TIMER_A_EX0_IDEX__8; // additional predivider of 8 (64x reduction total) LOOK_HERE

    TIMER_A0->CCR[0] = 99; // 2.5kHz PWM cycle rate

    //Set CCR for Compare mode with Toggle/Set output mode for PWM generation
    TIMER_A0->CCTL[1] = TIMER_A_CCTLN_OUTMOD_6;
    TIMER_A0->CCTL[2] = TIMER_A_CCTLN_OUTMOD_6;
    TIMER_A0->CCTL[3] = TIMER_A_CCTLN_OUTMOD_6;
    TIMER_A0->CCTL[4] = TIMER_A_CCTLN_OUTMOD_6;

    TIMER_A0->CCR[1]  = 0; //0% duty cycle by default
    TIMER_A0->CCR[2]  = 0; //0% duty cycle by default
    TIMER_A0->CCR[3]  = 0; //0% duty cycle by default
    TIMER_A0->CCR[4]  = 0; //0% duty cycle by default


    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7, GPIO_PRIMARY_MODULE_FUNCTION);
}

// Configures the following pins as inputs & initializes relevant state & interrupt edge direction
// P5.1 & P5.2 = Right Quad Encoder A & B signal inputs
void configEncoderGPIOInputs(){
    GPIO_setAsInputPin(GPIO_PORT_P5, GPIO_PIN1 | GPIO_PIN2);

    GPIO_enableInterrupt(GPIO_PORT_P5, GPIO_PIN1 | GPIO_PIN2);

    Interrupt_enableInterrupt(INT_PORT5);
    //No Pull-up or Pull-down resistor needed as input wire voltage is always clearly defined (output from encoder)
    uint8_t initvals = GPIO_getInputPinValue(GPIO_PORT_P5, GPIO_PIN1 | GPIO_PIN2);
    switch (initvals) {
        case 0b00000000:
            enc_state = enc_O;
            GPIO_interruptEdgeSelect(GPIO_PORT_P5, GPIO_PIN1 | GPIO_PIN2, GPIO_LOW_TO_HIGH_TRANSITION);
            break;
        case 0b00000010:
            enc_state = enc_B;
            GPIO_interruptEdgeSelect(GPIO_PORT_P5, GPIO_PIN2, GPIO_LOW_TO_HIGH_TRANSITION);
            GPIO_interruptEdgeSelect(GPIO_PORT_P5, GPIO_PIN1, GPIO_HIGH_TO_LOW_TRANSITION);
            break;
        case 0b00000100:
            enc_state = enc_A;
            GPIO_interruptEdgeSelect(GPIO_PORT_P5, GPIO_PIN1, GPIO_LOW_TO_HIGH_TRANSITION);
            GPIO_interruptEdgeSelect(GPIO_PORT_P5, GPIO_PIN2, GPIO_HIGH_TO_LOW_TRANSITION);
            break;
        case 0b00000110:
            enc_state = enc_AB;
            GPIO_interruptEdgeSelect(GPIO_PORT_P5, GPIO_PIN1 | GPIO_PIN2, GPIO_HIGH_TO_LOW_TRANSITION);
            break;
    }

    // Initialize States:
    enc_val = 0;
    enc_lastInterruptTime = Timer32_getValue(TIMER32_1_BASE);
    enc_curspeed = 0;
}

// Configures Timer32_1 as a long-running timer to measure the speed of Quad encoder interrupts
void configT32_1ForEncoder(){
    Timer32_initModule(TIMER32_1_BASE, TIMER32_PRESCALER_256, TIMER32_32BIT, TIMER32_FREE_RUN_MODE);
    // Not bothering to enable Timer32_1's interrupt as rover doesn't operate for long enough for it to run out.
}

int main(void)
{
    WDT_A_holdTimer();

    //init
    gathering = true;
    driveCommand_parseIndex = 0;
    parsedValue = 0;
    chasing_setpoint = false;

    // config basics:
    configClocks();

    //config - motor control
    configGPIOOutput(GPIO_PORT_P4, GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4);
    GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4);

    configTA0forMotorPWM();

    // config comms
    configUART();

    // config ADC
    configADC();

    configTimerA3PWMForADC();

    // config Sensors & Encoders
    configTA2andGPIOsForUltrasoundDistSensor();

    configEncoderGPIOInputs();
    configT32_1ForEncoder();

    // config live tuning:
    configTA1forLiveTuning();

    // Setup Circular Buffer
    index_TransmitNext = 0;
    index_WriteNext = 0;
    Interrupt_disableSleepOnIsrExit();

    Timer32_startTimer(TIMER32_1_BASE, 1); // For timing encoder changes & datapoints
    Timer_A_startCounter(TIMER_A2_BASE, TIMER_A_UP_MODE); // Distance sensor enabled
    ADC14->CTL0 |= ADC14_CTL0_ENC; // enables sampling/conversion for motor currents

    Interrupt_enableMaster();

    /* Going to sleep */
    while (1)
    {
        //Check if there's packets remaining to be sent:
        if(index_WriteNext != index_TransmitNext){
            packet_t p = circBuff[index_TransmitNext];
            printf(EUSCI_A0_BASE,"%n,%l,%l,%u,%u,%i\n\r",
                   p.timeStamp, p.encPos, p.encCurSpeed, p.distance_mm, p.current, p.pwm); // takes a while
            index_TransmitNext++;
            index_TransmitNext %= BUFF_LENGTH; //wraparound the circular buffer
        } else {
            PCM_gotoLPM0InterruptSafe(); // 'Shallowest' Low-Power mode; leaves all peripherals on at full frequency anticipating a relatively-soon wakeup.
        }
    }
}

// Triggered reliably at 20Hz
void TA2_0_IRQHandler(void){
    TIMER_A2->CCTL[0] &= ~TIMER_A_CCTLN_CCIFG; // Clears interrupt flag on TA2's CCR0,

    if(!gathering) // dont even package data if not supposed to be gathering
        return;

    // gather/handle this period's sensor values for transmission
    packageDataOntoBuffer();
}

void TA1_0_IRQHandler(void){
    TIMER_A1->CCTL[0] &= ~TIMER_A_CCTLN_CCIFG;

    uint16_t temp = setpoint;
    setpoint = tuning_AlternateSetpoint;
    tuning_AlternateSetpoint = temp;
}

// sets PWM duty cycle & direction pins for specified constant drive commands like F, x, and DSXX!
void driveIndefinitely(bool forward, uint16_t PWM){ //LOOK_HERE
    if(forward){
        GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4);
    } else {
        GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4);
    }


    TIMER_A0->CCR[1] = PWM;
    TIMER_A0->CCR[2] = PWM;
    TIMER_A0->CCR[3] = PWM;
    TIMER_A0->CCR[4] = PWM;
}

// TODO: Calculate the PID controller values and output, calculates the new PWM and direction settings, and applies those changes.\
// Called after any relevant new information has been acquired; left quad encoder interrupts and also new distance sensor values acquired.

void updatePIDController(){

    if(!chasing_setpoint){
        return;
    }
    //TODO: Update this in part two, when you filter your distance sensor's output:
    distance_mm_intoPI = distance_mm;

    //Todo: calc P, D, and I terms using the global K constants and relevant global sensor output values
    error = setpoint - distance_mm_intoPI; /* Compute the error */ \
    proportional_val = k_p * error; /* Compute P */ \
    integrator_val = integrator_val + k_i*proportional_val; /* Compute I */ //TODO
    differential_val = k_d *  enc_curspeed; //TODO
    PreSatOut = abs((proportional_val + integrator_val + differential_val)*control_pwm_scaling);

    // TODO Set Motor Direction Pins (4.1 & 4.2 left, 4.3 & 4.4 right) to match the control signal's sign
    // Set PWM Duty Cycles (percentages) as a saturating linear transformation of your control signal outputs

    if (PreSatOut > 100) /* Saturate output */ \
    {PWM = 100;} \
    else if (PreSatOut < 20)
    {PWM = 20;}
    else
    {PWM = PreSatOut;}

    if(error>>0)// go backwards
        driveIndefinitely(false,PWM);
    else
        driveIndefinitely(true, PWM);

}

void PORT6_IRQHandler(){

    uint_fast16_t status = GPIO_getEnabledInterruptStatus(GPIO_PORT_P6);
    GPIO_clearInterruptFlag(GPIO_PORT_P6, status);

    //Record Timer_A2 value on both ends of the sensor's output pulse, for later transmission
    if(status & GPIO_PIN0){
        if(distance_riseTimestamp == 0){ // we haven't seen the rising edge yet this cycle
            distance_riseTimestamp = TIMER_A2->R; // Record current timestamp of timerA
            GPIO_interruptEdgeSelect(GPIO_PORT_P6, GPIO_PIN0, GPIO_HIGH_TO_LOW_TRANSITION); // watch for falling edge
        } else {// already saw the rising edge, this is falling edge
            uint16_t curtime = TIMER_A2->R;
            //distance_mm = (curtime - distance_riseTimestamp)/128/10^6*340/2 // this is in mm/ms, because TIMER_A2 is sourced from 128kHz Aclock
            distance_highTime = (curtime - distance_riseTimestamp);
            distance_mm = DISTANCE_SLOPE_RISE * (distance_highTime - DISTANCE_OFFSET) / DISTANCE_SLOPE_RUN; //LOOK_HERE

            // set up for next distance reading
            distance_riseTimestamp = 0;
            GPIO_interruptEdgeSelect(GPIO_PORT_P6, GPIO_PIN0, GPIO_LOW_TO_HIGH_TRANSITION);

            //LOOK_HERE (updates the P and I terms, off of the new distance value)
            updatePIDController();
        }
    }
}

// For Right Quad Encoder
void PORT5_IRQHandler(){
    uint_fast16_t status = GPIO_getEnabledInterruptStatus(GPIO_PORT_P5);
    GPIO_clearInterruptFlag(GPIO_PORT_P5, status);

    if(status & GPIO_PIN1){ //Interrupt on A
        uint32_t curtime = Timer32_getValue(TIMER32_1_BASE);

        switch (enc_state) {
            case enc_O: //00 to 10 Transition
                enc_val++;
                enc_curspeed = INT32_MAX / (enc_lastInterruptTime - curtime);
                enc_state = enc_A;
                GPIO_interruptEdgeSelect(GPIO_PORT_P5, GPIO_PIN1, GPIO_HIGH_TO_LOW_TRANSITION);
                break;
            case enc_A: //10 to 00 Transition
                enc_val--;
                enc_curspeed = INT32_MAX / (-1 * (enc_lastInterruptTime - curtime));
                enc_state = enc_O;
                GPIO_interruptEdgeSelect(GPIO_PORT_P5, GPIO_PIN1, GPIO_LOW_TO_HIGH_TRANSITION);
                break;
            case enc_AB: //11 to 01 Transition
                enc_val++;
                enc_curspeed = INT32_MAX / (enc_lastInterruptTime - curtime);
                enc_state = enc_B;
                GPIO_interruptEdgeSelect(GPIO_PORT_P5, GPIO_PIN1, GPIO_LOW_TO_HIGH_TRANSITION);
                break;
            case enc_B: //01 to 11 Transition
                enc_val--;
                enc_curspeed = INT32_MAX / (-1 *(enc_lastInterruptTime - curtime));
                enc_state = enc_AB;
                GPIO_interruptEdgeSelect(GPIO_PORT_P5, GPIO_PIN1, GPIO_HIGH_TO_LOW_TRANSITION);
                break;
        }
        enc_lastInterruptTime = curtime;
    }
    if(status & GPIO_PIN2){ //Interrupt on B
        uint32_t curtime = Timer32_getValue(TIMER32_1_BASE);

        switch (enc_state) {
            case enc_O: //00 to 01 Transition
                enc_val--;
                enc_curspeed = INT32_MAX / (-1 * (enc_lastInterruptTime - curtime));
                enc_state = enc_B;
                GPIO_interruptEdgeSelect(GPIO_PORT_P5, GPIO_PIN2, GPIO_HIGH_TO_LOW_TRANSITION);
                break;
            case enc_A: //10 to 11 Transition
                enc_val++;
                enc_curspeed = INT32_MAX / (enc_lastInterruptTime - curtime);
                enc_state = enc_AB;
                GPIO_interruptEdgeSelect(GPIO_PORT_P5, GPIO_PIN2, GPIO_HIGH_TO_LOW_TRANSITION);
                break;
            case enc_AB: //11 to 10 Transition
                enc_val--;
                enc_curspeed = INT32_MAX / (-1*(enc_lastInterruptTime - curtime));
                enc_state = enc_A;
                GPIO_interruptEdgeSelect(GPIO_PORT_P5, GPIO_PIN2, GPIO_LOW_TO_HIGH_TRANSITION);
                break;
            case enc_B: //01 to 00 Transition
                enc_val++;
                enc_curspeed = INT32_MAX / (enc_lastInterruptTime - curtime) ;
                enc_state = enc_O;
                GPIO_interruptEdgeSelect(GPIO_PORT_P5, GPIO_PIN2, GPIO_LOW_TO_HIGH_TRANSITION);
                break;
        }
        enc_lastInterruptTime = curtime;
    }
    updatePIDController();
}

void EUSCIA0_IRQHandler(void){
    uint32_t status = UART_getEnabledInterruptStatus(EUSCI_A0_BASE);
    UART_clearInterruptFlag(EUSCI_A0_BASE, status);

    if(status & EUSCI_A_IFG_RXIFG){
        char readchar = UART_receiveData(EUSCI_A0_BASE);

        if(!tuning_mode || readchar == 'L'){ //Avoid disrupting Matlab parsing during live tuning
            printf(EUSCI_A0_BASE, "%c", readchar);
        }
        bool invalidCommand = true;

        // Drive command parsing: DSXX!
        switch(driveCommand_parseIndex){
        case 0:
            // Haven't received 'D' yet => not in Drive command yet
            break;
        case 1:
            //Anticipating '+' or '-' to indicate direction of motors
            if(readchar == '+'){
                invalidCommand = false;
                driveCommand_parseIndex++;
                driveCommand_forward = true;
            } else if(readchar == '-'){
                invalidCommand = false;
                driveCommand_parseIndex++;
                driveCommand_forward = false;
            } else{
                driveCommand_parseIndex = 0;
                printf(EUSCI_A0_BASE,"Invalid command.\n\r");
            }
            parsedValue = 0;
            break;
        case 2:
        case 3:
            //accumulating the PWM value for left side; 2 numeric digits expected
            if(readchar >= '0' && readchar <= '9'){
                int16_t newdigit = readchar - '0';
                parsedValue = parsedValue * 10 + newdigit;
                invalidCommand = false;
                driveCommand_parseIndex++;
            } else {
                driveCommand_parseIndex = 0;
                printf(EUSCI_A0_BASE,"Invalid command.\n\r");
            }
            break;
        case 4:
            //  awaiting ! (indicating an indefinite drive command, executed now)
            // all else is invalid
            if(readchar == '!'){
                invalidCommand = false;
                driveCommand_parseIndex = 0;
                driveIndefinitely(driveCommand_forward, parsedValue);
                printf(EUSCI_A0_BASE, "Driving at %i%% duty cycle.\n\r",
                       (driveCommand_forward ? parsedValue : -1 * parsedValue));
            } else{
                driveCommand_parseIndex = 0;
                printf(EUSCI_A0_BASE,"Invalid command.\n\r");
            }
            break;
        }

        // Setpoint Parsing:
        // Format: s#!, potentially-multidigit positive numeral
        if(parsingSetpoint){
            if(readchar >= '0' && readchar <= '9'){
                int16_t newdigit = readchar - '0';
                parsedValue = parsedValue * 10 + newdigit;
                invalidCommand = false;
            } else if(readchar == '!'){
                invalidCommand = false;
                parsingSetpoint = false;
                setpoint = parsedValue;
                printf(EUSCI_A0_BASE, "Set: %u\n\r",setpoint);
                chasing_setpoint = true;
            } else {
                parsingSetpoint = false;
                printf(EUSCI_A0_BASE,"Invalid command.\n\r");
            }
        }

        // Command First Character Parsing
        switch(readchar){
        case 'g':
            if(gathering){
                gathering = false;
                printf(EUSCI_A0_BASE, "Data Gathering Paused.\n\r");
            } else {
                gathering = true;
            }
            invalidCommand = false;
            break;
        case 'z':
            enc_val=0;
            printf(EUSCI_A0_BASE,"Zeroed.\n\r");
        case 's':
            parsingSetpoint = true;
            parsedValue = 0;
            invalidCommand = false;
            break;
        case 'F':
            driveIndefinitely(true, 100); // Full speed, forward.
            printf(EUSCI_A0_BASE, "Driving at 100%% duty cycle.\n\r");
            invalidCommand = false;
            chasing_setpoint = false;
            break;
        case 'x':
            driveIndefinitely(true, 0); //Stop.
            printf(EUSCI_A0_BASE, "Halting.\n\r");
            chasing_setpoint = false;
            invalidCommand = false;
            break;
        case 'D':
            driveCommand_parseIndex = 1;
            invalidCommand = false;
            break;
        case 'L':
            if(!tuning_mode){
                tuning_mode=true;
                tuning_whichK = tuning_Kp;
                GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN0);
                GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN1 | GPIO_PIN2);
                printf(EUSCI_A0_BASE, "Entered Tuning Mode.\n\r");
                Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_UP_MODE);
                chasing_setpoint = true;
                tuning_AlternateSetpoint = TUNING_SETPOINT_B;
                setpoint = TUNING_SETPOINT_A;
            }else{
                tuning_mode=false;
                chasing_setpoint = false;
                printf(EUSCI_A0_BASE, "Tuning Complete.\n\r");
                Timer_A_stopTimer(TIMER_A1_BASE);
                GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2);
            }
            invalidCommand = false;
            break;
        case 'u':
            if(!tuning_mode){
                printf(EUSCI_A0_BASE, "Not tuning.\n\r");
            } else {
                switch (tuning_whichK) {
                    case tuning_Kp:
                        tuning_whichK = tuning_Kd;
                        GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0);
                        GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN1);
                        break;
                    case tuning_Kd:
                        tuning_whichK = tuning_Ki;
                        GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN1);
                        GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN2);
                        break;
                    case tuning_Ki:
                        tuning_whichK = tuning_Kp;
                        GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN2);
                        GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN0);
                        break;
                }
            }
            break;
        case 'r':
            if(!tuning_mode){
                printf(EUSCI_A0_BASE,"Not tuning.\n\r");
                break;
            }
            *currentK() -=100;
            invalidCommand = false;
            break;
        case 't':
            if(!tuning_mode){
                printf(EUSCI_A0_BASE,"Not tuning.\n\r");
                break;
            }
            *currentK() -=100;
            invalidCommand = false;
            break;
        case 'y':
            if(!tuning_mode){
                printf(EUSCI_A0_BASE,"Not tuning.\n\r");
                break;
            }
            *currentK() -=1;
            invalidCommand = false;
            break;
        case 'i':
            if(!tuning_mode){
                printf(EUSCI_A0_BASE,"Not tuning.\n\r");
                break;
            }
            *currentK() +=1;
            invalidCommand = false;
            break;
        case 'o':
            if(!tuning_mode){
                printf(EUSCI_A0_BASE,"Not tuning.\n\r");
                break;
            }
            *currentK() +=10;
            invalidCommand = false;
            break;
        case 'p':
            if(!tuning_mode){
                printf(EUSCI_A0_BASE,"Not tuning.\n\r");
                break;
            }
            *currentK() +=100;
            invalidCommand = false;
            break;
        case '\n':
        case '\r':
            invalidCommand = false;
            break;
        }

        if(invalidCommand){
            printf(EUSCI_A0_BASE, "???\n\r");
        }
    }
}

void ADC14_IRQHandler(void)
{
    uint32_t status;

    status = ADC14->IFGR0;
    ADC14->CLRIFGR0 = status;

    if (status & ADC14_IER0_IE0)
    {
        current = ADC14->MEM[0]; //read result of conversion of A0 (right side motor current sensor)
    }
}
