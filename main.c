/*
                                            Vision Aerial Yaw-Mech Degration Test
    See repository for circuit diagram
    This source code is used to control the MSP430FR2310 which runs the controls system for the tester

                    |----------------|
                    |                |  ---> +5V  --------->    |-------|
                    |2310 Controller |  ---> GND  --------->    | Servo |
    USB Power --->  |                |  ---> PWM P1.6   --->    |-------|
                    |                |
                    |                |
                    |                |  ---> PWM P1.7   --->        |-------|                   |---------------|
                    |                |  ---> NC     ------->        | ESC   |    <--- +24V <--- | Benchtop PSU  |
                    |----------------|  ---> GND    ------->        |-------|    <--- GND  <--- |---------------|


*/
#include <msp430.h>

//Some defines to make the LED light control a little easier 
#define Red_LED BIT1;
#define Green_LED BIT0;
#define Blue_LED BIT0;


/*
Function declarations:
    1. Setup_GPIO       -> Sets up the general input/ouput pins needed for the system. See the circuit diagram
    2. Setup_Timers     -> Sets up the timers needed for generating the PWM signals for the Servo and Motor controller
    3. Measurment_Mode  -> Locks the Servo to the center position and disables the PWM for the motor
    4. Run_Mode         -> Cycles through the servo positions and runs the motor for 5 hours
    5. Completed        -> Clear PWM timers, set lights to green, then idle 
*/

void Setup_GPIO(void);
void Setup_Timers(void);
void Measurement_Mode(void);
void Run_Mode(void);
void Completed(void);


// Servo positions are as follows, center, left, center, right
//Initalize in data memory this is being treated as a constant
int Servo_Positions[4] = {1400, 1100, 1400, 2000};
//Initalize the variables needed into data memory, will be set later in the program
unsigned int Servo_Position_Counter;
unsigned int Mode;

//Counter to track running time
unsigned int Time_Out_Counter;
int main(void)
{
    //Initalize variables in program 
    Mode = 0;
    Time_Out_Counter = 0;
    Servo_Position_Counter = 0;
    //call Setup functions, see above
    Setup_GPIO();
    Setup_Timers();
    //Enable gloabl interrupt mask to create PWM with the timers and use the switch 
    __enable_interrupt();
    //Stop Watchdog timer as the requirement to be conitously active and being a "prototype" justifies not feeding the dog regularly. 
    WDTCTL = WDTPW | WDTHOLD; // stop watchdog timer

    //Switch case for the operating modee of the system:
    //Mode 0: Measurement
    //Mode 1: Run 
    //Mode 2: Completed
while(1){
    switch (Mode){

        case 0:
            Measurement_Mode();
            break;
        case 1:
            Start_Motor();
            Run_Mode();
            break;
        case 2:
            Completed();
            break;
        default:
            //This turns the LEDs white if there's an error in the system, this hopefully never actually happens
            P1OUT |= Red_LED;
            P1OUT |= Blue_LED;
            P2OUT |= Green_LED;
            break;
    }
}
    return 0;
}


void Completed(void){
    //Turn off timers
    TB0CTL &= ~MC__UP;
    TB1CTL &= ~MC__UP;
    TB1CTL |= MC__STOP;
    TB0CTL |= MC__STOP;


    //The system completed a 5 hour cycle, set LEDs to Green to indicate this then idle
    P1OUT &= ~Red_LED;
    P2OUT &= ~Blue_LED;
    P1OUT |= Green_LED;

    //Should add a lowpower-mode here but I'll add then when the system is reliable.
    while(1){
        //Do nothing.....
    }
}


void Run_Mode(void)
{
    //Clear LEDs
    P1OUT |= Red_LED;
    P1OUT &= ~Green_LED;
    P2OUT &= ~Blue_LED;
    // Delay for approximately .5 seconds
    int j, i;
    while (Mode == 1)
    {
        //This creates the approximately .5s delay for each side of the measurement
        for (j = 0; j < 7; j++)
        {
            for (i = 0; i < 20000; i++)
                ;
        }
        //Incriment the servo position counter to change to the next position
        Servo_Position_Counter++;
        if (Servo_Position_Counter > 3)
        { // Reset the counter to 0 if greater than 3
            Servo_Position_Counter = 0;
            Time_Out_Counter++;
            //Check if we have completed 9,000 cycles of the center, left, center, right pattern. This represents a 5 hour testing period. 
            if (Time_Out_Counter > 9000)
            {
                Mode = 3;
            }
        }
        TB1CCR1 = Servo_Positions[Servo_Position_Counter];

    }
    
}


void Measurement_Mode(void)
{
    // Set the PWM values for the Servo to hold in the middle.
    TB1CCR1 = 1400; // This is approximately the PWM value for the middle postion of the servo

    // Disable PWM to ESC
    TB0CTL &= ~MC__UP;  // Clear timer mode for the motor PWM timer
    TB0CTL |= MC__STOP; // Turn off the timer for motor PWM

    // Set lights to blue
    P1OUT &= ~Red_LED;
    P1OUT &= ~Green_LED;
    P2OUT |= Blue_LED;
    while(Mode == 0){
        //Nothing
    }
}



void Start_Motor(void){
    // Disable PWM to ESC
    //TB0CTL &= ~MC__UP;  // Clear timer mode for the motor PWM timer
    TB0CTL |= MC__UP; // Turn off the timer for motor PWM
    //Turn off the Blue led, and set the Green and Red LEDs to make a yellow
    P1OUT |= Red_LED;
    P1OUT |= Green_LED;
    P2OUT &= ~Blue_LED;
    
    //Delay the motor PWM start to copy the start sequence the ESC expectsd for flight
    //And adds some additional warning lights for people to prepare for the motor to start!
    unsigned int i, j;
    for(i = 0; i < 28; i++){
        for(j= 0; j < 30000; j++);
        P1OUT ^= Red_LED;
        P1OUT ^= Green_LED;
    }
    TB0CCR1 &= ~750;
    TB0CCR1 |= 2000;
}




void Setup_GPIO(void)
{
    // Switch
    P2DIR |= BIT7;  // Make BIT 7 an input for the rocker switch
    P2OUT &= ~BIT7; // Use pull down resistor on the switch
    //P2IES |= BIT7; //Set H-to-L sensativity (Switch connects to ground)
    P2IFG &= ~BIT7;
    P2IE |= BIT7;
    // Lights
    // Set all LED pins as outputs

    P1DIR |= BIT0;
    P1DIR |= BIT1;
    P1OUT &= ~Red_LED;
    P1OUT &= ~Green_LED;
    P2DIR |= Blue_LED;
    P2OUT &= ~Blue_LED;

    // Setup Servo PWM pin
    P1DIR |= BIT6;
    P1OUT &= ~BIT6;
    // Setup Motor PWM pin
    P1DIR |= BIT7;
    P1OUT &= ~BIT7;
    // Enable IO (take out of low power mode)
    PM5CTL0 &= ~LOCKLPM5;
}

void Setup_Timers(void)
{

    /*
     * Setup the Motor controller timer.
     * This sets the PWM value to 750 which is needed to forge the
     * handshake check of the throttle being at the lowest position
     */
    TB0CTL |= TBCLR;
    TB0CTL |= TBSSEL__SMCLK;
    TB0CTL |= MC__UP;   
    TB0CCR0 |= 20850;       //Attempted using 20,000 for fast PWM however, this clock is a little off and so 20,850 is correct for a 20ms pwm now
    TB0CCR1 |= 750;         //Throttle low position to spoof the Remote -> ESC handshake/safty feature

    TB0CCTL0 |= CCIE;
    TB0CCTL0 &= ~CCIFG;

    TB0CCTL1 |= CCIE;
    TB0CCTL1 &= ~CCIFG;

    /*
     * Setup the Servo controller timer
     * This set the PWM Value to 1400 which is the center point by
     * default. Changing the CCR1 value moves the servo.
     */

    TB1CTL |= TBCLR;
    TB1CTL |= TBSSEL__SMCLK;
    TB1CTL |= MC__UP;
    TB1CCR0 |= 20000;       //Using 20,000 on this timer creates a 50Hz PWM signal which is the standard for fast PWM
    TB1CCR1 |= 1400;        //Set center for starting position

    TB1CCTL0 |= CCIE;
    TB1CCTL0 &= ~CCIFG;

    TB1CCTL1 |= CCIE;
    TB1CCTL1 &= ~CCIFG;

    // End timer setup
}

/*
    Interrupt Service Routines
    Motor PWM controls the PWM signal timings for the ESC
    Servo PWM controls the PWM signal timings for the Servo Position
*/
#pragma vector = TIMER0_B0_VECTOR
__interrupt void Motor_End_PWM(void)
{
    TB0CCTL0 &= ~CCIFG;
    P1OUT |= BIT7;
}
#pragma vector = TIMER0_B1_VECTOR
__interrupt void Motor_Start_PWM(void)
{
    TB0CCTL1 &= ~CCIFG;
    P1OUT &= ~BIT7;
}

#pragma vector = TIMER1_B0_VECTOR
__interrupt void Servo_PWM_START(void)
{
    TB1CCTL0 &= ~CCIFG;
    P1OUT |= BIT6;
}
#pragma vector = TIMER1_B1_VECTOR
__interrupt void Servo_PWM_END(void)
{
    TB1CCTL1 &= ~CCIFG;
    P1OUT &= ~BIT6;
}

/*
Switch interrupt, this toggles the operating mode of the system from measure to run
This interrupt changes if it is looking for H to L change or L to H based on the mode as it is a switch not a button
The system starts with the switch connected to power and disconnecting P2.7, this represents Measurement mode
After the switch is flipped it begins looking for a H to L transition as the switch will disconnect from power leaving P2.7 floating or with a weak pull down resistor
*/
#pragma vector = PORT2_VECTOR
__interrupt void Toggle_Switch(void)
{
    int i;
    if(Mode == 0){
        Mode = 1;
    }
    else if(Mode == 1){
        Mode = 0;
    }
    for(i = 0; i < 1000; i++);
    P2IFG &= ~BIT7;
    
}
