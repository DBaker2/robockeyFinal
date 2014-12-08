/* Name: main.c
 * Author: Ben Weinreb
 * Copyright: Localisation Testing
 * License: November 2014
 */

#include "m_general.h"
#include "m_bus.h"
#include "m_wii.h"
#include "m_rf.h"
#include "m_usb.h"
#include "m_port.h"

#define RX_ADDRESS 0x19 //Receipt address
#define TX_ADDRESS 0x18 //Send address
#define PACKET_LENGTH_SEND 3
#define CHANNEL 1
#define PACKET_LENGTH_READ 10
#define PI 3.14

//ADC
#define m_port_ADDRESS 0x20

//States
#define Listen 1
#define Qualify 2
#define PuckFind 3
#define GoToGoal 4
#define ShootPuck 5
#define Follow 6
#define Celebration 7


//COMMUNICATIONS
#define PLAY -95
#define PAUSE -92
#define COMM_TEST -96


//Trajectory planning States
#define OppArcGoalHigh 1
#define ToArcGoalHigh 2
#define OppArcGoalLow 3
#define ToArcGoalLow 4
#define OppArcEdgeHigh 5
#define ToArcEdgeHigh 6
#define OppArcEdgeLow 7
#define ToArcEdgeLow 8
#define ToSpinEdgeLow 9
#define OppSpinEdgeLow 10
#define ToSpinEdgeHigh 11
#define OppSpinEdgeHigh 12
#define Straight 13


//OTHER CONSTANTS
#define RED 1
#define BLUE 2
#define FORWARD 8
#define POSSESSPUCK 10

#define low 40
#define high 90
#define plow 30
#define phigh 50
#define pvlow 30
#define vlow 18
#define med 60


char buffer[PACKET_LENGTH_READ] = {0,0,0,0,0,0,0,0,0,0}; //data to be received
char send_data[PACKET_LENGTH_SEND] = {0,0,0}; // data to be sent to game controller

//Localisation variables
volatile int State = Listen;


unsigned int star_data[12] = {0,0,0,0,0,0,0,0,0,0,0,0};
unsigned int star_data2[9] = {0,0,0,0,0,0,0,0,0};
float robot_position[2] = {0,0}; // vector for robot position (x and y)
float robot_orientation= 0; // vector for robot orientation (direction fo y-axis)
float robot_orientation_filtered = 0;
volatile bool timer1_flag = 0; // set high when timer1 overflows
float pixel_cm_conversion = 40.0/125.0; // conversion from pixels to cm, TBD
bool valid = 0; // true when position m_wii data is valid
//State variables
float target = 0;  //Target angle for driving across rink
int timer0count = 2; //0xff/100
int leftcommand = 0; //Duty cycle and direction of left motor
int rightcommand = 0; //Duty cycle and direction of right motor
float postarget = 0;
int sign = 0;
int checkside = 0;
int testvar = 0;
float direction = 0;
float opptarget = 1.57; //angle of opponents goal
volatile int lastPin = 0;
volatile int oppgoal = 130; //x position of opponents goal - will  be determined by comm or switch
//ADC variables
//ADC Order = L1, L2, L3, L4, R4, R3, R2 ,R1
float adcoffset[8] = {190,350,350,280,400,250,380,190};
float adcmultiplier[8] = {1,1,1,1,1,1,1,1};
volatile int adcChannel = 0;
volatile float ADCdata[9] = {0,0,0,0,0,0,0,0,0};
int maxADC;
int puckdirr = 0;
int puckdirl = 0;
int L1;
int L2;
int L3;
int L4;
int R1;
int R2;
int R3;
int R4;
int breakBeam;
int dataFlag;
int limitswitch = 0;
bool goalSwitchBlink = 1; // flag to only blink once to confirm goal
int goal = 0; // goal to go to, set by switch (RED and BLUE are defined)
int GoalState;
float robot_orientation_dir;
float robot_orientation_fil = 0;
float x_robot_position_fil;
float y_robot_position_fil;
int OppGoalSign= 1;
float t = .15;
float robot_orientation_old;
float robot_position_x_old;
float robot_position_y_old;
int maxchannel;
int adcflag = 1;
char rf_counter = 0;
bool commFlag = 0;


int stallcount;
int stallup;
int gtgstall;
int redblueswitch = 1;




volatile bool send_flag = 0;

void init(void);
bool find_position(unsigned int star_data[]);
float dot(float v1[], float v2[]);
//void motors_forward(dutyBLeft);
//void right_spin(dutyBLeft);
//void left_spin(dutyARight);
void left_motor(int leftcommand);
void right_motor(int rightcommand);
void red_LED(bool status);
void blue_LED(bool status);
void white_LED(bool status);
void green_LED(bool status);
void yellow_LED(bool status);
void findPuck(void);
void goScore(void);
void stall(void);
void adcHandler(void);
void commHandler(void);

int main(void){
    
    init();
    //    green_LED(OFF);
    //    white_LED(OFF);
    
    while(TRUE) {
        // fiene said this might fix our rf problems
        if (rf_counter >  30) {
            rf_counter = 0;
            m_rf_open(CHANNEL, RX_ADDRESS, PACKET_LENGTH_READ);
        }
        if (m_usb_isconnected()) {
            m_usb_tx_int((int)(robot_position[0])); //X position, whatever that means
            m_usb_tx_string("\t");
            m_usb_tx_int((int)robot_position[1]); //Y position
            m_usb_tx_string("\t");
            m_usb_tx_int((int)(robot_orientation*127/6.3)); //Y position
            m_usb_tx_string("\t");
            m_usb_tx_int((int)(x_robot_position_fil)); //X position, whatever that means
            m_usb_tx_string("\t");
            m_usb_tx_int((int)(y_robot_position_fil)); //Y position
            m_usb_tx_string("\t");
            m_usb_tx_int((int)(robot_orientation_fil*127/6.3)); //Y position
            m_usb_tx_string("\t");
            
            //            m_usb_tx_string("L1=");
            //            m_usb_tx_int(L1);
            //            m_usb_tx_string("  ");
            //            m_usb_tx_string("L2=");
            //            m_usb_tx_int(L2);
            //            m_usb_tx_string("L3 =");
            //            m_usb_tx_int(L3);
            //            m_usb_tx_string("  ");
            //            m_usb_tx_string("L4 =");
            //            m_usb_tx_int(L4);
            //            m_usb_tx_string("  ");
            //            m_usb_tx_string("R1=");
            //            m_usb_tx_int(R1);
            //            m_usb_tx_string("  ");
            //            m_usb_tx_string("R2 =");
            //            m_usb_tx_int(R2);
            //            m_usb_tx_string("  ");
            //            m_usb_tx_string("R3 =");
            //            m_usb_tx_int(R3);
            //            m_usb_tx_string("  ");
            //            m_usb_tx_string("R4 =");
            //            m_usb_tx_int(R4);
            //            m_usb_tx_string("  ");
            
            m_usb_tx_int(leftcommand);
            m_usb_tx_string("  ");
            m_usb_tx_int(rightcommand);
            m_usb_tx_string("\n");
            
        }
        if (commFlag) {
            commHandler();
        }
        
        if(adcflag){
            adcHandler();
            adcflag = 0;
        }
        
        if(send_flag == 1){
            
            //            stallcount++;
            if(stallcount == 9){
                robot_orientation_old = robot_orientation;
                robot_position_x_old = robot_position[0];
                robot_position_y_old = robot_position[1];
                stallcount = 0;
            }
            
            
            if(stallcount == 8 && abs(robot_position_x_old - robot_position[0])<3 && abs(robot_position_y_old-robot_position[1])<3) {
                stallup++;
            }
            
            if(stallup>=9){
                stall();
                stallup = 0;
            }
            
            
            m_wii_read(star_data);
            find_position(star_data);
            
            robot_orientation_fil = robot_orientation;
            x_robot_position_fil = robot_position[0];
            y_robot_position_fil = robot_position[1];
            
            //            if(OppGoalSign==1){ //Reverses the orientation of the robot depending on defended goal so that opponent goal is always at PI/2
            //                robot_orientation_dir = robot_orientation;}
            //            if(OppGoalSign==-1){
            //                if (robot_orientation>0 && robot_orientation<PI){
            //                robot_orientation_dir = robot_orientation + PI;}
            //                if (robot_orientation>PI && robot_orientation<(2*PI)) {
            //                    robot_orientation_dir = robot_orientation - PI;
            //                }
            //            }
            //
            //
            //            robot_orientation_fil = 0*robot_orientation_fil+1*robot_orientation_dir;
            //            x_robot_position_fil = robot_position[0]*-1*OppGoalSign; //Our goal is always the positive side
            //            y_robot_position_fil = robot_position[1]*-1*OppGoalSign; //The left of the rink from our perspective is always negative
            if (1) {
                send_data[0] = (char)robot_position[0];//TX_ADDRESS;
                send_data[1] = (char)robot_position[1];
                send_data[2] = (char)(robot_orientation*127/6.3);
                m_rf_send(TX_ADDRESS, send_data, PACKET_LENGTH_SEND); //Code for sending star data in footnote
            }
            send_flag = 0;
       	}
        
        switch (State) { //** Necessary states for 11/24: 1 = Wait |  2 = drive to opposite side of rink
            case Listen: //Wait for PLAY command
                leftcommand = 0;
                rightcommand = 0;
                left_motor(leftcommand);
                right_motor(rightcommand);
                white_LED(OFF);
                yellow_LED(OFF);
                green_LED(ON);
                red_LED(OFF);
                blue_LED(OFF);
                //                 blink led to confirm which goal is selected (will only occur at startup)
                if (check(PIND, PIN3) && goalSwitchBlink) {
                    //red_LED(ON);
                    OppGoalSign = -1;
                    goal = RED;
                    goalSwitchBlink = 0;
                } else if (!check(PIND, PIN3) && goalSwitchBlink) {
                    //blue_LED(ON);
                    OppGoalSign = 1;
                    goal = BLUE;
                    goalSwitchBlink = 0;
                }
                State = PuckFind;
                break;
                
            case PuckFind:
                //Transition to: Play Command, Puck Lost, Team Lost Puck, Puck Shot
                //Transition from: Got the Puck, Team has Puck
                
                white_LED(OFF);
                green_LED(OFF);
                yellow_LED(ON);
                findPuck();
                rightcommand = (puckdirr);
                leftcommand = (puckdirl);
                left_motor(leftcommand);
                right_motor(rightcommand);
                break;
                
            case GoToGoal:
                //Transition to: Got the Puck, Run into Opponent??
                //Transition from: No Obstacles, Lost the puck
                //green_LED(OFF);
                //blue_LED(OFF);
                
                white_LED(ON);
                green_LED(OFF);
                yellow_LED(OFF);
                goScore();
                
                
                // if we lose the puck, go back to looking for it.
                if (breakBeam > 900) {
                    State = PuckFind;
                }
                break;
                
            case Follow:
                
                //Transition to: Team has Puck
                //Transition from: Team lost Puck
                break;
                
                
            case ShootPuck:
                //Transition to: No Obstacles
                //Transition from: Default to Puck seek
                
                break;
                
            case Celebration:
                //Transition to: Goal interrupt
                //Transition from: Go to wait
                
                break;
                
            default:
                State = Listen;
                
                break;
                
        }
        
        // send adc data
        
        
        // dataFlag = 0;
        
    }
    
    
}

// Background interrupts: Timer 0: 10hz interrupt finds location and puck direction | RF interrupt for Play/Pause/Listen, Enemy Position, and Teammate Commands

//Initialize
void init(void) {
    //Kicker clear pin
    set(DDRD, PIN5);
    clear(PORTD,PIN5);
    
    
    m_clockdivide(0); // 16 MHz clock
    m_disableJTAG();
    
    m_bus_init();
    m_usb_init();
    
    //Set pins for directional motor output
    set(DDRB,PIN0); //Left Motor set forward
    set(DDRB,PIN1); //Left Motor clear forward
    set(DDRB,PIN2); //Right Motor set forward
    set(DDRB,PIN3); //Right Motor clear forward
    
    //pins for status LEDS
    set(DDRB, PIN4); // output pin for red LED
    set(DDRC, PIN6); // output pin for blue LED
    clear(DDRD, PIN3); //input for goal switch
    
    // open and initialize rf communications
    m_rf_open(CHANNEL, RX_ADDRESS, PACKET_LENGTH_READ);
    
    // open and initialize m_wii communication
    m_wii_open();
    
    // timer 1 set up (running at approx 10Hz)
    //    set(DDRB,PIN7);
    //    OCR1C = 0; //Interrupt when TCNT1 = OCR1C = 0
    
    clear(TCCR1B, CS12); // prescaler to /64
    set(TCCR1B, CS11);   // ^
    set(TCCR1B, CS10);  // ^
    
    set(TCCR1B, WGM13); // (mode 4) UP to OCR1A
    set(TCCR1B, WGM12);   // ^
    set(TCCR1A, WGM11); // ^
    set(TCCR1A, WGM10); // ^
    
    set(TIMSK1, TOIE1); // interrupt every overflow
    
    OCR1A = 2500; // timer up to this number
    
    sei();
    
    // Timer 4 for PWM
    
    OCR4A = 0; // initialize duty cycle to zero
    OCR4B = 0;
    OCR4C = 200;
    
    set(DDRC,PIN7); //Compare A pin
    set(DDRB,PIN6); //Compare B pin
    
    
    set(TCCR4B, CS43); //  set prescaler to /256
    set(TCCR4B, CS42); // ^
    clear(TCCR4B, CS41); // ^
    clear(TCCR4B, CS40); // ^
    
    clear(TCCR4D, WGM41); // Up to OCR4C, PWM mode
    clear(TCCR4D, WGM40); // ^
    
    //Output compare A for PWM of Left Motor ** PIN C7
    set(TCCR4A, PWM4A); // clear at OCR4A, set at 0xFF
    set(TCCR4A, COM4A1); // ^
    clear(TCCR4A, COM4A0); // ^
    
    //Output compare B for PWM of Right Motor  ** PIN B6
    set(TCCR4A, PWM4B); // clear at OCR4B, set at 0xFF
    set(TCCR4A, COM4B1); // ^
    clear(TCCR4A, COM4B0); // ^
    
    //ADC init
    // set the reference voltage to V_cc (5V)
    clear(ADMUX,REFS1);
    set(ADMUX,REFS0);
    
    // set the ADC prescaler to /128
    set(ADCSRA,ADPS2);
    set(ADCSRA,ADPS1);
    set(ADCSRA,ADPS0);
    
    // disable digital inputs
    set(DIDR0,ADC0D);
    set(DIDR0,ADC1D);
    set(DIDR0,ADC4D);
    set(DIDR0,ADC5D);
    set(DIDR0,ADC6D);
    set(DIDR0,ADC7D);
    set(DIDR2,ADC8D);
    set(DIDR2,ADC9D);
    set(DIDR2,ADC10D);
    
    //enable interrupt
    
    clear(ADCSRB,MUX5);
    clear(ADMUX,MUX2);
    clear(ADMUX,MUX1);
    clear(ADMUX,MUX0);
    set(ADCSRA,ADIE);
    
    //Start by reading from F0
    clear(ADCSRB,MUX5);
    clear(ADMUX,MUX2);
    clear(ADMUX,MUX1);
    clear(ADMUX,MUX0);
    
    // start conversion process
    set(ADCSRA,ADEN); //enable
    
    
    /////////////////////////// m_port register G to output //////////////////////
    m_port_set(m_port_ADDRESS,DDRG,0);
    m_port_set(m_port_ADDRESS,DDRG,1);
    m_port_clear(m_port_ADDRESS,DDRG,2);
    m_port_set(m_port_ADDRESS,DDRG,3);
    m_port_set(m_port_ADDRESS,DDRG,4);
    m_port_set(m_port_ADDRESS,DDRG,5);
    m_port_set(m_port_ADDRESS,DDRG,6);
    m_port_set(m_port_ADDRESS,DDRG,7);
    m_port_set(m_port_ADDRESS,DDRH,0);
    
    //ADC multiplier
    int i = 0;
    for (i = 0; i < 8; i++){
        adcmultiplier[i] = 1024/(1024-adcoffset[i]);
    }
    
}

// identify stars and find position and orientation. store to robot_position & robot_orientation
// returns true if all stars found, and false otherwise
bool find_position(unsigned int star_data[]) {
    float dist[4][4] = {{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0}}; // array for dsitances between stars
    float dist2[3][3] ={{0,0,0},{0,0,0},{0,0,0}};
    float robot_y_axis[2]= {1,0}; // robot reference frame y-axis
    float B_ratio = 2; // ratio of AB to CB
    float D_ratio = 1.25; // ratio of AD to CD
    float tol = .1; // tolerance to match ratio
    int a_guess = 0; // first guess for a
    int c_guess = 0; // first guess for c
    int A = 0; // index for A (x at 3*A, y at ((3*A)+1))
    int B = 0; // index for B ^
    int C = 0; // index for C ^
    int D = 0; // index for D ^
    int j=0;
    int max = 0;
    int k = 0;
    int LocIndex = 0;
    int LocState = 1;
    
    // check if all stars are found, if so State = 1, if not State = 2
    int i = 0;
    for (i = 0; i < 11; i++) {
        if (star_data[i] == 1023) {
            LocIndex = LocIndex+1;
        }
    }
    
    if (LocIndex < 2) {
        LocState = 1;}
    if (LocIndex == 2) {
        LocState = 2 ;}
    if (LocIndex > 2) {
        LocState = 3;}
    
    //    if (LocIndex=2) {
    //        LocState = 2;}
    //
    //    if (LocIndex>=4) {
    //        LocState = 3;}
    
    
    
    switch (LocState) {
        case 1: //ALL STARS FOUND
            m_green(ON);
            m_red(OFF);
            // calculate all distances and store them in array, find maximum of these distances
            for (i = 0; i < 4; i++) {
                for (j = 0; j < 4; j++) {
                    dist[i][j] = sqrtf(powf(((float)star_data[3*i] - (float)star_data[3*j]),2) + powf(((float)star_data[(3*i)+1] - (float)star_data[(3*j)+1]),2));
                    if (dist[i][j] > max) {
                        max = dist[i][j];
                        a_guess = i;
                        c_guess = j;
                    }
                    
                }
                
            }
            
            
            // Identify A and C (using fact the B and D are both father from C than A)
            
            for (k = 0; k < 4; k++) {
                // only examine distances not between A and C
                if ((k != a_guess) && (k != c_guess)) {
                    if (dist[a_guess][k] < dist[c_guess][k]) {
                        A = a_guess;
                        C = c_guess;
                    } else {
                        A = c_guess;
                        C = a_guess;
                    }
                    break;
                }
            }
            
            // Identify B and D (using ratio, now that A and C are known)
            for (k = 0; k < 4; k++) {
                if ((k != A) && (k != C)) {
                    float r = (dist[C][k])/(dist[A][k]);
                    if ((r > (B_ratio-tol)) && (r < (B_ratio+tol))) {
                        B = k;
                        D = 10 - (A + B + C);
                    } else {
                        D = k;
                        B = 10 - (A + C + D);
                    }
                    break;
                }
            }
            
            // extract A and C positions from data
            float Ax = star_data[3*A];
            float Ay = star_data[(3*A)+1];
            float Cx = star_data[3*C];
            float Cy = star_data[(3*C)+1];
            float Bx = star_data[3*B]; //Non axial star data for plotting raw star data
            float By = star_data[(3*B)+1];
            float Dx = star_data[3*D];
            float Dy = star_data[(3*D)+1];
            
            
            
            // calculate origin
            float ox = (Ax + Cx)/2;
            float oy = (Ay + Cy)/2;
            
            // calculate rotation from robot frame to star frame
            float theta = (-1)*atan2((Ax-Cx),(Ay-Cy));		// these arguments should be doubles, check here if there is a problem
            
            
            // calculate robot position using output from homogeneous transform matric
            float x = -1*((-1)*cos(theta)*(ox-512)-sin(theta)*(oy-384)); // added an extra inversion
            float y = sin(theta)*(ox-512)-cos(theta)*(oy-384);
            
            
            // store values to arrays (defined above so the can be accessed by main, C functions can't return an array)
            robot_position[0] = pixel_cm_conversion*x; // convert from pixels to cm
            robot_position[1] = pixel_cm_conversion*y; // convert from pixels to cm
            if (theta < 0) {
                robot_orientation = (2*PI) + theta;
            } else {
                robot_orientation = theta;
            }
            
            break;
            
        case 2: //3-star case. If star_data[i] = 1023, throw out data point, make new array, assume axial stars. Will later perform ratio calculations on the other three to determine axial stars, rotate that axis.
            
            m_green(ON);
            m_red(ON);
            
            int skip = 0;
            int i =0;
            
            for (i=0; i<4; i++) {
                if (star_data[3*i] != 1023) {
                    star_data2[2*i-2*skip] = star_data[3*i];
                    star_data2[2*i+1-2*skip] = star_data[3*i+1];
                }
                else {skip++;}
            }
            
            for (i = 0; i < 3; i++) {
                for (j = 0; j < 3; j++) {
                    dist2[i][j] = sqrtf(powf(((float)star_data2[2*i] - (float)star_data2[2*j]),2) + powf(((float)star_data2[(2*i)+1] - (float)star_data2[(2*j)+1]),2));
                    if (dist2[i][j] > max) {
                        max = dist2[i][j];
                        a_guess = i;
                        c_guess = j;
                        
                    }
                    
                }
                
            }
            
            
            // Identify A and C (using fact the B and D are both father from C than A)
            for (k = 0; k < 3; k++) {
                // only examine distances not between A and C
                if ((k != a_guess) && (k != c_guess)) {
                    if (dist2[a_guess][k] < dist2[c_guess][k]) {
                        A = a_guess;
                        C = c_guess;
                    } else {
                        A = c_guess;
                        C = a_guess;;
                    }
                    break;
                }
            }
            
            
            
            //            // Identify B and D (using ratio, now that A and C are known)
            //            for (k = 0; k < 4; k++) {
            //                if ((k != A) && (k != C)) {
            //                    float r = (dist[C][k])/(dist[A][k]);
            //                    if ((r > (B_ratio-tol)) && (r < (B_ratio+tol))) {
            //                        B = k;
            //                        D = 10 - (A + B + C);
            //                    } else {
            //                        D = k;
            //                        B = 10 - (A + C + D);
            //                    }
            //                    break;
            //                }
            //            }
            
            // extract A and C positions from data
            float Ax1 = star_data2[3*A];
            float Ay1 = star_data2[(3*A)+1];
            float Cx1 = star_data2[3*C];
            float Cy1 = star_data2[(3*C)+1];
            //            float Bx = star_data[3*B]; //Non axial star data for plotting raw star data
            //            float By = star_data[(3*B)+1];
            //            float Dx = star_data[3*D];
            //            float Dy = star_data[(3*D)+1];
            //
            //
            //
            // calculate origin
            float ox1 = (Ax1 + Cx1)/2;
            float oy1 = (Ay1 + Cy1)/2;
            
            // calculate rotation from robot frame to star frame
            float theta1 = (-1)*atan2((Ax1-Cx1),(Ay1-Cy1));		// these arguments should be doubles, check here if there is a problem
            
            if (theta < 0) {
                robot_orientation = (2*PI) + theta;
            } else {
                robot_orientation = theta;
            }
            
            // calculate robot position using output from homogeneous transform matric
            float x1 = -1*((-1)*cos(theta1)*(ox1-512)-sin(theta1)*(oy1-384)); // added an extra inversion
            float y1 = sin(theta1)*(ox1-512)-cos(theta1)*(oy1-384);
            
            // orientation measured relative to rink coordinate frame in radians, counter-clockwise
            float orientation1 = theta1; // no longer inverting
            
            // store values to arrays (defined above so the can be accessed by main, C functions can't return an array)
            robot_position[0] = pixel_cm_conversion*x1; // convert from pixels to cm
            robot_position[1] = pixel_cm_conversion*y1; // convert from pixels to cm
            robot_orientation = orientation1;
            
            break;
            
        case 3:
            m_red(ON);
            m_green(OFF);
            return false;
            
        default:
            LocState = 3;
            
            break;
    }
    // data is valid, return true
    return TRUE;
}

// computes dot product of two vectors
float dot(float v1[2], float v2[2]) {
    int i = 0;
    float result = 0;
    for (i = 0; i < 2; i++) {
        result = result + (v1[i]*v2[i]);
    }
    
    return result;
}

void adcHandler(void) {
    if (adcChannel == 0){
        L1  = (ADC-adcoffset[adcChannel])*adcmultiplier[adcChannel];
        adcChannel++;
        clear(ADCSRA,ADEN);     // ADC must be disabled when switching channels
        // Read from Pin F1 next
        clear(ADCSRB,MUX5);
        clear(ADMUX,MUX2);
        clear(ADMUX,MUX1);
        set(ADMUX,MUX0);
        set(ADCSRA,ADIF);
        
    } else if (adcChannel == 1){
        L2 = (ADC-adcoffset[adcChannel])*adcmultiplier[adcChannel];
        adcChannel++;
        clear(ADCSRA,ADEN);     // ADC must be disabled when switching channels
        // Read from Pin F4 next
        clear(ADCSRB,MUX5);
        set(ADMUX,MUX2);
        clear(ADMUX,MUX1);
        clear(ADMUX,MUX0);
        set(ADCSRA,ADIF);
        
    } else if (adcChannel == 2){
        L3 = (ADC-adcoffset[adcChannel])*adcmultiplier[adcChannel];
        adcChannel++;
        clear(ADCSRA,ADEN);     // ADC must be disabled when switching channels
        // Read from Pin F5 next
        clear(ADCSRB,MUX5);
        set(ADMUX,MUX2);
        clear(ADMUX,MUX1);
        set(ADMUX,MUX0);
        set(ADCSRA,ADIF);
        
    } else if (adcChannel == 3){
        L4 = (ADC-adcoffset[adcChannel])*adcmultiplier[adcChannel];
        adcChannel++;
        clear(ADCSRA,ADEN);     // ADC must be disabled when switching channels
        // Read from Pin F6 next
        clear(ADCSRB,MUX5);
        set(ADMUX,MUX2);
        set(ADMUX,MUX1);
        clear(ADMUX,MUX0);
        set(ADCSRA,ADIF);
        
        
    } else if (adcChannel == 4){
        R1 = (ADC-adcoffset[7])*adcmultiplier[7];
        adcChannel++;
        clear(ADCSRA,ADEN);     // ADC must be disabled when switching channels
        // Read from Pin F7 next
        clear(ADCSRB,MUX5);
        set(ADMUX,MUX2);
        set(ADMUX,MUX1);
        set(ADMUX,MUX0);
        set(ADCSRA,ADIF);
        
    } else if (adcChannel == 5){
        R2 = (ADC-adcoffset[6])*adcmultiplier[6];
        adcChannel++;
        clear(ADCSRA,ADEN);     // ADC must be disabled when switching channels
        // Read from Pin D4 next
        set(ADCSRB,MUX5);
        clear(ADMUX,MUX2);
        clear(ADMUX,MUX1);
        clear(ADMUX,MUX0);
        set(ADCSRA,ADIF);
        
    } else if (adcChannel == 6){
        R3 = (ADC-adcoffset[5])*adcmultiplier[5];
        adcChannel++;
        clear(ADCSRA,ADEN);     // ADC must be disabled when switching channels
        // Read from Pin D6 next
        set(ADCSRB,MUX5);
        clear(ADMUX,MUX2);
        clear(ADMUX,MUX1);
        set(ADMUX,MUX0);
        set(ADCSRA,ADIF);
        
    } else if (adcChannel == 7){
        R4 = (ADC-adcoffset[4])*adcmultiplier[4];
        adcChannel++;
        clear(ADCSRA,ADEN);     // ADC must be disabled when switching channels
        // Read from Pin D7 next
        set(ADCSRB,MUX5);
        clear(ADMUX,MUX2);
        set(ADMUX,MUX1);
        clear(ADMUX,MUX0);
        set(ADCSRA,ADIF);
    }
    else if (adcChannel == 8){
        breakBeam = ADC;
        adcChannel++;
        clear(ADCSRA,ADEN);     // ADC must be disabled when switching channels
        // Read from Pin F0 next
        clear(ADCSRB,MUX5);
        clear(ADMUX,MUX2);
        clear(ADMUX,MUX1);
        clear(ADMUX,MUX0);
        set(ADCSRA,ADIF);
        adcChannel = 0;
        dataFlag = 1;
    }
    
    if (dataFlag){
        ADCdata[0] = L1;
        ADCdata[1] = L2;
        ADCdata[2] = L3;
        ADCdata[3] = L4;
        ADCdata[4] = R4;
        ADCdata[5] = R3;
        ADCdata[6] = R2;
        ADCdata[7] = R1;
        ADCdata[8] = breakBeam;
    }
    dataFlag = 0;
    // to allow conversions again
    set(ADCSRA,ADEN);
    set(ADCSRA,ADSC);
    
}

void findPuck(void) {
    
    set(ADCSRA,ADSC); //start conversion
    int maxchannel = 0;
    float maxADC = 0;
    int z = 0;
    for (z = 0; z<8; z++) {
        if (ADCdata[z]>maxADC) {
            maxchannel = z;
            maxADC = ADCdata[z];
        }
    }
    if (breakBeam < 900) {
        State = GoToGoal;
        maxchannel = POSSESSPUCK;
        //        green_LED(ON);
    }
    
    // if both L1 and R1 are high, drive forward!
    if (maxchannel ==  0 || maxchannel == 7) {
        if (abs(ADCdata[0] - ADCdata[7]) < 100) {
            maxchannel = FORWARD;
        }
    }
    
    if (maxADC<100)
    {maxchannel = 8;}
    
    switch (maxchannel) {
        case 0:
            puckdirr= plow;
            puckdirl = -plow;
            lastPin = PIN0;
            //pindirection = 0;
            break;
        case 1:
            puckdirr = plow;
            puckdirl= -plow;
            lastPin = PIN1;
            //pindirection = 1;
            break;
        case 2:
            puckdirr = plow;
            puckdirl= -plow;
            lastPin = PIN2;
            //pindirection = 2;
            break;
        case 3:
            puckdirr = plow;
            puckdirl= -plow;
            lastPin = PIN3;
            //pindirection = 3;
            break;
        case 4:
            puckdirr = plow;
            puckdirl= -plow;
            lastPin = PIN4;
            //pindirection = 4;
            break;
        case 5:
            puckdirr = -plow;
            puckdirl= plow;
            lastPin = PIN5;
            //pindirection = 5;
            break;
        case 6:
            puckdirr = -plow;
            puckdirl= plow;
            lastPin = PIN6;
            //pindirection = 6;
            break;
        case 7:
            puckdirr = -plow;
            puckdirl= plow;
            lastPin = PIN7;
            //pindirection = 7;
            break;
            
        case FORWARD:
            puckdirr = plow;
            puckdirl= plow;
            break;
            
        case 9:
            puckdirr = plow;
            puckdirl= -plow;
            //pindirection = 8;
            break;
        case POSSESSPUCK:
            //  if you've got the puck, get out of this function and go score a fucking goal!!!
            break;
        default:
            break;
            
    }
}

void goScore(void) {
    
    if (y_robot_position_fil<=25 && y_robot_position_fil>0) {
        red_LED(ON);
        blue_LED(OFF);
        if(robot_orientation_fil>PI && robot_orientation_fil<=(3*PI/2)){
            GoalState = 1;
            rightcommand = low;
            leftcommand = high;
        }
        if(robot_orientation_fil>(3*PI/2) && robot_orientation_fil<=(2*PI)){
            GoalState = 1;
            rightcommand = high;
            leftcommand = low;
        }
        if(robot_orientation_fil<=(PI/2-t) && robot_orientation_fil>0){
            GoalState = 2;
            rightcommand = high;
            leftcommand = med;
        }
        
        if(robot_orientation_fil>(PI/2+t) && robot_orientation_fil<=PI){
            GoalState = 3;
            rightcommand = low;
            leftcommand = high;
        }
        if(robot_orientation_fil<=(PI/2+t) && robot_orientation_fil>(PI/2-t)){
            rightcommand = high;
            leftcommand = high;
            GoalState = 15;
        }
    }
    
    if (y_robot_position_fil<=0 && y_robot_position_fil>-25) {
        red_LED(OFF);
        blue_LED(OFF);
        if(robot_orientation_fil>(3*PI/2) && robot_orientation_fil<=(2*PI)){
            rightcommand = high;
            leftcommand = low;
            GoalState = 4;
        }
        if(robot_orientation_fil>(PI) && robot_orientation_fil<=(3*PI/2)){
            rightcommand = high;
            leftcommand = low;
            GoalState = 4;
        }
        if(robot_orientation_fil<=(PI/2-t) && robot_orientation_fil>0){
            rightcommand = high;
            leftcommand = low;
            GoalState = 5;
        }
        
        if(robot_orientation_fil>(PI/2+t) && robot_orientation_fil<=PI){
            rightcommand = low;
            leftcommand = high;
            
            GoalState = 6;
        }
        if(robot_orientation_fil<=(PI/2+t) && robot_orientation_fil>(PI/2-t)){
            rightcommand = high;
            leftcommand = high;
            GoalState = 15;
        }
    }
    
    if (y_robot_position_fil<=-25) {
        red_LED(OFF);
        blue_LED(ON);
        if(robot_orientation_fil>PI && robot_orientation_fil<=(3*PI/2)){
            rightcommand = high;
            leftcommand = vlow;
            GoalState = 7;
        }
        if(robot_orientation_fil>(0) && robot_orientation_fil<=(PI/4)){
            rightcommand = high;
            leftcommand = med;
        }
        if(robot_orientation_fil<=(PI/2) && robot_orientation_fil>(PI/4)){
            rightcommand = med;
            leftcommand = high;
            GoalState = 8;
        }
        if(robot_orientation_fil>(3*PI/2) && robot_orientation_fil<=(2*PI)){
            rightcommand = high;
            leftcommand = low;
            GoalState = 9;
        }
        if(robot_orientation_fil>(PI/2) && robot_orientation_fil<=PI){
            rightcommand = vlow;
            leftcommand = high;
            GoalState = 10;
        }
    }
    
    
    if (y_robot_position_fil>25) {
        red_LED(OFF);
        blue_LED(ON);
        if(robot_orientation_fil>PI && robot_orientation_fil<=(3*PI/2)){
            rightcommand = low;
            leftcommand = high;
            GoalState = 11;
        }
        if(robot_orientation_fil>(3*PI/4) && robot_orientation_fil<=(PI)){
            rightcommand = med;
            leftcommand = high;
            GoalState = 11;
        }
        if(robot_orientation_fil>(PI/2) && robot_orientation_fil<=(3*PI/4)){
            rightcommand = high;
            leftcommand = med;
            GoalState = 11;
        }
        if(robot_orientation_fil<=(PI/2) && robot_orientation_fil>0){ //
            rightcommand = high;
            leftcommand = vlow;
            GoalState = 12;
        }
        if(robot_orientation_fil>(3*PI/2) && robot_orientation_fil<=(2*PI)){//
            rightcommand = vlow;
            leftcommand = high;
            GoalState = 13;
        }
        }
    
    right_motor(rightcommand);
    left_motor(leftcommand);
    
}

void stall(void) {
    if(State == PuckFind)
    {
        if(robot_orientation_fil<(PI/2) && robot_orientation_fil>0)
        {leftcommand = -40;
            rightcommand = 0;
        }
        if(robot_orientation_fil>(3*PI/2) && robot_orientation_fil<=2*PI){
            rightcommand = -40;
            leftcommand = 0;
        }
        if(robot_orientation_fil>(PI/2) && robot_orientation_fil<(PI))
        {rightcommand = -40;
            leftcommand = 0;
        }
        if(robot_orientation_fil>PI && robot_orientation_fil<(3*PI/2))
        {rightcommand = 0;
            leftcommand = -40;}
        right_motor(rightcommand);
        left_motor(leftcommand);
        m_wait(200);
    }
    
    if(State == GoToGoal && gtgstall==1){
        leftcommand = 100;
        rightcommand = 100;
        right_motor(rightcommand);
        left_motor(leftcommand);
        m_wait(400);
        gtgstall = 0;
    }
    
    if(State == GoToGoal && gtgstall ==0){
        leftcommand = -40;
        rightcommand = -40;
        right_motor(rightcommand);
        left_motor(leftcommand);
        m_wait(200);
        gtgstall = 1;
    }
    
}

void left_motor(int leftcommand){
    
    if(leftcommand>=0){
        set(PORTB,PIN2); //Set B2 to go forward
        clear(PORTB,PIN3);
        OCR4B = leftcommand*timer0count;}
    
    else{clear(PORTB,PIN2); //Set B2 to go forward
        set(PORTB,PIN3);
        OCR4B = -1*leftcommand*timer0count;}
}

void right_motor(int rightcommand){
    if(rightcommand>=0){
        OCR4A = rightcommand*timer0count;
        set(PORTB,PIN0); //Set B0 to go forward
        clear(PORTB,PIN1);}
    else{
        OCR4A = -1*rightcommand*timer0count;
        clear(PORTB,PIN0); //Set B0 to go forward
        set(PORTB,PIN1);}
}

void commHandler(void) {
    m_rf_read(buffer,PACKET_LENGTH_READ);
    char CommState = buffer[0];
    redblueswitch = 1;
    switch (CommState) {
        case COMM_TEST:
            red_LED(OFF);
            blue_LED(OFF);
            m_wait(100);
            blue_LED(ON);
            m_wait(100);
            blue_LED(OFF);
            redblueswitch = 1;
            break;
        case PLAY:
            State = PuckFind;
            blue_LED(ON);
            //            if(m_port_check(m_port_ADDRESS,PORTG,PIN2) && redblueswitch) {
            //                red_LED(ON);
            //                blue_LED(OFF);
            //                redblueswitch = 0;
            //            }
            //            if(!m_port_check(m_port_ADDRESS,PORTG,PIN2) && redblueswitch) {
            //                blue_LED(ON);
            //                red_LED(OFF);
            //                redblueswitch = 0;
            //            }
            break;
        case PAUSE:
            State = Listen;
            break;
        default:
            break;
    }
    commFlag = 0;
}

void avoid(void) { //call case if oriented at our goal, near our goal, and in puckfind or go to goal
    
    if(y_robot_position_fil>0){
    
//    if(robot_orientation_fil<(3*PI/2) && robot_orientation_fil>(PI+t) && y_robot_position_fil){
//        rightcommand = 0;
//        leftcommand = low;}
//    if(robot_orientation_fil>(PI-t) && robot_orientation<(PI-t)){
//        rightcommand = 0;
//        leftcommand = 0;
//    }
    }
    
}

void red_LED(bool status) {
    if (status) {
        set(PORTB, PIN4);
    } else {
        clear(PORTB, PIN4);
    }
}

void blue_LED(bool status) {
    if (status) {
        set(PORTC, PIN6);
    } else {
        clear(PORTC, PIN6);
    }
}

void white_LED(bool status) {
    if (status) {
        m_port_set(m_port_ADDRESS,PORTG,PIN3);
    } else {
        m_port_clear(m_port_ADDRESS,PORTG,PIN3);
    }
}

void green_LED(bool status) {
    if (status) {
        m_port_set(m_port_ADDRESS,PORTG,PIN6);
    } else {
        m_port_clear(m_port_ADDRESS,PORTG,PIN6);
    }
}
void yellow_LED(bool status) {
    if (status) {
        m_port_set(m_port_ADDRESS,PORTG,PIN0);
    } else {
        m_port_clear(m_port_ADDRESS,PORTG,PIN0);
    }
}



////
ISR(TIMER1_OVF_vect) {
    // when timer oveflows (set for 10Hz), process and transmit data
    send_flag = 1;
    rf_counter++;
}


// interupt when comm is recieved
ISR(INT2_vect){
    commFlag = 1;
    //red_LED(ON);
    //Will determine values of oppgoal and opptarget
}


//ADC interrupt
ISR(ADC_vect){
    //ADC Order = L1, L2, L3, L4, R4, R3, R2 ,R1
    adcflag = 1;
}

//// Useful Blocks of code
//
//                send_data[0] = (char)(star_data[0]/10);//RX_ADDRESS;
//                send_data[1] = (char)(star_data[1]/10);//RX_ADDRESS;;
//                send_data[2] = (char)(star_data[3]/10);//RX_ADDRESS;
//                send_data[3] = (char)(star_data[4]/10);//RX_ADDRESS;
//                send_data[4] = (char)(star_data[6]/10);//RX_ADDRESS;
//                send_data[5] = (char)(star_data[7]/10);//RX_ADDRESS;
//                send_data[6] = (char)(star_data[9]/10);//RX_ADDRESS;
//                send_data[7] = (char)(star_data[10]/10);//RX_ADDRESS;
//
//                m_rf_send(TX_ADDRESS, send_data, PACKET_LENGTH_SEND);

//                send_data[0] = 0;//RX_ADDRESS;
//                send_data[1] = 1;//(char)robot_position[0];
//                send_data[2] = 2;//(char)robot_position[1];
//                m_rf_send(TX_ADDRESS, send_data, PACKET_LENGTH);

//                if(m_usb_isconnected()) {
//                    m_usb_tx_int((int)star_data[0]);//Raw x and y values of the stars in order of receipt
//                    m_usb_tx_string("\t");
//                    m_usb_tx_int((int)star_data[1]);
//                    m_usb_tx_string("\t");
//                    m_usb_tx_int((int)star_data[3]);
//                    m_usb_tx_string("\t");
//                    m_usb_tx_int((int)star_data[4]);
//                    m_usb_tx_string("\t");
//                    m_usb_tx_int((int)star_data[6]);
//                    m_usb_tx_string("\t");
//                    m_usb_tx_int((int)star_data[7]);
//                    m_usb_tx_string("\t");
//                    m_usb_tx_int((int)star_data[9]);
//                    m_usb_tx_string("\t");
//                    m_usb_tx_int((int)star_data[10]);
//                    m_usb_tx_string("\t");
//                    m_usb_tx_int((int)(robot_position[0])); //X position, whatever that means
//                    m_usb_tx_string("\t");
//                    m_usb_tx_int((int)(robot_position[1])); //Y position
//                    m_usb_tx_string("\t");
//                    m_usb_tx_int((int)(robot_orientation*127/6.3)); //Orientation converted from radians to a fraction of 127
//                    m_usb_tx_string("\n");
//
//                }
//
//
//    if (L1 > 512){
//        if (L1 > L2 && L1 > R1) {
//            m_port_clear(m_port_ADDRESS,PORTG,lastPin);
//            //   l_motor(20);
//            m_port_set(m_port_ADDRESS,PORTG,PIN0);
//            lastPin = PIN0;
//            pindirection = 0;
//        }
//    }
//    if (L2 > 512){
//        if (L2 > L3 && L2 > R1) {
//            m_port_clear(m_port_ADDRESS,PORTG,lastPin);
//            //   l_motor(40);
//            m_port_set(m_port_ADDRESS,PORTG,PIN1);
//            lastPin = PIN1;
//            pindirection = 1;
//        }
//    }
//    if (L3 > 512){
//        if (L3 > L4 && L3 > L2) {
//            m_port_clear(m_port_ADDRESS,PORTG,lastPin);
//            //   l_motor(60);
//            m_port_set(m_port_ADDRESS,PORTG,PIN2);
//            lastPin = PIN2;
//            pindirection = 2;
//        }
//    }
//    if (L4 > 512){
//        if (L4 > R4 && L4 > L3) {
//            m_port_clear(m_port_ADDRESS,PORTG,lastPin);
//            //  l_motor(80);
//            m_port_set(m_port_ADDRESS,PORTG,PIN3);
//            lastPin = PIN3;
//            pindirection = 3;
//        }
//    }
//    if (R4 > 512){
//        if (R4 > R3 && R4 > L4) {
//            m_port_clear(m_port_ADDRESS,PORTG,lastPin);
//            //  r_motor(80);
//            m_port_set(m_port_ADDRESS,PORTG,PIN4);
//            lastPin = PIN4;
//            pindirection = 4;
//        }
//    }
//    if (R3 > 512){
//        if (R3 > R2 && R3 > R4) {
//            m_port_clear(m_port_ADDRESS,PORTG,lastPin);
//            // r_motor(60);
//            m_port_set(m_port_ADDRESS,PORTG,PIN5);
//            lastPin = PIN5;
//            pindirection = 5;
//        }
//    }
//    if (R2 > 512){
//        if (R2 > R1 && R2 > R3) {
//            m_port_clear(m_port_ADDRESS,PORTG,lastPin);
//            // r_motor(40);
//            m_port_set(m_port_ADDRESS,PORTG,PIN6);
//            lastPin = PIN6;
//            pindirection = 6;
//        }
//    }
//    if (R1 > 512){
//        if (R1 > L1 && R1 > R2) {
//            m_port_clear(m_port_ADDRESS,PORTG,lastPin);
//            //l_motor(60);
//            m_port_set(m_port_ADDRESS,PORTG,PIN7);
//            lastPin = PIN7;
//            pindirection = 7;
//        }
//    }



//
//m_usb_tx_string("L1 = ");
//m_usb_tx_int(L1);
//m_usb_tx_string("  ");
//m_usb_tx_string(" L2= ");
//m_usb_tx_int(L2);
//m_usb_tx_string("  L3 = ");
//m_usb_tx_int(L3);
//m_usb_tx_string("  ");
//m_usb_tx_string(" L4 = ");
//m_usb_tx_int(L4);
//m_usb_tx_string("  ");
//m_usb_tx_string(" R1 = ");
//m_usb_tx_int(R1);
//m_usb_tx_string("  ");
//m_usb_tx_string(" R2 = ");
//m_usb_tx_int(R2);
//m_usb_tx_string("  ");
//m_usb_tx_string(" R3 = ");
//m_usb_tx_int(R3);
//m_usb_tx_string("  ");
//m_usb_tx_string(" R4 = ");
//m_usb_tx_int(R4);
//m_usb_tx_string(" BB = ");
//m_usb_tx_int(breakBeam);
//m_usb_tx_string("\n");

//if ()
//{
//    red_LED(OFF);
//    blue_LED(OFF);
//    if (robot_orientation_filtered>(0.3+opptarget)) { //right turn
//        leftcommand = 40;
//        rightcommand = 20;
//        left_motor(leftcommand);
//        right_motor(rightcommand);
//    }
//
//    if (robot_orientation_filtered<(-0.3+opptarget)) { //left turn
//        leftcommand = 20;
//        rightcommand = 40;
//        left_motor(leftcommand);
//        right_motor(rightcommand);
//    }
//
//
//    if ((-0.3+opptarget)<robot_orientation_filtered && robot_orientation_filtered<(0.3+opptarget)) {
//        //forward
//        leftcommand = 30;
//        rightcommand = 30;
//        left_motor(leftcommand);
//        right_motor(rightcommand);
//    }
//}
//if (robot_position[1]>35) {
//    red_LED(OFF);
//    blue_LED(ON);
//    //white_LED(OFF);
//    if (robot_orientation_filtered>(0.3+opptarget)) { //right turn
//        leftcommand = 20;
//        rightcommand = -20;
//        left_motor(leftcommand);
//        right_motor(rightcommand);}
//
//    if (robot_orientation_filtered<(-0.3+opptarget)) { //left turn
//        leftcommand = -20;
//        rightcommand = 20;
//        left_motor(leftcommand);
//        right_motor(rightcommand);}
//
//    if ((-0.3+opptarget)<robot_orientation_filtered && robot_orientation_filtered<(opptarget)) {
//        //forward
//        leftcommand = 30;
//        rightcommand = 40;
//        left_motor(leftcommand);
//        right_motor(rightcommand);}
//    if (robot_orientation_filtered<(0.3+opptarget) && robot_orientation_filtered>(opptarget)) {
//        leftcommand = 40;
//        rightcommand = 30;
//        left_motor(leftcommand);
//        right_motor(rightcommand);
//    }
//
//}
//if (robot_position[1]<-35) {
//    //                    white_LED(ON);
//    red_LED(OFF);
//    blue_LED(ON);
//    if (robot_orientation_filtered>(0.3+opptarget)) { //right turn
//        leftcommand = 20;
//        rightcommand = -20;
//        left_motor(leftcommand);
//        right_motor(rightcommand);}
//
//    if (robot_orientation_filtered<(-0.3+opptarget)) { //left turn
//        leftcommand = -20;
//        rightcommand = 20;
//        left_motor(leftcommand);
//        right_motor(rightcommand);}
//
//    if ((-0.3+opptarget)<robot_orientation_filtered && robot_orientation_filtered<(opptarget)) {
//        //forward
//        leftcommand = 40;
//        rightcommand = 30;
//        left_motor(leftcommand);
//        right_motor(rightcommand);}
//    if (robot_orientation_filtered<(0.3+opptarget) && robot_orientation_filtered>(opptarget)) {
//        leftcommand = 30;
//        rightcommand = 40;
//        left_motor(leftcommand);
//        right_motor(rightcommand);
//    }
//
//}

//case Qualify:
//    white_LED(OFF);
//    if (checkside == 0) {
//        checkside = 1;
//        if (robot_position[0] < 0){
//            target = 3*PI/2;
//            postarget = 100;
//        }
//        else {
//            target = PI/2;
//            postarget = -100;
//        }
//        m_wait(100);
//    }
//
//    if (abs(robot_position[0])<(abs(postarget))) {
//
//        if (robot_orientation_filtered>(0.3+target)) { //right spin
//            leftcommand = 20;
//            rightcommand = -20;
//            left_motor(leftcommand);
//            right_motor(rightcommand);
//        }
//
//
//        if (robot_orientation_filtered<(-0.3+target)) { //left spin
//            leftcommand = -20;
//            rightcommand = 20;
//            left_motor(leftcommand);
//            right_motor(rightcommand);
//        }
//
//
//        if ((-0.3+target)<robot_orientation_filtered && robot_orientation<(0.3+target)) {
//            //forward
//            leftcommand = 20;
//            rightcommand = 20;
//            left_motor(leftcommand);
//            right_motor(rightcommand);
//        }
//
//
//    } else {
//        State=Qualify;
//    }
//    break;

//m_usb_tx_string("L1 = ");
//m_usb_tx_int(L1);
//m_usb_tx_string("  ");
//m_usb_tx_string(" L2= ");
//m_usb_tx_int(L2);
//m_usb_tx_string("  L3 = ");
//m_usb_tx_int(L3);
//m_usb_tx_string("  ");
//m_usb_tx_string(" L4 = ");
//m_usb_tx_int(L4);
//m_usb_tx_string("  ");
//m_usb_tx_string(" R1 = ");
//m_usb_tx_int(R1);
//m_usb_tx_string("  ");
//m_usb_tx_string(" R2 = ");
//m_usb_tx_int(R2);
//m_usb_tx_string("  ");
//m_usb_tx_string(" R3 = ");
//m_usb_tx_int(R3);
//m_usb_tx_string("  ");
//m_usb_tx_string(" R4 = ");
//m_usb_tx_int(R4);
//m_usb_tx_string("\n");