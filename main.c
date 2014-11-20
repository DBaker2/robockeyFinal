/* Name: main.c
 * Author: Ben Weinreb
 * Copyright: A05 Localization
 * License: November 2014
 */

#include "m_general.h"
#include "m_bus.h"
#include "m_wii.h"
#include "m_rf.h"
#include "m_usb.h"

#define RX_ADDRESS 0x24
#define TX_ADDRESS 0xDA
#define PACKET_LENGTH 3
#define CHANNEL 1
#define PACKET_LENGTH_READ 10

char buffer[PACKET_LENGTH_READ] = {0,0,0,0,0,0,0,0,0,0}; //data to be received
char send_data[PACKET_LENGTH] = {0,0,0}; // data to be sent to game controller


unsigned int star_data[12] = {0,0,0,0,0,0,0,0,0,0,0,0};
float robot_position[2] = {0,0}; // vector for robot position (x and y)
float robot_orientation= 0; // vector for robot orientation (direction fo y-axis)
volatile bool timer1_flag = 0; // set high when timer1 overflows
float pixel_cm_conversion = 10; // conversion from pixels to cm, TBD
bool valid = 0; // true when position m_wii data is valid
//int dutyBLeft = 0; //Percentage of duty cycle left motor
//int dutyARight = 0; //Percentage of duty cycle right motor
int target = 0;  //Target angle for driving across rink
int timer0count = 327; //0xff/100
int leftcommand = [0 0]; //Duty cycle and direction of left motor
int rightcommand = [0 0]; //Duty cycle and direction of right motor
int State = 1;
int postarget = 0;
int sign = 0;

void init(void);
bool find_position(unsigned int data[]);
float dot(float v1[], float v2[]);
//void motors_forward(dutyBLeft);
//void right_spin(dutyBLeft);
//void left_spin(dutyARight);
void left_motor(leftcommand);
void right_motor(rightcommand);



int main(void)
{
    while(TRUE) {
        
        switch (State) { //** Necessary states for 11/24: 1 = Wait |  2 = drive to opposite side of rink
            case 1:
                OCR0A = 0;
                OCR0B = 0;
                
            case 2:
                if (checkside=0) {
                    checkside = 1
                    if (position[0]<0){
                        target = 0;
                        postarget = 100;
                        sign = 1;
                    }
                    
                    else {target = 180;
                    postarget = -100
                    sign = -1
                    }
                }
                while (position[0]<(postarget) {

                if (orientation>(4+target)) { //right spin
                    leftcommand = [50 1];
                    rightcommand = [50 0];
                    leftmotor(leftcommand);
                    rightmotor(rightcommand);
                    m_green(on);
                }
                if (orientation<(-4+target)) { //left spin
                    leftcommand = [50 0];
                    rightcommand = [50 1];
                    leftmotor(leftcommand);
                    rightmotor(rightcommand);
                    m_green(off);
                }
                if ((-4+target)<orientation<(4+target)) { //forward
                    leftcommand = [50 1];
                    rightcommand = [50 1];
                    leftmotor(leftcommand);
                    rightmotor(rightcommand);
                }
                    
                }
            default:
                state = 1;

                break;
        }
    }
}

void left_motor(leftcommand){
    OCR0B = leftcommand[0]*timer0count
    if(leftcommand[1] = 1)
        set(PORTB,B0); //Set B0 to go forward
        clear(PORTB,B1);
    
    else{clear(PORTB,B0); //Set B0 to go forward
        set(PORTB,B1);}
    
}

void right_motor(rightcommand){
    OCR0A = rightcommand[0]*timer0count
    if(rightcommand[1] = 1)
        set(PORTB,B2); //Set B2 to go forward
        clear(PORTB,B3);
    else{clear(PORTB,B2); //Set B2 to go forward
        set(PORTB,B3);}
}

                {//void motors_forward(dutyBLeft){
//    OCR0A = dutyBLeft*timer0count;
//    OCR0B = dutyBLeft*timer0count;
//    set(PORTB,B0); //Set B0 to go forward
//    clear(PORTB,B1);
//    set(PORTB,B2); //Set B2 to go forward
//    clear(PORTB,B3);
//}


//void right_spin(){
//    


//void left_spin(dutyBLeft){
//    
//    set(PORTB,B0); //Set B0 to go forward
//    clear(PORTB,B1);
//    clear(PORTB,B2); //Set B2 to go forward
//    set(PORTB,B3);
//    m_green(on);
//}
                }


void init(void) {
    m_clockdivide(0); // 16 MHz clock
    m_disableJTAG();
    
    m_bus_init();
    
    //Set pins for directional motor output
    set(DDRB,B0); //Left Motor set forward
    set(DDRB,B1); //Left Motor clear forward
    set(DDRB,B2); //Right Motor set forward
    set(DDRB,B3); //Right Motor clear forward
    
    // open and initialize rf communications
    m_rf_open(CHANNEL, RX_ADDRESS, PACKET_LENGTH);
    
    // open and initialize m_wii communication
    m_wii_open();
    
    // timer 1 set up (running at approx 10Hz)
    clear(TCCR1B, WGM13); // (mode 4) UP to OCR1A
    set(TCCR1B, WGM12);   // ^
    clear(TCCR1A, WGM11); // ^
    clear(TCCR1A, WGM10); // ^
    
    set(TIMSK1, TOIE1); // interrupt every overflow
    
    OCR1A = 25000; // timer up to this number
    
    clear(TCCR1B, CS12); // prescaler to /64
    set(TCCR1B, CS11);   // ^
    set(TCCR1B, CS10);  // ^
    
    sei();
    
    // Timer 0 for PWM
    
    OCR3A = 0; // initialize duty cycle to zero
    ICR3 = 32760;
    
    set(DDRB,B7); //Compare A pin
    set(DDRD,D0); //Compare B pin
    
    clear(TCCR0B, CS02); //  set prescaler to /8
    set(TCCR0B, CS01); // ^
    clear(TCCR0B, CS00); // ^
    
    clear(TCCR0B, WGM02); // Up to 0xff, PWM mode
    set(TCCR0A, WGM01); // ^
    set(TCCR0A, WGM00); // ^
    
    //Output compare A for PWM of Left Motor
    set(TCCR0A, COM0A1); // clear at OCR3A, set at 0xFF
    clear(TCCR0A, COM0A0); // ^
    
    //Output compare B for PWM of Left Motor
    set(TCCR0A,COM0B1); //Clear at OCR3B, set at 0xFF
    clear(TCCR0A, COM0B1);
    
}


ISR(TIMER1_OVF_vect) {
    timer1_flag = 1;
    if (timer1_flag) {
        m_wii_read(star_data);
        valid = find_position(star_data); // process data to find position
        // transmit data if valid (all stars were found)
        if (valid) {
            send_data[0] = RX_ADDRESS;
            send_data[1] = (char)robot_position[0];
            send_data[2] = (char)robot_position[1];
            m_rf_send(TX_ADDRESS, send_data, PACKET_LENGTH);
        }
        timer1_flag = 0;
        // when timer oveflows (set for 10Hz), process and transmit data
    }
}


// identify stars and find position and orientation. store to robot_position & robot_orientation
// returns true if all stars found, and false otherwise
                   
bool find_position(unsigned int data[]) {
    float dist[4][4] = {{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0}}; // array for dsitances between stars
    float robot_y_axis[2]= {0,1}; // robot reference frame y-axis
    float B_ratio = 2; // ratio of AB to CB
    float D_ratio = 1.25; // ratio of AD to CD
    float tol = .1; // tolerance to match ratio
    int a_guess = 0; // first guess for a
    int c_guess = 0; // first guess for c
    int A = 0; // index for A (x at 3*A, y at ((3*A)+1))
    int B = 0; // index for B ^
    int C = 0; // index for C ^
    int D = 0; // index for D ^
    
    // check if all stars are found, if so proceed, if not, return false
    int i = 0;
    for (i = 0; i < 11; i++) {
        if (data[i] == 1023) {
            return FALSE;
        }
    }
    
    
    // calculate all distances and store them in array, find maximum of these distances
    int j = 0;
    float max = 0;
    for (i = 0; i < 4; i++) {
        for (j = 0; j < 4; j++) {
            dist[i][j] = sqrt(powf((data[3*i] - data[3*j]),2) + powf((data[(3*i)+1] - data[(3*j)+1]),2));
            if (dist[i][j] > max) {
                max = dist[i][j];
                a_guess = i;
                c_guess = j;
            }
        }
    }
    
    // Identify A and C (using fact the B and D are both father from C than A)
    int k = 0;
    for (k = 0; k < 4; k++) {
        // only examine distances not between A and C
        if ((k != a_guess) && (k != c_guess)) {
            if (dist[A][k] < dist[C][k]) {
                A = a_guess;
                C = c_guess;
            } else {
                A = c_guess;
                C = a_guess;;
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
    float Ax = data[3*A];
    float Ay = data[(3*A)+1];
    float Cx = data[3*C];
    float Cy = data[(3*C)+1];
    
    // calculate origin
    float ox = (Ax + Cx)/2;
    float oy = (Ay + Cy)/2;

	// calculate rotation from robot frame to star frame
	float theta = (-1)*atan2((Ax-Cx),(Ay-Cy))		// these arguments should be doubles, check here if there is a problem

	// calculate robot position using output from homogeneous transform matric
	float x = (-1)*cos(theta)*(ox-512)-sin(theta)*(oy-384);
	float y = sin(theta)*(ox-512)-cos(theta)*(oy-384);
	
	// orientation measured relative to rink coordinate frame in radians, counter-clockwise
	float orientation = (-1)*theta;
    
    // store values to arrays (defined above so the can be accessed by main, C functions can't return an array)
    robot_position[0] = pixel_cm_conversion*x; // convert from pixels to cm
    robot_position[1] = pixel_cm_conversion*y; // convert from pixels to cm
    robot_orientation = orientation;
    
    // data is valid, return true
    return TRUE;
}

ISR(INT2_vect){
    m_rf_read(buffer,PACKET_LENGTH);
    m_red(TOGGLE);
    if (buffer[0] = 0xA1) { //If receive play command send to state 1
        state = 2;}
    else {state = 1}  //If received command other than play, continue to wait.
}
//


// computes dot product of two vectors
float dot(float v1[2], float v2[2]) {
    int i = 0;
    float result = 0;
    for (i = 0; i < 2; i++) {
        result = result + (v1[i]*v2[i]);
    }
    
    return result;
}
////
