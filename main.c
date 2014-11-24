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

#define RX_ADDRESS 0x18 //Receipt address This isnt right but is working because im not receiving anything
#define TX_ADDRESS 0x18 //Send address
#define PACKET_LENGTH_SEND 3
#define CHANNEL 2
#define PACKET_LENGTH_READ 10

// STATES: Currently operating in only state 1 which has been repurposed to localisation and communication with the other M2. State 2 is motor driving depending on initial location and orientation. Change the right State=1 commands to State=2 commands to re-enable the qualifying code. Note: I have the states switch with receipt of a play command
//LOCALISATION SENDING: I changed the send_data array to send the star positions for debugging, but change the packet lengths back and uncomment the send data stuff when we figure that out. Timer1 was not sending an interrupt so we commented it out, but ideally we will use it's 10hz overflow interrupt to find and send position.
//MOTOR CONTROL: The motor commands are set by valuing a signed int "leftcommand" or "rightcommand" to duty cycle in percent with positive being forward and negative being backward. I havent been able to test with the h-bridge yet obviously, so we will have to make sure that we are setting and clearing the right pins for direction. Right and left motors could be switched too, depending on how we plug in the molex.


char buffer[PACKET_LENGTH_READ] = {0,0,0,0,0,0,0,0,0,0}; //data to be received
char send_data[PACKET_LENGTH_SEND] = {0,0,0};//,0,0,0,0,0,0,0,0}; // data to be sent to game controller


unsigned int star_data[12] = {0,0,0,0,0,0,0,0,0,0,0,0};
unsigned int star_data2[12] = {0,0,0,0,0,0,0,0,0};
float robot_position[2] = {0,0}; // vector for robot position (x and y)
float robot_orientation= 0; // vector for robot orientation (direction fo y-axis)
volatile bool timer1_flag = 0; // set high when timer1 overflows
float pixel_cm_conversion = 40.0/125.0; // conversion from pixels to cm, TBD
bool valid = 0; // true when position m_wii data is valid
//int dutyBLeft = 0; //Percentage of duty cycle left motor
//int dutyARight = 0; //Percentage of duty cycle right motor
float target = 0;  //Target angle for driving across rink
int timer0count = 2; //0xff/100
int leftcommand = 0; //Duty cycle and direction of left motor
int rightcommand = 0; //Duty cycle and direction of right motor
int State = 2;
float postarget = 0;
int sign = 0;
int checkside = 0;
int testvar = 0;
//float x = 0;
//float y = 0;
//float theta = 0;
//int skip = 0;
//int LocState =0;
//float max = 0;
//int k = 0;
//float Ax = 0;
//float Ay = 0;
//float Cx = 0;
//float Cy = 0;
//float ox = 0;
//float oy = 0;



void init(void);
bool find_position(unsigned int star_data[]);
float dot(float v1[], float v2[]);
//void motors_forward(dutyBLeft);
//void right_spin(dutyBLeft);
//void left_spin(dutyARight);
void left_motor(int leftcommand);
void right_motor(int rightcommand);



int main(void)
{
    init();
    while(TRUE) {
        
        switch (State) { //** Necessary states for 11/24: 1 = Wait |  2 = drive to opposite side of rink
            case 1: //Wait for PLAY command
                
                m_wait(300);
                State = 2;
                break;
                
            case 2:
                
//                rightcommand = -50;
//                leftcommand = -50;
//                right_motor(rightcommand);
//                left_motor(leftcommand);
                
                m_wait(150);
                
                if (checkside<10) {
                    checkside++;
                
                    if (robot_position[0]<0){
                        target = 0;
                        postarget = 100;
                    }
                    
                    else {target = 3.14;
                        postarget = -100;
                    }
                }
                
                m_wait(100);
                
                if (robot_position[0]<(postarget)) {
                    
                    if (robot_orientation>(4+target)) { //right spin
                        leftcommand = 50;
                        rightcommand = -50;
                        left_motor(leftcommand);
                        right_motor(rightcommand);
                        m_green(ON);
                        
                    }
                    m_wait(50);
                    
                    
                    if (robot_orientation<(-4+target)) { //left spin
                        leftcommand = -50;
                        rightcommand = 50;
                        left_motor(leftcommand);
                        right_motor(rightcommand);
                        m_green(OFF);
                    }
                    m_wait(50);
                    
                    if ((-4+target)<robot_orientation & robot_orientation<(4+target)) {
                        //forward
                        leftcommand = 50;
                        rightcommand = 50;
                        left_motor(leftcommand);
                        right_motor(rightcommand);
                    }
                    m_wait(50);

                } else {
                    State=1;
                }
                break;
                
        
        
    default:
        State = 1;
        
        break;
        
        }
        }
}
    

void left_motor(int leftcommand){
    
    if(leftcommand>0){
        set(PORTB,PIN0); //Set B0 to go forward
        clear(PORTB,PIN1);
        OCR4B = leftcommand*timer0count;
    }
    
    else{clear(PORTB,PIN0); //Set B0 to go forward
        set(PORTB,PIN1);
        OCR4B = -1*leftcommand*timer0count;}
}

void right_motor(int rightcommand){
    if(rightcommand>0){
        OCR4A = rightcommand*timer0count;
        set(PORTB,PIN2); //Set B2 to go forward
        clear(PORTB,PIN3);}
    else{
        OCR4A = -1*rightcommand*timer0count;
        clear(PORTB,PIN2); //Set B2 to go forward
        set(PORTB,PIN3);}
}


void init(void) {
    m_clockdivide(0); // 16 MHz clock
    m_disableJTAG();
    
    m_bus_init();
    m_usb_init();
    
    //Set pins for directional motor output
    set(DDRB,PIN0); //Left Motor set forward
    set(DDRB,PIN1); //Left Motor clear forward
    set(DDRB,PIN2); //Right Motor set forward
    set(DDRB,PIN3); //Right Motor clear forward
    
    // open and initialize rf communications
    m_rf_open(CHANNEL, RX_ADDRESS, PACKET_LENGTH_SEND);
    
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
    
    // Timer 0 for PWM
    
    OCR4A = 0; // initialize duty cycle to zero
    OCR4B = 0;
    OCR4C = 200;
    
    set(DDRC,PIN7); //Compare A pin
    set(DDRB,PIN6); //Compare B pin
    
    
    set(TCCR4B, CS43); //  set prescaler to /128
    clear(TCCR4B, CS42); // ^
    set(TCCR4B, CS41); // ^
    clear(TCCR4B, CS40); // ^
    
    clear(TCCR4D, WGM41); // Up to OCR4C, PWM mode
    clear(TCCR4D, WGM40); // ^
    
    //Output compare A for PWM of Left Motor ** PIN C6
    set(TCCR4A, PWM4A); // clear at OCR4A, set at 0xFF
    set(TCCR4A, COM4A1); // ^
    clear(TCCR4A, COM4A0); // ^
    
    //Output compare B for PWM of Right Motor  ** PIN B6
    set(TCCR4A, PWM4B); // clear at OCR4B, set at 0xFF
    set(TCCR4A, COM4B1); // ^
    clear(TCCR4A, COM4B0); // ^
    
}

////
ISR(TIMER1_OVF_vect) {
    m_wii_read(star_data);
    valid = find_position(star_data);
    if (valid) {
        send_data[0] = (char)robot_position[0];//TX_ADDRESS;
        send_data[1] = (char)robot_position[1];
        send_data[2] = (char)(robot_orientation*127/6.3);
//        send_data[0] = (char)(star_data[0]/10);//RX_ADDRESS;
//        send_data[1] = (char)(star_data[1]/10);//RX_ADDRESS;;
//        send_data[2] = (char)(star_data[3]/10);//RX_ADDRESS;
//        send_data[3] = (char)(star_data[4]/10);//RX_ADDRESS;
//        send_data[4] = (char)(star_data[6]/10);//RX_ADDRESS;
//        send_data[5] = (char)(star_data[7]/10);//RX_ADDRESS;
//        send_data[6] = (char)(star_data[9]/10);//RX_ADDRESS;
//        send_data[7] = (char)(star_data[10]/10);//RX_ADDRESS;
        
        m_rf_send(TX_ADDRESS, send_data, PACKET_LENGTH_SEND);
    }
    // when timer oveflows (set for 10Hz), process and transmit data
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
    
    if (LocIndex<2) {
        LocState = 1;}
    if (LocIndex=2) {
        LocState =2;}
   if (LocIndex>2) {
       LocState = 3;}
    
//    if (LocIndex=2) {
//        LocState = 2;}
//    
//    if (LocIndex>=4) {
//        LocState = 3;}

    
    
    switch (LocState) {
        case 1: //ALL STARS FOUND
            m_red(OFF);
            m_green(ON);
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

            // orientation measured relative to rink coordinate frame in radians, counter-clockwise
            float orientation = theta; // no longer inverting
            
            // store values to arrays (defined above so the can be accessed by main, C functions can't return an array)
            robot_position[0] = pixel_cm_conversion*x; // convert from pixels to cm
            robot_position[1] = pixel_cm_conversion*y; // convert from pixels to cm
            robot_orientation = orientation;
            
            break;
            
        case 2: //3-star case. If star_data[i] = 1023, throw out data point, make new array, assume axial stars. Will later perform ratio calculations on the other three to determine axial stars, rotate that axis.
            
            m_red(ON);
            m_green(ON);
            
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

//ISR(INT2_vect){
//    m_rf_read(buffer,PACKET_LENGTH_READ);
//    m_red(TOGGLE);
//    if (buffer[0] = 0xA1) { //If receive play command send to state 1
//        State = 2;}
//    else {State = 1;}  //If received command other than play, continue to wait.
//}
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

