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

#define RX_ADDRESS 24
#define TX_ADDRESS 0xDA
#define PACKET_LENGTH 3
#define CHANNEL 1


unsigned int star_data[11] = {0,0,0,0,0,0,0,0,0,0,0};
float robot_position[2] = {0,0}; // vector for robot position (x and y)
float robot_orientation[2] = {0,0}; // vector for robot orientation (direction fo y-axis)
char send_data[PACKET_LENGTH] = {0,0}; // data to be sent to game controller
volatile bool timer1_flag = 0; // set high when timer1 overflows
float pixel_cm_conversion = 10; // conversion from pixels to cm, TBD
bool valid = 0; // true when position m_wii data is valid


void init(void);
bool find_position(unsigned int data[]);
float dot(float v1[], float v2[]);

int main(void)
{
    while(TRUE) {
        // when timer oveflows (set for 10Hz), process and transmit data
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
        }
    }
}

void init(void) {
    m_clockdivide(0); // 16 MHz clock
    m_disableJTAG();
    
    m_bus_init();
    
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
    
}


ISR(TIMER1_OVF_vect) {
    timer1_flag = 1;
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

    // calculate star axes (x axis may be flipped, not sure)
    float y_axis[2] = {(Ax-ox)/(sqrt(powf((Ax-ox),2) + powf((Ay-oy),2))), (Ay-oy)/(sqrt(powf((Ax-ox),2) + powf((Ay-oy),2)))};
    float x_axis[2] = {-(Ay-oy)/(sqrt(powf((Ax-ox),2) + powf((Ay-oy),2))), (Ax-ox)/(sqrt(powf((Ax-ox),2) + powf((Ay-oy),2)))};

    // calculate position vectors from star origin to robot
    float rx = 512 - ox;
    float ry = 512 - oy;
    float r_vect[2] = {rx,ry};
    
    // calculate x and y position
    float x = dot(r_vect, x_axis);
    float y = dot(r_vect, y_axis);
    
    // calculate robot orientation
    float x_orient = dot(x_axis, robot_y_axis);
    float y_orient = dot(y_axis, robot_y_axis);
    
    // store values to arrays (defined above so the can be accessed by main, C functions can't return an array)
    robot_position[0] = pixel_cm_conversion*x; // convert from pixels to cm
    robot_position[1] = pixel_cm_conversion*y; // convert from pixels to cm
    robot_orientation[0] = x_orient;
    robot_orientation[1] = y_orient;
    
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
