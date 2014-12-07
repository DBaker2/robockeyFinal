/* Name: main.c
 * Author: Ben Weinreb
 * Copyright: Wireless_debug
 * License: allows wirless debugging using usb and m_rf
 */

#include "m_general.h"
#include "m_usb.c"
#include "m_bus.c"
#include "m_rf.c"



#define RX_ADDRESS 0x18
#define TX_ADDRESS 0x19
#define CHANNEL 1
#define PACKET_LENGTH 10
#define PLAY 0xA1
#define COMM_TEST 0xA0
#define PAUSE 0xA4



char buffer[10] = {PAUSE, 0xA0, 0xA0, 0xA0, 0xA0, 0xA0, 0xA0, 0xA0, 0xA0, 0xA0};
int ax = 0;
volatile bool flag = 0;

int main(void)
{
    m_clockdivide(0);
    m_disableJTAG();
    m_bus_init();
    m_usb_init();
    
    m_rf_open(CHANNEL, RX_ADDRESS, PACKET_LENGTH);
    sei();
    
    while(TRUE) {
        m_rf_send(TX_ADDRESS, buffer, PACKET_LENGTH);
        m_green(TOGGLE);
        m_wait(200);
        
    }
    
    return 0;
    
}

ISR(INT2_vect) {
    m_green(TOGGLE);
}



