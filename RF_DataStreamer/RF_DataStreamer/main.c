/*
 * RF_DataStreamer.c
 *
 * Created: 11/19/2014 7:41:53 PM
 *  Author: Tyler
 */ 


#include <avr/io.h>
#include "m_general.h"
#include "m_usb.h"
#include "m_rf.h"
#include "m_bus.h"

char packet[3] = {0, 0, 0};
volatile char x_pos;
volatile char y_pos;
volatile char orientation;

int main(void)
{
	m_clockdivide(0);
	m_usb_init();
	m_bus_init();
	m_rf_open(2,0x18,3);
	sei();

	while(1)
	{
	}
}

ISR(INT2_vect){
	
	// read the packet
	m_rf_read(packet, 3);
	x_pos = packet[0];
	y_pos = packet[1];
	orientation = packet[2];	// 0-127 maps to 0-2pi radians
	
	m_usb_tx_long((long)x_pos);
	m_usb_tx_string("\t");
	m_usb_tx_long((long)y_pos);
	m_usb_tx_string("\t");
	m_usb_tx_long((long)orientation);
	m_usb_tx_string("\n");
}
