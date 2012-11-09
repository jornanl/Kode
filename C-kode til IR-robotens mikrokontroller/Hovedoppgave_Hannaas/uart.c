/**
 *  uart.c - UART functions.
 *
 * @author Håkon Skjelten <skjelten@pvv.org>
 * @author Changed by Sveinung Helgeland for version 2.0 of RoboRadar
 * @author Changed by Bjørn Syvertsen for version 3.0 of Roboadar
 * @author Changed by Johannes Schrimpf
 * @author Changed by Jannicke Selnes Tusvik
 * @author Changed by Sigurd Hannaas
 * @file uart.c
 * @brief UART functions.
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <math.h>

#include "lego.h"
#include "main.h"
#include "uart.h"
#include "servo.h"
#include "lego.h"
#include "gp2d12.h"
#include "encoder.h"
#include "position.h"
#include "navigation.h"
#include "messages.h"
#include "battery.h"



uint8_t ui8_robot_cmd     = 0;
uint8_t ui8_wait_for_data = 0;
uint8_t ui8_data1 		= 0;
uint8_t ui8_data2 		= 0;
uint8_t ui8_data3 		= 0;
uint8_t ui8_data4 		= 0;
uint8_t ui8_data5 		= 0;
uint8_t ui8_data6 		= 0;

uint8_t  ui8_uartVer   = 2;
uint8_t  ui8_temp  = 0;
uint16_t ui16_tmpx = 0;
uint16_t ui16_tmpy = 0;
uint16_t ui16_tmpt = 0;




/**
 * Initialize the internal UART driver.
 */
int uart_init(uint16_t baudRate)
{
  	switch(baudRate)
    {
    	case 9600:
      		UBRRL = 207;
      		break;

    	case 19200:
      		UBRRL = 103;
      		break;

    	case 38400:
      		UBRRL = 51;
      		break;
    	
		case 57600:
      		UBRRL = 34;
      		break;
    	
		default:
      		UBRRL = 207;
     	break;
    }

  	UCSRA = _BV(U2X);
 	UCSRB = _BV(RXCIE) | _BV(RXEN) | _BV(TXEN);

  	DDRD &= ~(_BV(DDD0));
  	DDRD |= _BV(DDD1);

	//fdevopen(uart_send, NULL, 0);  // Initialize UART to STDOUT 

  	return 0;
}


/**
 * Sends one byte data on the UART.
 */
inline void uart_send(int8_t i8_data)
{
  	while(!(UCSRA & (1 <<  UDRE))) ;
  	UDR = i8_data;
}


/**
 * Sends an uint16_t value on the UART.
 */	
inline void uart_send16(uint16_t ui16_data)	
{
	uart_send((uint8_t) (ui16_data >> 8));
	uart_send((uint8_t) (ui16_data & 0xff));
}


/**
 * Sends an uint8_t in a human readable format
 */	
inline void uart_sendh(uint8_t ui8_data)	
{
	uint8_t ui8_tmp=0;
	if (ui8_data>=100)
	{
		ui8_tmp=(ui8_data/100);
		uart_send(ui8_tmp+48);
		ui8_data-=ui8_tmp*100;
	}
	else
	{
		uart_send(48);
	}
	if (ui8_data>=10)
	{
		ui8_tmp=(ui8_data/10);
		uart_send(ui8_tmp+48);
		ui8_data-=ui8_tmp*10;
	}
	else
	{
		uart_send(48);
	}
	
	uart_send(ui8_data+48);
}

/**
 * Sends an uint16_t in a human readable format
 */	
inline void uart_send16h(int16_t i16_data)	
{
	
	uint16_t ui16_tmp=0;
	if (i16_data>=0)
	{
		uart_send(' ');
	}
	else
	{
		uart_send('-');
		i16_data=-i16_data;
	}

	if (i16_data>=10000)
	{
		ui16_tmp=(i16_data/10000);
		uart_send(ui16_tmp+48);
		i16_data-=ui16_tmp*10000;
	}
	else
	{
		uart_send(48);
	}
	if (i16_data>=1000)
	{
		ui16_tmp=(i16_data/1000);
		uart_send(ui16_tmp+48);
		i16_data-=ui16_tmp*1000;
	}
	else
	{
		uart_send(48);
	}

	if (i16_data>=100)
	{
		ui16_tmp=(i16_data/100);
		uart_send(ui16_tmp+48);
		i16_data-=ui16_tmp*100;
	}
	else
	{
		uart_send(48);
	}
	if (i16_data>=10)
	{
		ui16_tmp=(i16_data/10);
		uart_send(ui16_tmp+48);
		i16_data-=ui16_tmp*10;
	}
	else
	{
		uart_send(48);
	}
	
	uart_send(i16_data+48);

}





/**
 * Signal handler for incoming data
 */
SIGNAL(SIG_UART_RECV)
{
	uint8_t ui8_data = UDR;

	uartVer2(ui8_data);

} // SIGNAL(SIG_UART_RECV)


/**
 * The 
 */
inline int uartVer2(uint8_t ui8_data)
{
		switch (ui8_wait_for_data)
		{
		case 0:
			switch(ui8_data)
			{
				case V2_HUMAN_MODUS:
					ui8_h = 1; 
					send_msg(1);
					break;

				case V2_MACHINE_MODUS:
					ui8_h = 0;
					break;
				
								
				// --------------------------
				case V2_ROBOT_FORWARD:
					robotControl(LEGO_FORWARD);
					ui8_status=6;
					if(ui8_h==1)
					{
						send_msg(7);
					}
					break;

				case V2_ROBOT_STOP:
					robotControl(LEGO_STOP);
					ui8_status=1;
					if(ui8_h==1)
					{
						send_msg(8);
					}
					break;

				case V2_ROBOT_LEFT:
					robotControl(LEGO_LEFT);
					ui8_status=6;
					if(ui8_h==1)
					{
						send_msg(9);
					}
					break;
		
				case V2_ROBOT_RIGHT:
					robotControl(LEGO_RIGHT);
					ui8_status=6;
					if(ui8_h==1)
					{
						send_msg(10);
					}					
					break;
			
				case V2_ROBOT_BACKWARD:
					robotControl(LEGO_BACKWARD);
					ui8_status=7;
					if(ui8_h==1)
					{
						send_msg(11);
					}
					break;

				// --------------------------
				case V2_ROBOT_SET_POS:
					ui8_robot_cmd = ui8_data;
					ui8_wait_for_data =6;
					break;
				
				case V2_ROBOT_CLR_POS:
					i8_reset = 1;
					if(ui8_h==1)
					{
						send_msg(12);
					}
					break;

				case V2_ROBOT_GET_POS:


					if(ui8_h==1)
					{
						uart_send(0x0A);
						uart_send(0x0D);
						uart_send('P');
						uart_send('o');
						uart_send('s');
						uart_send(':');
						uart_send(' ');
						uart_send('x');
						uart_send(':');
						uart_send16h((uint16_t)f_robotX);
						uart_send(' ');
						uart_send('y');
						uart_send(':');
						uart_send16h((uint16_t)f_robotY);
						uart_send(' ');
						uart_send('T');
						uart_send('h');
						uart_send('e');
						uart_send('t');
						uart_send('a');
						uart_send(':');
						uart_send16h((uint16_t)(f_robotTheta*180/M_PI));
						uart_send(0x0A);
						uart_send(0x0D);
					}
					else
					{
						uart_send(V2_MSG_POSITION_HEADER);
						uart_send16((uint16_t)f_robotX);
						uart_send16((uint16_t)f_robotY);
						uart_send16((uint16_t)(f_robotTheta*1000)); // 1/1000 rad resolution.
					}


					break;

				// --------------------------
				case V2_RADAR_SET_ANGLE:
					ui8_robot_cmd = ui8_data;
					ui8_wait_for_data =1;
					break;

				case V2_RADAR_GET_ANGLE:
					if(ui8_h==1)
					{
						uart_send(0x0A);
						uart_send(0x0D);
						uart_send('R');
						uart_send('a');
						uart_send('d');
						uart_send('a');
						uart_send('r');
						uart_send('a');
						uart_send('n');
						uart_send('g');
						uart_send('l');
						uart_send('e');
						uart_send(':');
						uart_send(' ');
						uart_sendh((uint8_t) (90-i8_servoAngle));
						uart_send(0x0A);
						uart_send(0x0D);
					}
					else
					{
						uart_send((uint8_t) (90-i8_servoAngle));
					}
					ui8_wait_for_data =1;
					break;
				
				// --------------------------
				case V2_STATUS:
					if(ui8_h==1)
					{
						uart_send(0x0A);
						uart_send(0x0D);
						uart_send('R');
						uart_send('o');
						uart_send('b');
						uart_send('o');
						uart_send('t');
						uart_send('s');
						uart_send('t');
						uart_send('a');
						uart_send('t');
						uart_send('u');
						uart_send('s');
						uart_send(':');
						uart_send(' ');
						uart_sendh(ui8_status);						
						uart_send(0x0A);
						uart_send(0x0D);
					}
					else
					{
						uart_send(V2_MSG_STATUS_HEADER);
						uart_send(ui8_status+48);						
					}					
					break;

				case V2_PING:
					uart_send(V2_PING_RESPONSE);
					uart_send('k');
					break;

				// --------------------------
				case V2_ROBOT_SET_HEADING:
					ui8_robot_cmd     = ui8_data;
					ui8_wait_for_data = 1;
					break;

				case V2_ROBOT_SET_DELTA_DIST:
					ui8_robot_cmd     = ui8_data;
					ui8_wait_for_data = 1;
					break;

				case V2_ROBOT_GO_POS:
					ui8_robot_cmd     = ui8_data;
					ui8_wait_for_data = 4;
					break;

				// --------------------------
				case V2_GET_IR_SCAN:
					ui8_status=5;
					break;

				case V2_GET_SENSOR_1:
					if (ui8_status !=5)
					{	
						sendIR(0);
						if(ui8_h==1)
						{
							uart_send(0x0A);
							uart_send(0x0D);
						}
					}
					break;

				case V2_GET_SENSOR_2:
					if (ui8_status !=5)
					{
						sendIR(1);
						if(ui8_h==1)
						{
							uart_send(0x0A);
							uart_send(0x0D);
						}
					}
					break;

				case V2_GET_SENSOR_3:
					if (ui8_status !=5)
					{
						sendIR(2);
						if(ui8_h==1)
						{
							uart_send(0x0A);
							uart_send(0x0D);
						}
					}
					break;

				case V2_GET_SENSOR_4:
					if (ui8_status !=5)
					{
						sendIR(3);
						if(ui8_h==1)
						{
							uart_send(0x0A);
							uart_send(0x0D);
						}
					}
					break;

				case V2_CALIBRATE_SENSORS:
					ui8_robot_cmd = ui8_data;
					ui8_wait_for_data =1;
					break;
					
				// -----------
					
				case V2_RECHARGE_COMPLETE:
					battery_init(5200);
					break;
				
				case V2_CHECK_BATTERY:
					if (ui8_needsToCharge)
					{
						uart_send(V2_BATTERY_LOW);
						uart_send16(ui16_chargingTime);
					}
					else
					{
						uart_send(V2_BATTERY_OK);
					}
					break;


				// --------------------------
				case V2_GET_RIGHTWHEEL: //For testing and debug only
					// Total measured distance 8 MSB //
					uart_send((uint8_t) (i16_rightWheelTicks >> 8));
					// Total measured distance 8 LSB //
   					uart_send((uint8_t) (i16_rightWheelTicks & 0xff));
					break;

				case V2_GET_LEFTWHEEL: //For testing and debug only
					uart_send((uint8_t) (i16_leftWheelTicks >> 8));
					// Total measured distance 8 LSB //
  					uart_send((uint8_t) (i16_leftWheelTicks & 0xff));
					break;

				case V2_SET_VER:
					ui8_robot_cmd     = ui8_data;
					ui8_wait_for_data = 1;
					break;

				case V2_SET_C: //Sets data to Port C (for testing)
					ui8_robot_cmd     = ui8_data;
					ui8_wait_for_data = 1;
					break;
				
				// Didn't get a right command
				default:
					uart_send(V2_NACK);
					break;

			} // switch(data)
			break;
		case 1:
			ui8_data1=ui8_data;
			switch(ui8_robot_cmd)
			{
				case V2_SET_C:
					PORTC = ui8_data;
					uart_send(V2_ACK);
					break;

				case V2_ROBOT_SET_POS:
					setRobotPos( ui8_data5|ui8_data6<<8,ui8_data3|ui8_data4<<8,ui8_data1|ui8_data2<<8);
					uart_send(V2_ACK);
					break;

				case V2_RADAR_SET_ANGLE:
					servo_set_angle(90-ui8_data);
					wait_ms(1000);
					uart_send(V2_ACK);
					break;

				case V2_ROBOT_SET_HEADING:
					ui8_status=2;
					ui8_targetHeading=ui8_data;
					uart_send(V2_ACK);
					break;

				case V2_ROBOT_SET_DELTA_DIST:
					if (ui8_status==2 || ui8_status==3)
					{
						ui8_status = 3;
					}
					else
					{
						ui8_status = 4;
					}
					i8_targetDistance = ui8_data;
					uart_send(V2_ACK);
					break;

				case V2_ROBOT_GO_POS:
					goRobotPos(ui8_data3|ui8_data4<<8,ui8_data1|ui8_data2<<8);
					uart_send(V2_ACK);
					break;


				case V2_SET_VER:
					uartVer = ui8_data;
					uart_send(V2_ACK);
					break;

				case V2_CALIBRATE_SENSORS:
					ui8_status = 8;
					ui8_sensornr = ui8_data;
					break;
								
				default:
					uart_send(V2_NACK);
					break;

			} // switch(robot_cmd)
			ui8_robot_cmd     = 0;
			ui8_wait_for_data = 0;
			break; //case 1

		case 2:
			ui8_data2 = ui8_data;
			ui8_wait_for_data--;
			break;
			
		case 3:
			ui8_data3 = ui8_data;
			ui8_wait_for_data--;
			break;

		case 4:
			ui8_data4 = ui8_data;
			ui8_wait_for_data--;
			break;

		case 5:
			ui8_data5 = ui8_data;
			ui8_wait_for_data--;
			break;

		case 6:
			ui8_data6 = ui8_data;
			ui8_wait_for_data--;
			break;
			
		case 7:
			ui8_data4 = ui8_data;
			ui8_wait_for_data--;
			break;

		case 8:
			ui8_data5 = ui8_data;
			ui8_wait_for_data--;
			break;

		case 9:
			ui8_data6 = ui8_data;
			ui8_wait_for_data--;
			break;

		case 10:
			ui8_data6 = ui8_data;
			ui8_wait_for_data--;
			break;

		} // switch (wait_for_data)
	return 0;
} //int uartVer2(char data)







