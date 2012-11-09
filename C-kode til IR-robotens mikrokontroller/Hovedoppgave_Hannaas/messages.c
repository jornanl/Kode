/** 
 *  messages.c - human readable messages
 * 
 * @author Johannes Schrimpf
 * @file messages.c
 * @brief human readable messages
 */

#include "messages.h"
#include "uart.h"

/**
 * Sends messages to terminal which are readable by humans
 */
inline void send_msg(uint8_t msg)
{

	switch(msg)
	{
		case 1:
			uart_send(0x0A);
			uart_send(0x0D);
			uart_send('L');
			uart_send('e');
			uart_send('g');
			uart_send('o');
			uart_send('r');
			uart_send('o');
			uart_send('b');
			uart_send('o');
			uart_send('t');
			uart_send(' ');
			uart_send('2');
			uart_send('0');
			uart_send('0');
			uart_send('9');
			uart_send(0x0A);
			uart_send(0x0D);
			break;

		case 2:
			uart_send(0x0A);
			uart_send(0x0D);
			uart_send('S');
			uart_send('t');
			uart_send('a');
			uart_send('r');
			uart_send('t');
			uart_send('i');
			uart_send('n');
			uart_send('g');
			uart_send(' ');
			uart_send('s');
			uart_send('c');
			uart_send('a');
			uart_send('n');	
			break;

		case 3:
			uart_send(0x0A);
			uart_send(0x0D);
			uart_send('S');
			uart_send('c');
			uart_send('a');
			uart_send('n');
			uart_send(' ');
			uart_send('f');
			uart_send('i');
			uart_send('n');
			uart_send('i');
			uart_send('s');
			uart_send('h');
			uart_send('e');
			uart_send('d');
			uart_send(0x0A);
			uart_send(0x0D);
			break;

		case 4:
			uart_send(0x0A);
			uart_send(0x0D);
			uart_send('R');
			uart_send('o');
			uart_send('b');
			uart_send('o');
			uart_send('t');
			uart_send(' ');
			uart_send('a');
			uart_send('r');
			uart_send('r');
			uart_send('i');
			uart_send('v');
			uart_send('e');
			uart_send('d');
			uart_send(0x0A);
			uart_send(0x0D);
			break;
		
		case 5:
			uart_send(0x0A);
			uart_send(0x0D);
			uart_send('C');
			uart_send('o');
			uart_send('l');
			uart_send('l');
			uart_send('i');
			uart_send('s');
			uart_send('i');
			uart_send('o');
			uart_send('n');
			uart_send(' ');
			uart_send('W');
			uart_send('a');
			uart_send('r');
			uart_send('n');
			uart_send('i');
			uart_send('n');
			uart_send('g');
			uart_send('!');
			uart_send(0x0A);
			uart_send(0x0D);
			break;

		case 6:
			uart_send(0x0A);
			uart_send(0x0D);
			uart_send('C');
			uart_send('o');
			uart_send('l');
			uart_send('l');
			uart_send('i');
			uart_send('s');
			uart_send('i');
			uart_send('o');
			uart_send('n');
			uart_send(' ');
			uart_send('E');
			uart_send('r');
			uart_send('r');
			uart_send('o');
			uart_send('r');
			uart_send('!');
			uart_send(' ');
			uart_send('S');
			uart_send('t');
			uart_send('o');
			uart_send('p');
			uart_send('p');
			uart_send('i');
			uart_send('n');
			uart_send('g');
			uart_send(' ');
			uart_send('r');
			uart_send('o');
			uart_send('b');
			uart_send('o');
			uart_send('t');
			uart_send(0x0A);
			uart_send(0x0D);
			break;

		case 7:
			uart_send(0x0A);
			uart_send(0x0D);
			uart_send('D');
			uart_send('r');
			uart_send('i');
			uart_send('v');
			uart_send('i');
			uart_send('n');
			uart_send('g');
			uart_send(' ');
			uart_send('f');
			uart_send('o');
			uart_send('r');
			uart_send('w');
			uart_send('a');
			uart_send('r');
			uart_send('d');
			uart_send(0x0A);
			uart_send(0x0D);
			break;

		case 8:
			uart_send(0x0A);
			uart_send(0x0D);
			uart_send('S');
			uart_send('t');
			uart_send('o');
			uart_send('p');
			uart_send('p');
			uart_send('i');
			uart_send('n');
			uart_send('g');
			uart_send(' ');
			uart_send('r');
			uart_send('o');
			uart_send('b');
			uart_send('o');
			uart_send('t');
			uart_send(0x0A);
			uart_send(0x0D);
			break;

		case 9:
			uart_send(0x0A);
			uart_send(0x0D);
			uart_send('R');
			uart_send('o');
			uart_send('t');
			uart_send('a');
			uart_send('t');
			uart_send('i');
			uart_send('n');
			uart_send('g');
			uart_send(' ');
			uart_send('l');
			uart_send('e');
			uart_send('f');
			uart_send('t');
			uart_send(0x0A);
			uart_send(0x0D);
			break;

		case 10:
			uart_send(0x0A);
			uart_send(0x0D);
			uart_send('R');
			uart_send('o');
			uart_send('t');
			uart_send('a');
			uart_send('t');
			uart_send('i');
			uart_send('n');
			uart_send('g');
			uart_send(' ');
			uart_send('r');
			uart_send('i');
			uart_send('g');
			uart_send('h');
			uart_send('t');
			uart_send(0x0A);
			uart_send(0x0D);
			break;

		case 11:
			uart_send(0x0A);
			uart_send(0x0D);
			uart_send('D');
			uart_send('r');
			uart_send('i');
			uart_send('v');
			uart_send('i');
			uart_send('n');
			uart_send('g');
			uart_send(' ');
			uart_send('b');
			uart_send('a');
			uart_send('c');
			uart_send('k');
			uart_send('w');
			uart_send('a');
			uart_send('r');
			uart_send('d');
			uart_send(0x0A);
			uart_send(0x0D);
			break;

		case 12:
			uart_send(0x0A);
			uart_send(0x0D);
			uart_send('R');
			uart_send('e');
			uart_send('s');
			uart_send('e');
			uart_send('t');
			uart_send('e');
			uart_send('d');
			uart_send(' ');
			uart_send('p');
			uart_send('o');
			uart_send('s');
			uart_send('i');
			uart_send('t');
			uart_send('i');
			uart_send('o');
			uart_send('n');
			uart_send(0x0A);
			uart_send(0x0D);
			break;
		}
}
