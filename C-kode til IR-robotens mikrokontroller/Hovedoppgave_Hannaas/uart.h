/** 
 *  uart.h - UART interface.
 *
 * @author Håkon Skjelten <skjelten@pvv.org>
 * @author Changed by Sveinung Helgeland for version 2.0 of RoboRadar
 * @author Changed by Bjørn Syvertsen for version 3.0 of RoboRadar
 * @author Changed by Johannes Schrimpf
 * @author Changed by Jannicke Selnes Tusvik
 * @author Changed by Sigurd Hannaas
 * @file uart.h
 * @brief UART interface.
 */


#ifndef __uart_h__
#define __uart_h__




/**
 * Initialize the internal UART driver.
 */
int uart_init(uint16_t baudRate);

/**
 * Sends one byte data on the UART.
 */
inline void uart_send(int8_t i8_data);
inline void uart_sendh(uint8_t ui8_data);
inline void uart_send16(uint16_t ui16_data);
inline void uart_send16h(int16_t i16_data);


uint8_t uartVer;

inline int uartVer1(uint8_t ui8_data);
inline int uartVer2(uint8_t ui8_data);



/**
 * Commands to Robot
 */
#define V2_HUMAN_MODUS	 			0x7A 	//char -> z					dec: 122
#define V2_MACHINE_MODUS	 		0x6D 	//char -> m					dec: 109

#define V2_ROBOT_FORWARD 			0x77 	//char -> w					dec: 119
#define V2_ROBOT_STOP 				0x73 	//char -> s					dec: 115
#define V2_ROBOT_LEFT 				0x61 	//char -> a					dec: 97
#define V2_ROBOT_RIGHT 				0x64 	//char -> d					dec: 100
#define V2_ROBOT_BACKWARD 			0x78 	//char -> x					dec: 120

#define V2_ROBOT_SET_POS 			0x71	//char -> q <-(xxyytt)		dec: 113
#define V2_ROBOT_GET_POS 			0x70	//char -> p ->(xxyyyy)		dec: 112
#define V2_ROBOT_CLR_POS			0x63 	//char -> c					dec: 99

#define V2_RADAR_SET_ANGLE 			0x72 	//char -> r <-(t)			dec: 114
#define V2_RADAR_GET_ANGLE 			0x74 	//char -> t					dec: 116


#define V2_ROBOT_SET_HEADING 		0x68 	//char -> h <-(t)			dec: 104
#define V2_ROBOT_SET_DELTA_DIST	 	0x6A 	//char -> j <-(zz)			dec: 106
#define V2_ROBOT_GO_POS 			0x67 	//char -> g <-(xxyy)		dec: 103

#define V2_STATUS 					0x62 	//char -> b					dec: 98
#define V2_PING 					0x6F 	//char -> o					dec: 111
#define V2_PING_RESPONSE_MATLAB		0x6b 	//char -> k					dec: 107


#define V2_GET_IR_SCAN 				0x66 	//char -> f					dec: 102
#define V2_GET_SENSOR_1 			0x31 	//char -> 1					dec: 49
#define V2_GET_SENSOR_2 			0x32 	//char -> 2					dec: 50
#define V2_GET_SENSOR_3 			0x33 	//char -> 3					dec: 51
#define V2_GET_SENSOR_4 			0x34 	//char -> 4					dec: 52
//#define V2_GET_SENSOR_5 			0x35 	//char -> 5
//#define V2_GET_SENSOR_6 			0x36 	//char -> 6
//#define V2_GET_SENSOR_7 			0x37 	//char -> 7
//#define V2_GET_SENSOR_8 			0x38 	//char -> 8

#define V2_CALIBRATE_SENSORS		0x69	//char -> i					dec: 105

#define V2_SET_VER 					0x76	//char -> v					dec: 118
#define V2_SET_C					0x35 	//char -> 5					dec: 53

#define V2_CHECK_BATTERY			0x3F	//char -> ?					dec: 63
#define V2_RECHARGE_COMPLETE		0x3E	//char -> >					dec: 62

#define V2_GET_LEFTWHEEL 			0x01 	//char -> -					dec: 1
#define V2_GET_RIGHTWHEEL 			0x02	//char -> -					dec: 2





/**
 * Messages to Matlab  (decimal values are displayed in the Matlab console if Matlab interprets the values as wrong/not expected)
 */
#define V2_PING_RESPONSE 				0x6B 	//char -> k					dec: 107
#define V2_PING_MATLAB					0x6F 	//char -> o					dec: 111

#define V2_MSG_ARRIVED 					0x71 	//char -> q					dec: 113
#define V2_MSG_IR_SCAN_HEADER 			0x66	//char -> f					dec: 102
#define V2_MSG_IR_SCAN_HEADER_END		0x67	//char -> g					dec: 103
#define V2_MSG_IR_HEADER_1 				0x31 	//char -> 1					dec: 49
#define V2_MSG_IR_HEADER_2 				0x32 	//char -> 2					dec: 50
#define V2_MSG_IR_HEADER_3 				0x33 	//char -> 3					dec: 51
#define V2_MSG_IR_HEADER_4 				0x34 	//char -> 4					dec: 52
#define V2_MSG_IR_HEADER_NO_DATA_1  	0x35 	//char -> 5					dec: 53
#define V2_MSG_IR_HEADER_NO_DATA_2 		0x36 	//char -> 6					dec: 54
#define V2_MSG_IR_HEADER_NO_DATA_3 		0x37 	//char -> 7					dec: 55
#define V2_MSG_IR_HEADER_NO_DATA_4 		0x38 	//char -> 8					dec: 56
#define V2_MSG_IR_CALIBRATION_END		0x21	//char -> !					dec: 33
#define V2_MSG_POSITION_HEADER			0x70	//char -> p					dec: 112
#define V2_MSG_RADAR_HEADER		 		0x72	//char -> r					dec: 114
#define V2_MSG_STATUS_HEADER			0x73	//char -> s					dec: 115

#define V2_NACK 						0x6E	//char -> n					dec: 110
#define V2_ACK 							0x61	//char -> a					dec: 97
#define V2_MSG_BUSY 					0x62 	//char -> b					dec: 98

#define V2_COLLISION_WARNING			0x77	//char -> w					dec: 119
#define V2_COLLISION_ERROR  			0x65	//char -> e					dec: 101

#define V2_BATTERY_LOW					0x3C	//char -> <					dec: 60
#define V2_BATTERY_OK					0x3D	//char -> =					dec: 61

#define V2_DEBUG_MARKER1				0x23	//char -> #					dec: 35
#define V2_DEBUG_MARKER2				0x24	//char -> $					dec: 36
#define V2_DEBUG_MARKER3				0x25	//char -> %					dec: 37

#endif /* __uart_h__ */
