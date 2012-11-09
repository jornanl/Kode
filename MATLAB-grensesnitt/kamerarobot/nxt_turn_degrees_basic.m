function [vinkel rot_B rot_C] = nxt_turn_degrees_basic(degrees)

%positiv retning er rotasjon med klokka bla bla
gear = 4.222;


dist = (degrees*gear)/2;

data_B = nxt_get_motor_info(MOTOR_B);
data_C = nxt_get_motor_info(MOTOR_C);
pos_B = data_B.Angle;
pos_C = data_C.Angle;
rot_B = dist + pos_B;
rot_C = dist - pos_C;


MotorRotateAbs(MOTOR_B, rot_B,10 );
MotorRotateAbs(MOTOR_C, rot_C,10 );


