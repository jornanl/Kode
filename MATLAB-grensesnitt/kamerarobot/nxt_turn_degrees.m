function [vinkel rot_B rot_C] = nxt_turn_degrees(degrees)

%positiv retning er rotasjon med klokka bla bla
data_B = nxt_get_motor_info(MOTOR_B);
data_C = nxt_get_motor_info(MOTOR_C);

pos_B = data_B.Angle;
pos_C = data_C.Angle;

gear = 4.222;


dist = (degrees*gear)/2;

rot_B = dist + pos_B;
rot_C = dist + pos_C;


MotorRotateAbs(MOTOR_B,dist,10);
MotorRotateAbs(MOTOR_C,-dist,10);


vinkel = dist;
