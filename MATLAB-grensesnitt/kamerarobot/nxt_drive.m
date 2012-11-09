function movement = nxt_drive(speed, movement, rot_dist_cm)

ResetMotorAngle('MOTOR_B');
ResetMotorAngle('MOTOR_C');

%unøyaktig tall
gear = 15.82;

rot_dist = gear*rot_dist_cm;
SetMotor(MOTOR_B);

SetPower(speed);
SetRampMode off;
%SyncToMotor(MOTOR_C);
SendMotorSettings;

SetMotor(MOTOR_C);

SetPower(speed);
%SyncToMotor(MOTOR_B)
SetRampMode off;
SendMotorSettings;

while(1)
    data_B = nxt_get_motor_info(MOTOR_B);
    data_C = nxt_get_motor_info(MOTOR_C);
    dist_B = data_B.Angle;
    dist_C = data_C.Angle;
    if(dist_B >= rot_dist || dist_C >= rot_dist)
        break;
    end
    
end


nxt_stop()
%SetMotor(MOTOR_B);
%SetPower(0);
%SendMotorSettings;

%SetMotor(MOTOR_C);
%SetPower(0);
%SendMotorSettings;


tot_dist = (abs(dist_B/gear) + abs(dist_C/gear))/2;

move_size = size(movement);

movement(move_size(2)+1).action = 'drive';
movement(move_size(2)+1).value = tot_dist;

end