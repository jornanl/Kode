function cam_turn_degrees(degrees)

%denne vrir motoren til vinkelen degrees, altså absolutt vinkel.
%forholdet i girene er 1,611111
gear = 1.6111111;
motor_turn = degrees*gear;

MotorRotateAbs(MOTOR_A,motor_turn,10);
end

