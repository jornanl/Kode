function data = nxt_get_motor_info(motor)

SetMotor(motor);

data = GetMotorSettings(motor);
end