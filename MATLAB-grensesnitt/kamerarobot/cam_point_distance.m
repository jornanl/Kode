function [lines I rot_angel_motor rot_angel dist] = cam_point_distance(cam_path, myHandles)

%Hardkodet bredde og høyde for mitt ip cam

bredde = 640;
hoyde = 480;

[lines I] = cam_grab_and_compute(cam_path, myHandles);

[rot_angel_motor rot_angel] = get_direction(lines, bredde, hoyde);

cam_turn_degrees(rot_angel_motor);

OpenUltrasonic(SENSOR_1);
dist = GetUltrasonic(SENSOR_1);
CloseSensor(SENSOR_1);

PAUSE2(0.5);

cam_turn_degrees(0);

end



