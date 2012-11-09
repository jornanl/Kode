function xy = mapping_get_robot_pos(movement)
cur_cord_x = 0;
cur_cord_y = 0;

xy = [cur_cord_x cur_cord_y];

last_angle = 0;
length = size(movement);

rad_const = pi/180;

for i=1:length(2)
    
    if(strcmp('drive', movement(i).action))
        x = sin(last_angle*rad_const)*movement(i).value;
        y = cos(last_angle*rad_const)*movement(i).value;
        
        cur_cord_x = cur_cord_x + x;
        cur_cord_y = cur_cord_y + y;
        
        xy = [cur_cord_x cur_cord_y];         
    end
    if(strcmp('rot',movement(i).action))    
        last_angle = last_angle + movement(i).value;
    end
    
end

end