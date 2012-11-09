function drawRectangle(G,rectWidth,rectLength)
% This function draws a rectagle around the given position of the robot
% The width is twics as big as it is supposed to be?


X = get(G,'x');
roboPose = get(X{1,1},'x');

% rotate rectangular region to global frame and draw
% figure(7);clf;hold on;
rectReg=[-rectWidth 0 +rectWidth 0;...
        -rectWidth rectLength rectWidth rectLength;...
        -rectWidth 0 -rectWidth rectLength;...
        rectWidth 0 rectWidth rectLength ];
rotmat3=[cos(-roboPose(3)+pi/2) sin(-roboPose(3)+pi/2) ; -sin(-roboPose(3)+pi/2) cos(-roboPose(3)+pi/2)];
rotmat3=[rotmat3 [0 0;0 0];[0 0;0 0] rotmat3];
rectRegGlobalFrame = ( rotmat3 * rectReg' )';
rectRegGlobalFrame(1:4,1:2:3) = rectRegGlobalFrame(1:4,1:2:3) + roboPose(1);
rectRegGlobalFrame(1:4,2:2:4) = rectRegGlobalFrame(1:4,2:2:4) + roboPose(2);
for r=1:4,
    plot([rectRegGlobalFrame(r,1) rectRegGlobalFrame(r,3)],[rectRegGlobalFrame(r,2) rectRegGlobalFrame(r,4)],'b:')
end

