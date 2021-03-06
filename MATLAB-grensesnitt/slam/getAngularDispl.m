function [aL,aR]=getAngularDispl(pose1,pose2, rl,rr,b)
%
% pose1 and pose2 are [1x3] arrays representing robotpose [x, y, theta],
% aL is the angular displacement of left wheel between pose1 and pose2,
% aR is the angular displacement of right wheel between pose1 and pose2
% rl=radius of left wheel, rr=radius of right wheel and b=width between
% wheels.
% All angles in rad.
aL=0;
aR=0;

x1=pose1(1);
y1=pose1(2);
theta1=pose1(3);

x2=pose2(1);
y2=pose2(2);
theta2=pose2(3);

dx=x2-x1;
dy=y2-y1;

dist=sqrt(dx^2 + dy^2);

% angular displacement from distance
aL=aL + dist/rl;
aR=aR + dist/rr;

% angular displacement from rotations
deltaTheta=(theta2 - theta1);
if deltaTheta~=0,
    if deltaTheta<0 && deltaTheta>=-pi,
        % rotation clockwise
        aL=aL + (deltaTheta*b/2)/rl;
        aR=aR - (deltaTheta*b/2)/rr;
    elseif deltaTheta>pi,
        % rotation clockwise
        deltaTheta=deltaTheta-2*pi;
        aL=aL - (deltaTheta*b/2)/rl;
        aR=aR + (deltaTheta*b/2)/rr;
        
    elseif deltaTheta<-pi,
        % rotation counterclockwise
        deltaTheta=deltaTheta+2*pi;
        aL=aL - (deltaTheta*b/2)/rl;
        aR=aR + (deltaTheta*b/2)/rr;
    elseif deltaTheta>0 && deltaTheta<=pi,
        % rotation counterclockwise
        aL=aL - (deltaTheta*b/2)/rl;
        aR=aR + (deltaTheta*b/2)/rr;
    
        
    end
else
    % no rotation
end
%aL=aL*180/pi;        
%aR=aR*180/pi;
