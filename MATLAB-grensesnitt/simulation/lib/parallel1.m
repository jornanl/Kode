function Ir=parallel1(x,y,theta,s,rot,b,e)
myX=x;
myY=y;

a=(90+theta-s); % a==relative angel of the IR sensor in degrees
radA=pi*a/180; % a in radians

%  "north"
%  _______
%    ___
%   |   |
if strcmp(rot,'n'),    
    v=[];
    v(1)=atan((b-myY)/(2*b-myX));
    v(2)=pi-atan( (b-myY)/(b+myX) );
    v(3)=3*pi/2-atan(myX/myY);
    v(4)=3*pi/2+atan((b-myX)/myY);
    v=v*180/pi;
    if (a>=-90 && a<=v(4)-360) || ( a>=v(3) && a<=v(4))
        ir=(myY/cos(3*pi/2-radA)) + e(2);
    elseif ( a>v(4)-360 && a<=v(1)) || ( a>v(4) && a<=v(1)+360 ) || ( a>=v(2) && a<v(3) )
        ir=255;
    elseif ( a>v(1) && a<v(2) ) || (a>v(1)+360 && a<=450)
        ir=(b-myY)/sin(radA) + e(4);
    else
        disp('fault...not all radar angles are covered!')
    end
    
 
%       |  _
% "west"| |_ 
%       |
elseif strcmp(rot,'w'),
    v=[];
    v(1)=pi/2 + atan(myX/(2*b-myY));
    v(2)=3*pi/2 - atan(myX/(b+myY));
    v(3)=2*pi-atan(myY/(b-myX));
    v(4)=atan((b-myY)/(b-myX));
    v=v*180/pi;
    if (a>=-90 && a<v(3)-360) || (a>v(4) && a<= v(1)) || ( a>=v(2) && a<v(3) ) ...
            || ( a>v(4)+360 && a<=450 )
        ir=255;
    elseif (a>v(1) && a<v(2))
        ir=myX/cos(radA-pi) + e(3);
    elseif (a >= v(3)-360 && a<= v(4)) || (a >= v(3) && a<= v(4)+360)
        ir=(b-myX)/cos(2*pi-radA) + e(1);
    else
        disp('fault...not all radar angles are covered!')
    end
    
%  |_|
% _____
% "south"
elseif strcmp(rot,'s'),
    v=[];
    v(1)=pi/2-atan((b-myX)/(b-myY));
    v(2)=pi/2+atan(myX/(b-myY));
    v(3)=pi + atan(myY/(b+myX));
    v(4)=2*pi - atan(myY/(2*b-myX));
    v=v*180/pi;
    if (a>=-90 && a<=v(4)-360) || (a>v(3) && a< v(4)) 
        ir=myY/cos(radA-3*pi/2) + e(2);
    elseif (a>=v(2) && a<v(3)) || (a>v(4)-360 && a<=v(1) ) || (a>=v(4) && a<v(1)+360)
        ir=255;
    elseif (a >= v(1) && a<= v(2)) || (a >= v(1)+360 && a<= 450)
        ir=(b-myY)/cos(pi/2-radA) + e(4);
    else
        disp('fault...not all radar angles are covered!')
    end
    
%_  |
%_| | 'east'
%   |
elseif strcmp(rot,'e'),
    v=[];
    v(1)=pi/2 - atan((b-myX)/(2*b-myX));
    v(2)=pi-atan((b-myY)/myX);
    v(3)=pi+atan(myY/myX);
    v(4)=3*pi/2 + atan((b-myX)/(b+myX));
    v=v*180/pi;    
    if (a>v(4)-360 && a<v(1)) || (a>v(4) && a<=v(1)+360)
        ir=(b-myX)/cos(radA) + e(1);
    elseif ( (a>=-90 && a<v(4)-360) || (a>=v(1) && a<v(2)) || (a>v(3) && a<=v(4)) || (a>=v(1)+360 && a<=450))
        ir=255;
    elseif ( a>=v(2) && a<=v(3) )
        ir=(myX)/cos(pi-radA) + e(3);
    else
        disp('fault...not all radar angles are covered!')
    end
   
end
if ir>255,
    Ir=255;
else
    Ir=abs(ir);
end
%------------end-passage-modul--------------%
