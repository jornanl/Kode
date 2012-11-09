%-------corner modul---------%
function Ir=maze8(x,y,theta,s,type,b,e)
% "Empty circle"

RAD=100; %[cm]


% forbehandling av data
a=(90+theta-s); % 0< a < 180 degrees
radA=pi*a/180;


% "Circle"
% 
% 
%
    %v=[];
    %v(1)=atan((b-y)/(b-x));
    %v(2)=pi/2+atan(x/(b-y));
    %v(3)=pi+atan(y/x);
    %v(4)=3*pi/2+atan((b-x)/y);
    %v=v*180/pi;
    
    %if (a>=v(1) && a<v(2)) || ( a>=v(1)+360 && a <= 450 )
    %    ir= (b-y)/cos(pi/2-radA);
    %elseif (a>=v(2) && a<v(3))
    %    ir= x/cos(pi-radA);
    %elseif (a>=v(3) && a< v(4)) || (a>=-90 && a<v(4)-360)
    %    ir=y/cos(3*pi/2-radA);
    %elseif (a >= v(4)-360 && a < v(1)) || (a >= v(4) && a < v(1)+360)
    %    ir=(b-x)/cos(0-radA);
    %else
    %    disp('fault...not all radar angles are covered!')
    %end
ir= 255;
    
if (abs(ir)>255)
    Ir=255;
else
    Ir=abs(ir);
end
