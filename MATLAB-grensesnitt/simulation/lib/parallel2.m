%------------parallel2 ----------------- %
function Ir=parallel2(x,y,theta,s,dir,b,e)

%myX=x;
%myY=y;

a=(90+theta-s);
radA=pi*a/180;

 
%  _    _
%  _|  |_
% 

v=[];
v(1)=atan((b-y)/(b-x));
v(2)=pi-atan((b-y)/x);
v(3)=pi+atan(y/x);
v(4)=2*pi-atan(y/(b-x));
v=v*180/pi;
if dir=='ns'    
    if (a>=-90 && a<v(4)-360) || ( a>v(3) && a<v(4)) || ( a>v(1) && a<v(2)) || (a>v(1)+360 && a<=450)
        ir=255;
    elseif ( a>=v(4)-360 && a<=v(1)) || ( a>=v(4) && a<=v(1)+360 ) 
        ir=(b-x)/cos(radA) + e(1);
    elseif ( a>=v(2) && a<=v(3) )
        ir=x/cos(pi-radA) + e(3);
    else
        disp('fault...not all radar angles are covered!')
    end
elseif dir=='we'
    if (a>=-90 && a<v(4)-360) || ( a>v(3) && a<v(4)) 
        ir=y/cos(3*pi/2-radA) + e(2);
    elseif ( a>=v(4)-360 && a<=v(1)) || ( a>=v(4) && a<=v(1)+360 ) || ( a>=v(2) && a<=v(3) )
        ir=255;
    elseif  ( a>v(1) && a<v(2)) || (a>v(1)+360 && a<=450)
        ir=(y-b)/cos(pi/2-radA) + e(4);
    else
        disp('fault...not all radar angles are covered!')
    end
end
if (abs(ir)>255)
    Ir=255;
else
    Ir=abs(ir);
end