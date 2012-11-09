function Ir=deadEnd(x,y,theta,s,dir,b,e)

a=(90+theta-s);
radA=pi*a/180; 

% "East"
% ___
% ___|
%
if dir=='e'
    v=[];
    v(1)=atan((b-y)/(b-x));
    v(2)=pi;
    v(3)=2*pi-atan(y/(b-x));
    v=v*180/pi;    
    if (a>=-90 && a<v(3)-360) || ( a>v(2) && a < v(3) )
        ir= y/cos(3*pi/2-radA) + e(2);
    elseif (a>=v(3)-360 && a<v(1)) || (a>=v(3) && a<v(1)+360)
        ir=(b-x)/cos(radA) + e(1);
        
    elseif (a>=v(1) && a<v(2)) || (a>=v(1)+360 && a<=450)
        ir= (b-y)/cos(pi/2-radA) + e(4);
    elseif (a==180)
        ir=255;
    else
        disp('fault...not all radar angles are covered!')
    end


else
    disp('warning...not a valid direction spesified!')
end
if (abs(ir)>255)
    Ir=255;
else
    Ir=abs(ir);
end