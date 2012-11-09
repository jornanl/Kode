%-------start parallel 3--------%
function Ir=parallel3(x,y,theta,s,dir,b,e)

a=(90+theta-s);
radA=pi*a/180; 

% "West-East"
% _______
% _______
%
if dir=='we'
    if (a>=-90 && a<0) || ( a>180 && a < 360 )
        ir= y/cos(3*pi/2-radA) + e(4);
    elseif (a>0 && a<180) || (a>360 || a <=450)
        ir= (b-y)/cos(pi/2-radA) + e(2);
    elseif (a==0 || a== 180 || a==360)
        ir=255;
    else
        disp('fault...not all radar angles are covered!')
    end

% "North-South"
%   | |
%   | |
%   | |
elseif dir=='ns'
    if (a>-90 && a<90) || ( a>270 && a < 450 )
        ir= (b-x)/cos(radA) + e(1);        
    elseif (a>90 && a<270)
        ir= x/cos(pi-radA) + e(3);
    elseif (a==-90 || a== 90 || a==270 || a==450)
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
%-----end parallel---------%