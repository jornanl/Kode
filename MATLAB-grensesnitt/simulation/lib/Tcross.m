%------------start T-cross modul------------%
function Ir=Tcross(x,y,theta,s,dir,b,e)


a=(90+theta-s);
radA=pi*a/180; 

% "north"
% _______
% __   __
%   | |
if dir=='Tnorth'
    v=[];
    v(1)=atan((b-y)/(2*b-x));
    v(2)=pi-atan((b-y)/(b+x));
    v(3)=pi+atan(y/(b+x));
    v(4)=pi+atan(y/x);
    v(5)=3*pi/2-atan(x/(b+y));
    v(6)=3*pi/2+atan((b-x)/(b+y));
    v(7)=2*pi-atan(y/(b-x));
    v(8)=2*pi-atan(y/(2*b-x));
    v=v*180/pi;
    if (a>=v(1) && a<=v(2)) || ( a>=v(1)+360 && a<=450 )
        ir= (b-y)/cos(pi/2-radA) + e(4);
    elseif (a>=-90 && a<v(6)-360) || (a>v(8)-360 && a<v(1)) || ( a>v(2) && a<v(3) )...
            || ( a>v(5) && a<v(6) ) || ( a>v(8) && a<v(1)+360)
        ir=255;
    elseif  ( a>=v(3) && a<v(4) )
        ir= y/sin(radA-pi) + e(2);
    elseif ( a>=v(4) && a<=v(5) )
        ir= x/sin(3*pi/2-radA) + e(3);
    elseif ( a>=v(6) && a<v(7) ) || ( a>=v(6)-360 && a<v(7)-360 )   
        ir= (b-x)/sin(radA-3*pi/2) + e(1);
    elseif ( a>=v(7) && a<v(8) ) || ( a>=v(7)-360 && a<v(8)-360 )   
        ir=y/sin(2*pi-radA) + e(2);
    else
        disp('fault...not all radar angles are covered!')
    end

% __| |__
% _______
% "south"
elseif dir=='Tsouth'
    v=[];
    v(1)=0;
    v(2)=atan((b-y)/(b-x));
    v(3)=pi/2;
    v(4)=pi-atan((b-y)/x);
    v(5)=pi;
    v=v*180/pi;
    warning off;
    if (a>0 && a<v(2)) || ( a>360 && a<v(2)+360 )
        ir= (b-y)/sin(radA) + e(4);
    elseif ( a>=v(2) && a<=v(3) ) || ( a>=v(2)+360 && a<450)
        ir= (b-x)/sin(pi/2-radA) + e(1);
    elseif  ( a>v(3) && a<v(4) )
        ir= x/sin(radA-pi/2) + e(3);
    elseif ( a>=v(4) && a<v(5) ) 
        ir= (b-y)/sin(pi-radA) + e(4);
    elseif ( a>v(5) && a<360 ) || (a>=-90 && a<0)   
        ir= y/cos(3*pi/2-radA) + e(2);
    elseif (a==0 || a==90 || a==180 || a==360 || a==450 )    
        ir=255;
    else
        disp('fault...not all radar angles are covered!')
    warning on;
    end
elseif dir=='T_west'
    v=[];
    v(1)=atan((b-y)/(2*b-x));
    v(2)=atan((b-y)/(b-x));
    v(3)=pi/2-atan((b-x)/(2*b-y));
    v(4)=pi/2+atan(x/(2*b-y));
    v(5)=3*pi/2-atan(x/(b+y));
    v(6)=3*pi/2+atan((b-x)/(b+y));
    v(7)=2*pi-atan(y/(b-x));
    v(8)=2*pi-atan(y/(2*b-x));
    v=v*180/pi;
    if (a>=v(1) && a<=v(2)) || ( a>=v(1)+360 && a<=v(2)+360 )
        ir= (b-y)/sin(radA) + e(4);
    elseif (a>=-90 && a<v(6)-360) || (a>v(8)-360 && a<v(1)) || ( a>v(3) && a<v(4) )...
            || ( a>v(5) && a<v(6) ) || ( a>v(8) && a<v(1)+360) || (a>v(3)+360 && a<=450)
        ir=255;
    elseif  ( a>v(2) && a<=v(3) ) || ( a>v(2)+360 && a<=v(3)+360 )
        ir= (b-x)/sin(pi/2-radA) + e(1);
    elseif ( a>=v(4) && a<=v(5) )
        ir= x/cos(radA-pi) + e(3);
    elseif ( a>=v(6) && a<v(7) ) || ( a>=v(6)-360 && a<v(7)-360 )   
        ir= (b-x)/sin(radA-3*pi/2) + e(1);
    elseif ( a>=v(7) && a<v(8) ) || ( a>=v(7)-360 && a<v(8)-360 )   
        ir=y/sin(2*pi-radA) + e(2);
    else
        disp('fault...not all radar angles are covered!')
    end
elseif dir=='T_east'
    v=[];
    v(1)=atan((2*b-y)/(b-x));
    v(2)=pi/2+atan(x/(2*b-y));
    v(3)=pi/2+atan(x/(b-y));
    v(4)=pi-atan((b-y)/(b+x));
    v(5)=pi+atan(y/(b+x));
    v(6)=pi+atan(y/x);
    v(7)=3*pi/2-atan(x/(b+y));
    v(8)=3*pi/2+atan((b-x)/(b+y));
    v=v*180/pi;
    
    if (a>=-90 && a<v(8)-360) || (a>v(1) && a<v(2)) || ( a>v(4) && a<v(5) ) ||...
            ( a>v(7) && a<v(8) ) || (a>v(1)+360 && a<=450)
        ir=255;
    elseif ( a>=v(2) && a<v(3) ) 
        ir= x/sin(radA-pi/2) + e(3);
    elseif  ( a>=v(3) && a<=v(4) )
        ir= (b-y)/sin(pi-radA) + e(4);
    elseif ( a>=v(5) && a<v(6) ) 
        ir= y/sin(radA-pi) + e(2);
    elseif ( a>=v(6) && a<=v(7) )   
        ir= x/sin(3*pi/2-radA) + e(3);
    elseif (a>=v(8)-360 && a<=v(1)) || (a>=v(8) && a<=v(1)+360)    
        ir=(b-x)/cos(radA) + e(1);
    else
        disp('fault...not all radar angles are covered!')
    end
else
    disp('fault...not a valid direction spesified!')
end

if (abs(ir)>255)
    Ir=255;
else
    Ir=abs(ir);
end
%------------end Tcross function----------- %