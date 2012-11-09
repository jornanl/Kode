%-------corner modul---------%
function Ir=corner(x,y,theta,s,type,b,e)

MyX=x;
MyY=y;

a=(90+theta-s);
radA=pi*a/180;    

% corner piece that points in North-West direction
if strcmp(type,'nw')
    v=[];
    v(1)=atan((b-MyY)/(2*b-MyX));
    v(2)=atan(MyX/(b-MyY))+pi/2;
    v(3)=3*pi/2-atan(MyX/(b+MyY));
    v(4)=3*pi/2+atan((b-MyX)/(MyY+b));
    v(5)=3*pi/2+atan((b-MyX)/MyY);
    v(6)=2*pi-atan(MyY/(2*b-MyX));
    v=v*180/pi;
    if (a>=v(1) && a<v(2)) || (a>=v(1)+360 && a<=450)
        ir=(b-MyY)/cos(pi/2-radA) + e(4);
    elseif ( a>=v(2) && a<=v(3) )
        ir= MyX/(cos(pi-radA)) +e(3);    
    elseif  (a>v(3) && a<v(4)) || (a>v(6) && a<v(1)+360) ||...
            (a>=-90 && a<v(4)-360) || ( a>v(6)-360 && a<v(1) )
        ir=255;
    elseif ( a>=v(4) && a<v(5)) || ( a>=v(4)-360 && a<=v(5)-360 )
        ir= (b-MyX)/(sin(radA-3*pi/2)) + e(1);
    elseif ( a>=v(5) && a<=v(6)) || ( a>=v(5)-360 && a<=v(6)-360 )    
        ir= (MyY)/(sin(2*pi-radA)) +e(2);
    else    
        disp('warning...not a valid direction spesified!');
        disp(a);
    end

% corner piece that points in South-West direction
elseif strcmp(type,'sw')
    v=[];
    v(1)=atan((b-MyY)/(2*b-MyX));
    v(2)=atan((b-MyY)/(b-MyX));
    v(3)=pi/2-atan((b-MyX)/(2*b-MyY));
    v(4)=pi/2+atan(MyX/(2*b-MyY));
    v(5)=pi+atan(MyY/MyX);
    v(6)=2*pi-atan(MyY/(2*b-MyX));
    v=v*180/pi;
    if (a>=v(1) && a<v(2)) || (a>=v(1)+360 && a<=v(2)+360)
        ir= (b-MyY)/sin(radA) + e(4);
    elseif ( a>=v(2)+360 && a<=v(3)+360 ) || ( a>=v(2) && a<=v(3))
        ir= (b-MyX)/sin(pi/2-radA) + e(1);
    elseif  (a>v(3) && a<v(4)) || ( a>v(3)+360 && a<450 ) || ( a>v(6)-360 && a<v(1) ) || ( a>v(6) && a<v(1)+360 )
        ir=255;
    elseif ( a>=v(4) && a<v(5)) 
        ir= MyX/cos(radA-pi) + e(3);
    elseif ( a>=v(5) && a<=v(6)) || ( a>=-90 && a<=v(6)-360 )
        ir= MyY/cos(radA-3*pi/2) + e(2);
    else    
        disp('warning...not a valid direction spesified!');
        disp(a);
    end

% corner piece that points in South-East direction    
elseif strcmp(type,'se'),
    v=[];
    v(1)=pi/2-atan((b-MyX)/(2*b-MyY));
    v(2)=pi/2+atan(MyX/(2*b-MyY));
    v(3)=pi-atan((b-MyY)/MyX);
    v(4)=pi-atan((b-MyY)/(b+MyX));
    v(5)=pi+atan(MyY/(b+MyX));
    v(6)=2*pi-atan(MyY/(b-MyX));
    v=v*180/pi;
    if (a>v(1) && a<v(2)) || (a>v(1)+360 && a<=450) || (a>v(4) && a<v(5))
        ir= 255;
    elseif ( a>=v(2) && a<v(3) ) 
        ir= MyX/sin(radA-pi/2) + e(3);
    elseif  ( a>=v(3) && a<v(4) )
        ir= (b-MyY)/sin(pi-radA) + e(4);
    elseif ( a>=v(5) && a<v(6)) || (a>=-90 && a<v(6)-360) 
        ir= MyY/cos(radA-3*pi/2) + e(2);
    elseif ( a>=v(6) && a<=v(1)+360 ) || (a>=v(6)-360 && a<=v(1))  
        ir= (b-MyX)/cos(radA-2*pi) + e(1);
    else    
        disp('warning...not a valid direction spesified!');
        disp(a);
    end

% corner piece that points in North-East direction
elseif strcmp(type,'ne'),
    v=[];
    v(1)=atan((b-MyY)/(b-MyX));
    v(2)=pi-atan((b-MyY)/(b+MyX));
    v(3)=pi+atan(MyY/(b+MyX));
    v(4)=pi+atan(MyY/MyX); 
    v(5)=3*pi/2-atan(MyX/(b+MyY)); 
    v(6)=3*pi/2+atan((b-MyX)/(b+MyY));
    v=v*180/pi;
    if (a>=v(1) && a<=v(2)) || ( a>=v(1)+360 && a <= 450 )
        ir= (b-MyY)/cos(radA-pi/2) + e(4);
    elseif ( a>v(2) && a<v(3) ) || ( a>v(5) && a<v(6)) || ( a>=-90 && a<v(6)-360)
        ir=255;
    elseif ( a>=v(3) && a<v(4) ) 
        ir=MyY/sin(radA-pi) + e(2);
    elseif ( a>=v(4) && a<=v(5) ) 
        ir= MyX/sin(3*pi/2-radA) + e(3);
    elseif ( a>=v(6)-360 && a<v(1)) || (a>=v(6) && a<v(1)+360 )
        ir= (b-MyX)/cos(radA) + e(1);
    else    
        disp('warning...not a valid direction spesified!');
        disp(a);
    end
end
if (abs(ir)>255)
    Ir=255;
else
    Ir=abs(ir);
end
%----------end corner modul------------%