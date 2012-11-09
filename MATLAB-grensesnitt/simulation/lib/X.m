function Ir=X(x,y,theta,s,b,e)

a=(90+theta-s);
radA=pi*a/180;

v=[];

v(1)=atan((b-y)/(2*b-x));
v(2)=atan((b-y)/(b-x));
v(3)=pi/2-atan((b-x)/(2*b-y));
v(4)=pi/2+atan(x/(2*b-y));
v(5)=pi/2+atan(x/(b-y));
v(6)=pi-atan((b-y)/(b+x));
v(7)=pi+atan(y/(b+x));
v(8)=pi+atan(y/x);
v(9)=3*pi/2-atan(x/(b+y));
v(10)=3*pi/2+atan((b-x)/(b+y));
v(11)=2*pi-atan(y/(b-x));
v(12)=2*pi-atan(y/(2*b-x));

v=v*180/pi;


if (a>=v(1) && a<=v(2)) || ( a>=v(1)+360 && a<=v(2)+360 )
    ir= (b-y)/sin(radA) + e(4);
elseif ( a>v(2) && a<=v(3) ) || ( a>v(2)+360 && a<=v(3)+360 )
    ir= (b-x)/sin(pi/2-radA) + e(1);
    
elseif ( a>=v(4) && a<v(5) ) 
    ir= x/sin(radA-pi/2) + e(3);
elseif  ( a>=v(5) && a<=v(6) )
    ir= (b-y)/sin(pi-radA) + e(4);
elseif ( a>=v(7) && a<v(8) ) 
    ir= y/sin(radA-pi) + e(2);
elseif ( a>=v(8) && a<=v(9) )   
    ir= x/sin(3*pi/2-radA) + e(3);

elseif ( a>=v(10) && a<v(11) ) || ( a>=v(10)-360 && a<v(11)-360 )   
    ir= (b-x)/sin(radA-3*pi/2) + e(1);
elseif ( a>=v(11) && a<v(12) ) || ( a>=v(11)-360 && a<v(12)-360 )   
    ir=y/sin(2*pi-radA) + e(2);
        
elseif (a>=-90 && a<v(10)-360) || (a>v(12)-360 && a<v(1)) || ( a>v(3) && a<v(4) )...
        || ( a>v(6) && a<v(7) ) || ( a>v(9) && a<v(10)) || (a>v(12) && a<v(1)+360) || (a>v(3)+360 && a<=450)
    ir=255;    

else
    disp('fault...not all radar angles are covered!')
end


if (abs(ir)>255)
    Ir=255;
else
    Ir=abs(ir);
end