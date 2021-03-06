%-------corner modul---------%
function Ir=cornerC(x,y,theta,s,type,b,e)
ir=255;
MyX=x;
MyY=y;

a=(90+theta-s);
radA=pi*a/180;    
c=0;
R=sqrt((80-b)^2/2)+c;
ir=255;

% corner piece that points in North-West direction
if type=='nw'
    origoC=[60,-20]; % origo of circle
    xC=origoC(1);
    yC=origoC(2);
    ir=255;
    d=sqrt((MyX-xC)^2+(MyY-yC)^2)-R; % distance between robot and the nearest point of circle arc
    if d<=0,
        disp('feil! roboten er utenfor lovlig omr�de...');
    end
    phi=3*pi/2+atan((MyX-xC)/(yC-MyY));
    phiDegree=phi*180/pi;
    v=[];
    v(1)=atan((b-MyY)/(2*b-MyX));
    v(2)=atan(MyX/(b-MyY))+pi/2;
    v(3)=3*pi/2-atan(MyX/(b+MyY));
    %v(4)=3*pi/2+atan((b-MyX)/(MyY+b));
    %v(5)=3*pi/2+atan((b-MyX)/MyY);
    %v(6)=2*pi-atan(MyY/(2*b-MyX));
    v(4)=phi-asin(R/(R+d));
    v(5)=phi+asin(R/(R+d));
    
    v=v*180/pi;
    if (a>=v(1) && a<v(2)) || (a>=v(1)+360 && a<=450)
        ir=(b-MyY)/cos(pi/2-radA) + e(4);
    elseif ( a>=v(2) && a<=v(3) )
        ir= MyX/(cos(pi-radA)) +e(3);    
    elseif  (a>v(3) && a<v(4)) || (a>v(5) && a<v(1)+360) ||...
            (a>=-90 && a<v(4)-360) || ( a>v(5)-360 && a<v(1) )
        ir=255;
    elseif ( a>=v(4) && a<v(5)) || ( a>=v(4)-360 && a<=v(5)-360 )
        %sol=solve(strcat(num2str(R),'^2+',num2str(R+d),'^2-2*',num2str(R),'*',num2str(R+d),...
        %    '*cos(arcsin(a/',num2str(R),'*sin(',num2str(abs(phi-radA)),')))-a^2'))
        %for i=1:1:size(sol),
        %    if double(sol(i))>0 && imag(double(sol(i)))==0 && ir==255,
        %        ir=double(sol(i));
        %    end
        %end
        
        Belta = R^2 + (R+d)^2 -2*(R+d)^2*(sin(abs(phi-radA)))^2;
        Delta = sqrt(Belta^2 - (R^2 - (R+d)^2)^2);
        sol1=sqrt( Belta - Delta );
        sol2=sqrt( Belta + Delta );
        if imag(sol1)~=0 || imag(sol2)~=0,disp('error imag');end
        if sol1<sol2
            ir=sol1;
        else
            ir=sol2;
        end
        
    else    
        disp('warning...not a valid direction spesified!');
        disp(a);
    end

% corner piece that points in South-West direction
elseif type=='sw'
    origoC=[60,60]; % origo of circle
    xC=origoC(1);
    yC=origoC(2);
    ir=255;
    d=sqrt((x-xC)^2+(y-yC)^2)-R; % distance between robot and the nearest point of circle arc
    if d<=0,
        disp('feil! roboten er utenfor lovlig omr�de...');
    end
    phi=atan((yC-y)/(xC-x));
    
    v=[];
    v(1)=phi-asin(R/(R+d));
    v(2)=phi+asin(R/(R+d));
    %v(1)=atan((b-MyY)/(2*b-MyX));
    %v(2)=atan((b-MyY)/(b-MyX));
    %v(3)=pi/2-atan((b-MyX)/(2*b-MyY));
    v(4)=pi/2+atan(MyX/(2*b-MyY));
    v(5)=pi+atan(MyY/MyX);
    v(6)=2*pi-atan(MyY/(2*b-MyX));
    v=v*180/pi;
    if (a>=v(1) && a<v(2)) || (a>=v(1)+360 && a<=v(2)+360)
        %sol=solve(strcat(num2str(R),'^2+',num2str(R+d),'^2-2*',num2str(R),'*',num2str(R+d),...
        %    '*cos(arcsin(a/',num2str(R),'*sin(',num2str(abs(phi-radA)),')))-a^2'));
        %for i=1:1:size(sol),
        %    if double(sol(i))>0 && imag(double(sol(i)))==0 && ir==255,
        %        ir=double(sol(i));
        %    end
        %end
        Belta = R^2 + (R+d)^2 -2*(R+d)^2*(sin(abs(phi-radA)))^2;
        Delta = sqrt(Belta^2 - (R^2 - (R+d)^2)^2);
        sol1=sqrt( Belta - Delta );
        sol2=sqrt( Belta + Delta );
        if imag(sol1)~=0 || imag(sol2)~=0,disp('error imag');end
        if sol1<sol2
            ir=sol1;
        else
            ir=sol2;
        end
    
    elseif  (a>v(2) && a<v(4)) || ( a>v(2)+360 && a<450 ) || ( a>v(6)-360 && a<v(1) ) || ( a>v(6) && a<v(1)+360 )
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
elseif type=='se'
    origoC=[-20,60]; % origo of circle
    xC=origoC(1);
    yC=origoC(2);
    ir=255;
    d=sqrt((x-xC)^2+(y-yC)^2)-R; % distance between robot and the nearest point of circle arc
    if d<=0,
        disp('feil! roboten er utenfor lovlig omr�de...');
    end
    phi=3*pi/2+atan((x-xC)/(yC-y));

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
    elseif ( a>v(6) && a<=v(1)+360 ) || (a>=v(6)-360 && a<=v(1))  
        ir= (b-MyX)/cos(radA-2*pi) + e(1);
    else    
        disp('warning...not a valid direction spesified!');
        disp(a);
    end

% corner piece that points in North-East direction
elseif type=='ne'
    origoC=[-20,-20]; % origo of circle
    xC=origoC(1);
    yC=origoC(2);
    ir=255;
    d=sqrt((x-xC)^2+(y-yC)^2)-R; % distance between robot and the nearest point of circle arc
    if d<=0,
        disp('feil! roboten er utenfor lovlig omr�de...');
    end
    phi=3*pi/2+atan((x-xC)/(yC-y));
    
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
else
    disp('input not "nw", "ne", "se" or "sw"')
end

if (abs(ir)>255)
    Ir=255;
else
    Ir=abs(ir);
end
%----------end corner modul------------%