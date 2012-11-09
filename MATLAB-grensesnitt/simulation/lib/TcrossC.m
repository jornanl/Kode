%------------start T-crossC modul------------%
function Ir=TcrossC(x,y,theta,s,dir,b,e)


a=(90+theta-s);
radA=pi*a/180; 
c=0;
R=sqrt((80-b)^2/2)+c;
ir=255;

% "north"
% _______
% __   __
%   | |
if strcmp(dir,'Tnorth'),
    origoC1=[-20,-20]; % origo of circle in local cell coordinates
    xC1=origoC1(1);
    yC1=origoC1(2);
    d1=sqrt((x-xC1)^2+(y-yC1)^2)-R; % distance between robot and the nearest point of circle arc

    origoC2=[60,-20]; % origo of circle in local cell coordinates
    xC2=origoC2(1);
    yC2=origoC2(2);
    d2=sqrt((x-xC2)^2+(y-yC2)^2)-R; % distance between robot and the nearest point of circle arc

    if d1<0 || d2<0,
        disp('feil! roboten er utenfor lovlig område...');
    end
    phi1=3*pi/2-atan((x-xC1)/(y-yC1));
    phi2=3*pi/2+atan((xC2-x)/(y-yC2));
    
    v=[];
    v(1)=atan((b-y)/(2*b-x));
    v(2)=pi-atan((b-y)/(b+x));
    %v(3)=pi+atan(y/(b+x));
    %v(4)=pi+atan(y/x);
    %v(5)=3*pi/2-atan(x/(b+y));
    v(3)=phi1-asin(R/(R+d2));
    v(4)=phi1+asin(R/(R+d2));
    
    v(6)=3*pi/2+atan((b-x)/(b+y));
    v(7)=2*pi-atan(y/(b-x));
    v(8)=2*pi-atan(y/(2*b-x));
    v=v*180/pi;
    if (a>=v(1) && a<=v(2)) || ( a>=v(1)+360 && a<=450 )
        ir= (b-y)/cos(pi/2-radA) + e(4);
    elseif (a>=v(4)-360 && a<v(6)-360) || (a>v(8)-360 && a<v(1)) || ( a>v(2) && a<v(3) )...
            || ( a>v(5) && a<v(6) ) || ( a>v(8) && a<v(1)+360)
        ir=255;
    elseif  ( a>=v(3) && a<v(4) )
        %sol=solve(strcat(num2str(R),'^2+',num2str(R+d1),'^2-2*',num2str(R),'*',num2str(R+d1),...
        %    '*cos(arcsin(a/',num2str(R),'*sin(',num2str(abs(phi1-radA)),')))-a^2'));
        %for i=1:1:size(sol),
        %    if double(sol(i))>0 && imag(double(sol(i)))==0 && ir==255,
        %        ir=double(sol(i)) + e(2);
        %    end
        %end
        Belta = R^2 + (R+d)^2 -2*(R+d)^2*(sin(abs(phi-radA)))^2;
        Delta = sqrt(Belta^2 - (R^2 - (R+d)^2)^2);
        sol1=sqrt( Belta - Delta );
        sol2=sqrt( Belta + Delta );
        if imag(sol1)~=0 || imag(sol2)~=0,
            disp('error imag');
        end
        if sol1<sol2
            ir=sol1;
        else
            ir=sol2;
        end
        if imag(sol1)~=0 || imag(sol2)~=0,disp('error imag');ir=255;end
    
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
elseif strcmp(dir,'Tsouth'),
    origoC1=[-20,60]; % origo of circle in local cell coordinates
    xC1=origoC1(1);
    yC1=origoC1(2);
    d1=sqrt((x-xC1)^2+(y-yC1)^2)-R; % distance between robot and the nearest point of circle arc

    origoC2=[60,60]; % origo of circle in local cell coordinates
    xC2=origoC2(1);
    yC2=origoC2(2);
    d2=sqrt((x-xC2)^2+(y-yC2)^2)-R; % distance between robot and the nearest point of circle arc

    if d1<0 || d2<0,
        disp('feil! roboten er utenfor lovlig område...');
    end
    phi1=pi-atan((yC1-y)/(x-xC1));
    phi2=atan((yC2-y)/(xC2-x));
    
    
    v=[];
    v(1)=atan((b-y)/(2*b-x));
    v(2)=atan((b-y)/(b-x));
    v(3)=pi/2-atan((b-x)/(2*b-y));
    v(4)=phi1-asin(R/(R+d1));
    v(5)=phi1+asin(R/(R+d1));
    v(6)=pi+atan(y/(b+x));
    v(7)=2*pi-atan(y/(2*b-x));
    v=v*180/pi;
    warning off;
    if (a>=-90 && a<v(7)-360) || ( a>=v(6) && a<=v(7) )
        ir= y/cos(3*pi/2-radA) + e(2);
        %
    elseif ( a>=v(2) && a<=v(3) ) || ( a>=v(2)+360 && a<v(3)+360)
        ir= (b-x)/sin(pi/2-radA) + e(1);
    elseif  ( a>v(1) && a<v(2) || ( a>=v(1)+360 && a<v(2)+360))
        ir= (b-y)/sin(radA);
        
    elseif ( a>=v(4) && a<v(5) )
        %sol=solve(strcat(num2str(R),'^2+',num2str(R+d1),'^2-2*',num2str(R),'*',num2str(R+d1),...
        %    '*cos(arcsin(a/',num2str(R),'*sin(',num2str(abs(phi1-radA)),')))-a^2'));
        %for i=1:1:size(sol),
        %    if double(sol(i))>0 && imag(double(sol(i)))==0 && ir==255,
        %        ir=double(sol(i)) + e(2);
        %    end
        %end
        Belta = R^2 + (R+d1)^2 -2*(R+d1)^2*(sin(abs(phi1-radA)))^2;
        Delta = sqrt(Belta^2 - (R^2 - (R+d1)^2)^2);
        sol1=sqrt( Belta - Delta );
        sol2=sqrt( Belta + Delta );
        if imag(sol1)~=0 || imag(sol2)~=0,
            disp('error imag');
        end
        if sol1<sol2
            ir=sol1;
        else
            ir=sol2;
        end
        if imag(sol1)~=0 || imag(sol2)~=0,disp('error imag');ir=255;end
      
    elseif (a>v(7)-360 && a<v(1)) || (a>v(7) && a<v(1)+360) || (a>v(3) && a<v(4)) ||...
            (a>v(3)+360 && a<v(4)+360) || (a>v(5) && a<v(6))    
        ir=255;
    else
        disp('fault...not all radar angles are covered!')
    warning on;
    end
elseif strcmp(dir,'T_west'),
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
elseif strcmp(dir,'T_east')
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