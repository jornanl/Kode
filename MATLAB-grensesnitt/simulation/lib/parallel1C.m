function ir=parallel1C(x,y,theta,s,rot,b,e)
myX=x;
myY=y;

a=(90+theta-s); % a==relative angel of the IR sensor in degrees
radA=pi*a/180; % a in radians
c=0;
R=sqrt((80-b)^2/2)+c;
ir=255;

%  "north"
%  _______
%    ___
%   |   |
if rot=='n'    
    origoC=[20,-20]; % origo of circle in local cell coordinates
    xC=origoC(1);
    yC=origoC(2);

    d=sqrt((x-xC)^2+(y-yC)^2)-R; % distance between robot and the nearest point of circle arc
    if d<=0,
        disp('feil! roboten er utenfor lovlig omr�de...');
    end
    phi=3*pi/2+atan((xC-x)/(y-yC));
    
    v=[];
    v(1)=0;
    v(2)=pi;
    %v(3)=3*pi/2-atan(myX/myY);
    v(3)=phi-asin(R/(R+d));
    v(4)=phi+asin(R/(R+d));
    v=v*180/pi;
    if (a>=-90 && a<=v(4)-360) || ( a>=v(3) && a<=v(4))
        %sol=solve(strcat(num2str(R),'^2+',num2str(R+d),'^2-2*',num2str(R),'*',num2str(R+d),...
        %    '*cos(arcsin(a/',num2str(R),'*sin(',num2str(abs(phi-radA)),')))-a^2'));
        %for i=1:1:size(sol),
        %    if double(sol(i))>0 && imag(double(sol(i)))==0 && ir==255,
        %        ir=double(sol(i)) + e(2);
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
        if imag(sol1)~=0 || imag(sol2)~=0,disp('error imag');ir=255;end
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
elseif rot=='w'
    origoC=[60,20]; % origo of circle in local cell coordinates
    xC=origoC(1);
    yC=origoC(2);

    d=sqrt((x-xC)^2+(y-yC)^2)-R; % distance between robot and the nearest point of circle arc
    if d<=0,
        disp('feil! roboten er utenfor lovlig omr�de...');
    end
    phi=atan((yC-y)/(xC-x));
    if phi<0,
        phi=phi+2*pi;
    end
    
    v=[];
    v(1)=phi+asin(R/(R+d));
    v(2)=pi/2;
    v(3)=3*pi/2;
    v(4)=phi-asin(R/(R+d))+2*pi;
    v=v*180/pi;
    if (a>=-90 && a<v(4)-360) || (a>v(1) && a<= v(2)) || ( a>=v(3) && a<v(4) ) ...
            || ( a>v(1)+360 && a<=450 )
        ir=255;
    elseif (a>v(2) && a<v(3))
        ir=myX/cos(radA-pi) + e(3);
    elseif (a >= v(4)-360 && a<= v(1)) || (a >= v(4) && a<= v(1)+360)
        %sol=solve(strcat(num2str(R),'^2+',num2str(R+d),'^2-2*',num2str(R),'*',num2str(R+d),...
        %    '*cos(arcsin(a/',num2str(R),'*sin(',num2str(abs(phi-radA)),')))-a^2'));
        %for i=1:1:size(sol),
        %    if double(sol(i))>0 && imag(double(sol(i)))==0 && ir==255,
        %        ir=double(sol(i)) + e(1);
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
        if imag(sol1)~=0 || imag(sol2)~=0,disp('error imag');ir=255;end
    else
        disp('fault...not all radar angles are covered!')
    end
    
%  |_|
% _____
% "south"
elseif rot=='s'
    origoC=[20,60]; % origo of circle in local cell coordinates
    xC=origoC(1);
    yC=origoC(2);

    d=sqrt((x-xC)^2+(y-yC)^2)-R; % distance between robot and the nearest point of circle arc
    if d<=0,
        disp('feil! roboten er utenfor lovlig omr�de...');
    end
    phi=atan((yC-y)/(xC-x));
    if phi<0
        phi=phi+pi;
    end
    
    v=[];
    v(1)=pi;
    v(2)=0;
    v(3)=phi-asin(R/(R+d));
    v(4)=phi+asin(R/(R+d));  
    v=v*180/pi;
    if (a>=-90 && a<v(2)) || (a>v(1) && a< v(2)+360) 
        ir=myY/cos(radA-3*pi/2) + e(2);
    elseif (a>=v(2) && a<v(3)) || (a>v(4) && a<=v(1) ) || (a>=v(2)+360 && a<v(3)+360)
        ir=255;
    elseif (a >= v(3) && a<= v(4)) || (a >= v(3)+360 && a<= 450)
        %sol=solve(strcat(num2str(R),'^2+',num2str(R+d),'^2-2*',num2str(R),'*',num2str(R+d),...
        %    '*cos(arcsin(a/',num2str(R),'*sin(',num2str(abs(phi-radA)),')))-a^2'));
        %for i=1:1:size(sol),
        %    if double(sol(i))>0 && imag(double(sol(i)))==0 && (ir==255 || double(sol(i))< ir ),
        %        ir=double(sol(i)) + e(4);
        %    end
        %end;
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
        if imag(sol1)~=0 || imag(sol2)~=0,disp('error imag');ir=255;end
    else
        disp('fault...not all radar angles are covered!')
    end
    
%_  |
%_| | 'east'
%   |
elseif rot=='e'
    v=[];
    v(1)=3*pi/2;
    v(2)=pi/2;
    v(3)=pi-atan((b-myY)/myX);
    v(4)=pi+atan(myY/myX);
    v=v*180/pi;    
    if (a>v(1)-360 && a<v(2)) || (a>v(1) && a<=450)
        ir=(b-myX)/cos(radA) + e(1);
    elseif ( a>=v(2) && a<v(3))|| (a>v(4) && a<=v(1) )
        ir=255;
    elseif ( a>v(3) && a<v(4) )
        ir=(myX)/cos(pi-radA) + e(3);
    else
        disp('fault...not all radar angles are covered!')
    end
   
end
if (abs(ir)>255)
    Ir=255;
else
    Ir=abs(ir);
end
%------------end-passage-modul--------------%