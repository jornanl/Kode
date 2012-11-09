%------------parallel2C ----------------- %
function Ir=parallel2C(x,y,theta,s,dir,b,e)

%myX=x;
%myY=y;

a=(90+theta-s);
radA=pi*a/180;

c=0;
R=sqrt((80-b)^2/2)+c;
ir=255;
 
%  _    _
%  _|  |_
% 
origoC1=[-20,20]; % origo of circle in local cell coordinates
xC1=origoC1(1);
yC1=origoC1(2);
d1=sqrt((x-xC1)^2+(y-yC1)^2)-R; % distance between robot and the nearest point of circle arc

origoC2=[60,20]; % origo of circle in local cell coordinates
xC2=origoC2(1);
yC2=origoC2(2);
d2=sqrt((x-xC2)^2+(y-yC2)^2)-R; % distance between robot and the nearest point of circle arc

if d1<0 || d2<0,
    disp('feil! roboten er utenfor lovlig område...');
end
phi1=pi+atan((y-yC1)/(x-xC1));
%phi1Degree=phi1*180/pi;
phi2=atan((yC2-y)/(x-xC2));

v=[];
v(1)=atan((b-y)/(b-x));
v(2)=phi1-asin(R/(R+d1));
v(3)=phi1+asin(R/(R+d1));
v(4)=2*pi-atan(y/(b-x));
v=v*180/pi;
if strcmp(dir,'ns'),    
    if (a>=-90 && a<v(4)-360) || ( a>v(3) && a<v(4)) || ( a>v(1) && a<v(2)) || (a>v(1)+360 && a<=450)
        ir=255;
    elseif ( a>=v(4)-360 && a<=v(1)) || ( a>=v(4) && a<=v(1)+360 ) 
        ir=(b-x)/cos(radA) + e(1);
    elseif ( a>=v(2) && a<=v(3) )
        %sol=solve(strcat(num2str(R),'^2+',num2str(R+d1),'^2-2*',num2str(R),'*',num2str(R+d1),...
        %    '*cos(arcsin(a/',num2str(R),'*sin(',num2str(abs(phi1-radA)),')))-a^2'));
        %for i=1:1:size(sol),
        %    if double(sol(i))>0 && imag(double(sol(i)))==0 && ir==255,
        %        ir=double(sol(i)) + e(3);
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
    else
        disp('fault...not all radar angles are covered!')
    end
%%%%%%%%%%

%%%%%%%%%%
elseif strcmp(dir,'we'),
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