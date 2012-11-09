%-------corner modul---------%
function Ir=corners_Map5(x,y,theta,s,type,b,e)
ir=255;
myX=x;
myY=y;
% variable
R1=15;
R2=40;
b=40;


% forbehandling av data
a=(90+theta-s); % 0< a < 180 degrees
radA=pi*a/180;


% corner piece that points in North-West direction
switch type
case 'nw'
    origoC1=[100, 0];
    origoC2=[0,-60];
    xC1=origoC1(1);
    yC1=origoC1(2);
    xC2=origoC2(1);
    yC2=origoC2(2);

    d1=sqrt((xC1-myX)^2+(myY-yC1)^2)-R1; % distance between robot and the nearest point of circle arc
    d2=sqrt((myX-xC2)^2+(myY-yC2)^2)-R2; % distance between robot and the nearest point of circle arc
    
    if d1<=0 || d2<=0,
        disp('feil! roboten er utenfor lovlig område...');
    end
 
    phi1=3*pi/2+atan((xC1-myX)/(myY-yC1));
    phiDegree1=phi1*180/pi;
    phi2=3*pi/2-atan((myX-xC2)/(myY-yC2));
    phiDegree2=phi2*180/pi;
    
    % Angles
    v=[];
    v(1)=atan((1.5*b-myY)/(4*b-myX));
    v(2)=pi-atan((1.5*b-myY)/(myX));
    v(3)=pi+atan((myY-yC2-R2)/(myX));
    if myX>=R2,v(4)=3*pi/2-atan((myX-R2)/(myY-yC2));else v(4)=phi2+asin(R2/(R2+d2)); end
    v(5)=phi1-asin(R1/(R1+d1));
    v(6)=phi1+asin(R1/(R1+d1));
    
    v=v*180/pi;
    if (a>=v(1) && a<v(2)) || (a>=v(1)+360 && a<=450)
        ir=(1.5*b-myY)/cos(pi/2-radA) + e(4);
    elseif ( a>=v(2) && a<=v(3) )
        ir= myX/(cos(pi-radA)) + e(3);
    elseif  (a>v(3) && a<v(4)) 
        Belta2 = R2^2 + (R2+d2)^2 -2*(R2+d2)^2*(sin(abs(phi2-radA)))^2;
        Delta2 = sqrt(Belta2^2 - (R2^2 - (R2+d2)^2)^2);
        sol1=sqrt( Belta2 - Delta2 );
        sol2=sqrt( Belta2 + Delta2 );
        if imag(sol1)~=0 || imag(sol2)~=0,disp('error imag');end
        if sol1<sol2
            ir=sol1;
        else
            ir=sol2;
        end
        
    elseif ( a>v(4) && a<v(5)) || ( a>=-90 && a<v(5)-360 ) ||...
            (a>v(6)-360 && a<v(1)) || (a>v(6) && a<v(1)+360)  
        ir=255;
    elseif ( a>v(5) && a<v(6)) || ( a>v(5)-360 && a<v(6)-360 )
        Belta = R1^2 + (R1+d1)^2 -2*(R1+d1)^2*(sin(abs(phi1-radA)))^2;
        Delta = sqrt(Belta^2 - (R1^2 - (R1+d1)^2)^2);
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
        
    else    
        disp('warning...not a valid direction spesified!');
        disp(a);
    end
%
% corner piece that points in South-West direction
%
case 'sw'
    origoC1=[100, 60];
    origoC2=[0,0];    
    xC1=origoC1(1);
    yC1=origoC1(2);
    xC2=origoC2(1);
    yC2=origoC2(2);

    
    d1=sqrt((xC1-myX)^2+(yC1-myY)^2)-R1; % distance between robot and the nearest point of circle arc
    d2=sqrt((myX-xC2)^2+(myY-yC2)^2)-R2; % distance between robot and the nearest point of circle arc
    
    if d1<=0 || d2<=0,
        disp('feil! roboten er utenfor lovlig område...');
    end
 
    phi1=atan((yC1-myY)/(xC1-myX));
    phiDegree1=phi1*180/pi;
    phi2=3*pi/2-atan((myX-xC2)/(myY-yC2));
    phiDegree2=phi2*180/pi;
    
    % Angles
    v=[];
    %if 1.5*b-myY<R1 && 2.5*b-myX<R1,
        
    %    v(6)=phi1-asin(R1/(R1+d1))+360;
    %    if 3*pi/2+atan((3.5*b-myX)/(myY)) < v(6),
    %        v(5)=3*pi/2+atan((3.5*b-myX)/(myY));
    %    else
    %        v(5)=phi1-asin(R1/(R1+d1))+360;
    %        v(6)=3*pi/2+atan((3.5*b-myX)/(myY));    
    %    end
    %else
    %    
    %end
        
        
    v(1)=phi1-asin(R1/(R1+d1));
    v(2)=phi1+asin(R1/(R1+d1));
    %if v(1)<0,
    %    v(1)=v(1)+360;
    %end
    
    v(3)=pi/2+atan((myX)/(100-myY));    
    
    if myY>R2,v(4)=pi+atan((myY-R2)/(myX));else v(4)=phi2-asin(R2/(R2+d2)); end
    
    if myX>=R2,v(5)=3*pi/2-atan((myX-R2)/(myY-yC2));else v(5)=phi2+asin(R2/(R2+d2)); end
    
    v(6)=3*pi/2+atan((3.5*b-myX)/(myY));
    
    v=v*180/pi;
    if (a>=v(1) && a<v(2)) || (a>=v(1)+360 && a<=v(1)+360)
        Belta = R1^2 + (R1+d1)^2 -2*(R1+d1)^2*(sin(abs(phi1-radA)))^2;
        Delta = sqrt(Belta^2 - (R1^2 - (R1+d1)^2)^2);
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
        
    elseif ( a>=v(2) && a<=v(3) ) ||( a>=v(2)+360 && a<=450 ) ...
            || (a>v(6)-360 && a<v(1)) || (a>v(6) && a<v(1)+360)
        ir=255; 
        
        
    elseif  (a>v(3) && a<v(4)) 
        ir=myX/(cos(pi-radA)) + e(3);
    elseif ( a>v(5) && a<v(6)) || ( a>=-90 && a<v(6)-360 )
              
        ir=myY/(cos(3*pi/2-radA));
    elseif ( a>=v(4) && a<=v(5))
        Belta = R2^2 + (R2+d2)^2 -2*(R2+d2)^2*(sin(abs(phi2-radA)))^2;
        Delta = sqrt(Belta^2 - (R2^2 - (R2+d2)^2)^2);
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
        
    else    
        disp('warning...not a valid direction spesified!');
        disp(a);
    end
% corner piece that points in South-East direction    
case 'se'
    origoC1=[0,60]; % origo of circle
    xC1=origoC1(1);
    yC1=origoC1(2);
   
    
    d1=sqrt((myX-xC1)^2+(yC1-myY)^2)-R1; % distance between robot and the nearest point of circle arc
    if d1<=0,
        disp('feil! roboten er utenfor lovlig område...');
    end
    phi1=pi-atan((yC1-myY)/(myX-xC1));

    v=[];
    v(1)=atan((2.5*b-myY)/(2.5*b-myX));
    v(2)=phi1-asin(R1/(R1+d1));
    v(3)=phi1+asin(R1/(R1+d1));
    v(4)=3*pi/2-atan((b+myX)/(myY));
    v(5)=3*pi/2+atan((2.5*b-myX)/(myY));

    v=v*180/pi;
    if (a>v(1) && a<v(2)) || (a>v(1)+360 && a<=450) || (a>v(3) && a<v(4))
        ir= 255;
    elseif ( a>=v(2) && a<v(3) ) 
        Belta = R1^2 + (R1+d1)^2 -2*(R1+d1)^2*(sin(abs(phi1-radA)))^2;
        Delta = sqrt(Belta^2 - (R1^2 - (R1+d1)^2)^2);
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
    elseif ( a>=v(4) && a<v(5)) || (a>=-90 && a<v(5)-360)
        ir= myY/cos(3*pi/2-radA);
    elseif ( a>=v(5)-360 && a<v(1)) || (a>=v(5) && a<v(1)+360) 
        ir= (2.5*b-myX)/cos(radA);
    else    
        disp('warning...not a valid direction spesified!');
        disp(a);
    end

% corner piece that points in North-East direction
case 'ne'
    origoC=[0,0]; % origo of circle
    xC1=origoC(1);
    yC1=origoC(2);
    d1=sqrt((myX-xC1)^2+(myY-yC1)^2)-R1; % distance between robot and the nearest point of circle arc
    if d1<=0,
        disp('feil! roboten er utenfor lovlig område...');
    end
    phi1=pi+atan((myY-yC1)/(myX-xC1));
    
    
    v=[];
    v(1)=atan((1.5*b-myY)/(2.5*b-myX));
    v(2)=pi/2+atan((b+myX)/(1.5*b-myY));
    v(3)=phi1-asin(R1/(R1+d1));
    v(4)=phi1+asin(R1/(R1+d1));
    v(5)=2*pi-atan((b+myY)/(2.5*b-myX));

    v=v*180/pi;
    if (a>v(2) && a<v(3)) || (a>v(4) && a<v(5)) || (a>-90 && a<v(5)-360)
        ir= 255;
    elseif ( a>=v(3) && a<v(4) ) 
        Belta = R1^2 + (R1+d1)^2 -2*(R1+d1)^2*(sin(abs(phi1-radA)))^2;
        Delta = sqrt(Belta^2 - (R1^2 - (R1+d1)^2)^2);
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
    elseif ( a>=v(5) && a<v(1)+360) || (a>=v(5)-360 && a<v(1))
        ir= (2.5*b-myX)/cos(radA);
    elseif ( a>=v(1) && a<v(2)) || (a>=v(1)+360 && a<=450) 
        ir= (1.5*b-myY)/cos(pi/2-radA);
    else    
        disp('warning...not a valid direction spesified!');
        disp(a);
    end
otherwise
    disp('input not "nw", "ne", "se" or "sw"')
end

if (abs(ir)>255)
    Ir=255;
else
    Ir=abs(ir);
end
%----------end corner modul------------%