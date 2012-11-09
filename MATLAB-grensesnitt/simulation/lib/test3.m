function test3(type)


h1=figure('Numbertitle','off','Name','testing av simuleringsblokker...');clf;hold on;
for theta=0:4:360,
    plot(0,0,'b+');
    ir=corners_Map5(10,20,theta,90,type,40,[0,0,0,0]);
    theta;
    if ir~=255,
        [x y]=pol2cart(theta*pi/180,ir);
        plot(x,y,'rx');
    end
end
h2=figure('Numbertitle','off','Name','testing av simuleringsblokker...');clf;hold on;
for theta=0:4:360,
    plot(0,0,'b+');
    ir=corners_Map5(05,40,theta,90,type,40,[0,0,0,0]);
    
    if ir~=255,
        [x y]=pol2cart(theta*pi/180,ir);
        plot(x,y,'rx');
    end
end
h3=figure('Numbertitle','off','Name','testing av simuleringsblokker...');clf;hold on;
for theta=0:4:360,
    plot(0,0,'b+');
   ir=corners_Map5(05,50,theta,90,type,40,[0,0,0,0]);
   
   if ir~=255
       [x y]=pol2cart(theta*pi/180,ir);
       plot(x,y,'rx');
   end
end
pause();
close(h1);
close(h2);
close(h3);
 