function ir=maze5(x,y,theta,s,b,e)

%---- concatenation of different map-pieces ---% 
if (x>=0 && x<2.5*b && y>=0 && y<1.5*b)
    cell='sw';
    ir=corners_Map5(x,y,theta,s,cell,b,e);
elseif (x>=0 && x<2.5*b && y>=1.5*b && y<=3*b)
    cell='nw';
    y=y-1.5*b;
    ir=corners_Map5(x,y,theta,s,cell,b,e);
elseif (x>=2.5*b && x<5*b && y>1.5*b && y<3*b)
    cell='ne';
    y=y-1.5*b;
    x=x-2.5*b;
    ir=corners_Map5(x,y,theta,s,cell,b,e);
elseif (x>=2.5*b && x<=5*b && y>=0 && y<1.5*b)
    cell='se';
    x=x-2.5*b;
    ir=corners_Map5(x,y,theta,s,cell,b,e);
else
disp('Error...robot is outside legal simulation area...!!');
end    
