function ir=maze1(x,y,theta,s,b,e)

%---- concatenation of different map-pieces ---%  
if (x>=0 && x<b && y>=0 && y<b)
    cell='sw';
    ir=corner(x,y,theta,s,cell,b,e);
elseif (x>=0 && x<b && y>=b && y<=2*b)
    cell='w';
    y=y-b;
    ir=parallel1(x,y,theta,s,cell,b,e);
elseif ((x>=0 && x<b && y>2*b && y<3*b))
    cell='nw';
    y=y-2*b;
    ir=corner(x,y,theta,s,cell,b,e);
elseif (x>=b && x<=2*b && y>=2*b && y<3*b)
    cell='n';
    y=y-2*b;
    x=x-b;
    ir=parallel1(x,y,theta,s,cell,b,e);
elseif (x>=3*b && x<=4*b && y>=2*b && y<3*b)
    cell='n';
    y=y-2*b;
    x=x-3*b;
    ir=parallel1(x,y,theta,s,cell,b,e);
elseif (x>4*b && x<5*b && y>2*b && y<3*b)
    cell='ne';
    y=y-2*b;
    x=x-4*b;
    ir=corner(x,y,theta,s,cell,b,e);
elseif (x>=4*b && x<5*b && y>=b && y<=2*b)
    cell='e';
    y=y-b;
    x=x-4*b;
    ir=parallel1(x,y,theta,s,cell,b,e);
elseif (x>4*b && x<5*b && y>=0 && y<b)
    cell='se';
    x=x-4*b;
    ir=corner(x,y,theta,s,cell,b,e);
elseif (x>=b && x<=2*b && y>0 && y<b)
    cell='s';
    x=x-b;
    ir=parallel1(x,y,theta,s,cell,b,e);
elseif (x>=3*b && x<=4*b && y>0 && y<b)
    cell='s';
    x=x-3*b;
    ir=parallel1(x,y,theta,s,cell,b,e);
elseif (x>2*b && x<3*b && y>0 && y<b)
    cell='Tsouth';
    x=x-2*b;
    ir=Tcross(x,y,theta,s,cell,b,e);
elseif (x>2*b && x<3*b && y>2*b && y<3*b)
    cell='Tnorth';
    x=x-2*b;
    y=y-2*b;
    ir=Tcross(x,y,theta,s,cell,b,e);
elseif (x>2*b && x<3*b && y>=b && y<=2*b)
    cell='ns';
    x=x-2*b;
    y=y-b;
    ir=parallel2(x,y,theta,s,cell,b,e);
else
    disp('Error...robot is outside legal simulation area...!!');
end    
