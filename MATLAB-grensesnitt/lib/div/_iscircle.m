function result=_iscircle(X,Y,x,y);
% ISCIRCLE:% This program checks whether a point (x,y) lies 
% inside,outside or on a circle defined by 3 other points.
% 
% Syntax:::
% iscircle(X,Y,x,y)
% where,X=[x1 x2 x3]
%       Y=[y1 y2 y3]
%       
% Thus, a circle can be made out of these 3 points--
% (x1,y1),(x2,y2)&(x3,y3).
% Program checks whether point (x,y) lies inside,outside or on the circle.
% ans=0  ==> lie on the circle
% ans=1  ==> lie outside the circle
% ans=-1 ==> lie inside the circle

x1=X(1);y1=Y(1);
x2=X(2);y2=Y(2);
x3=X(3);y3=Y(3);

k=((x1-x2)*(x2*x2-x3*x3+y2*y2-y3*y3)-(x2-x3)*(x1*x1-x2*x2+y1*y1-y2*y2))/((2)*((y2-y3)*(x1-x2)-(y1-y2)*(x2-x3)));
h=((y1-y2)*(y1+y2-2*k))/((2)*(x1-x2))+(x1+x2)/2;
r=sqrt((x3-h)*(x3-h)+(y3-k)*(y3-k));

X=0;Y=0;
THETA=linspace(0,2*pi,1000);
RHO=ones(1,1000)*0.6;
[X,Y] = pol2cart(THETA,RHO);


figure(33),clf,hold on
plot(X,Y,'b',x,y,'r');
axis equal;


%val=(x-h)*(x-h)+(y-k)*(y-k)-r*r;



val=(x-h).*(x-h)+(y-k).*(y-k)-r*r;
result=sign(val);