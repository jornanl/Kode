function X=circleOnAPlane(P1,P2,P3)
% e.g.
% P1 = [1 2 3];
% P2 = [2 1 1];
% P3 = [3 3 2];

% X = center of the circle

 N = cross(P1-P2,P1-P3);
 N = N/norm(N);

 A = [2*(P2-P1);2*(P3-P1);N];
 b = [sum(P2.^2 - P1.^2);sum(P3.^2-P1.^2);P1*N'];

 X = (A\b)';
 
 