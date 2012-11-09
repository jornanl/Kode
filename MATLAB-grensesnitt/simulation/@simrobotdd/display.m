%DISPLAY Display method for differential drive simrobot object.
%   DISPLAY(S) displays the differential drive simrobot object S.
%
% See also ROBOTDD.


function display(s)

display(s.robot);
if size(s.x) == [1 3], x = s.x'; else x = s.x; end;
xr = num2str(x);
Cr = num2str(s.C);
disp(sprintf('  x = %s       C = %s', xr(1,:), Cr(1,:)));
disp(sprintf('      %s           %s', xr(2,:), Cr(2,:)));
disp(sprintf('      %s           %s', xr(3,:), Cr(3,:)));
disp(sprintf('  rl = %s', num2str(s.rl)));
disp(sprintf('  rr = %s', num2str(s.rr)));
disp(sprintf('  b  = %s', num2str(s.b)));