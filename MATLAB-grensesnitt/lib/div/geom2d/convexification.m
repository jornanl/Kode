function co = convexification(varargin)
%CONVEXIFICATION compute convexification of a polygon
%
%   CO = CONVEXIFICATION(H)
%   Create convexification from support function. Suuport function is
%   supposed to be uniformly distributed over [0 2pi].
%
%   CO = CONVEXIFICATION(POLYGON)
%   Compute support function of the polygon, then the corresponding
%   convexification.
%
%   CO = CONVEXIFICATION(POLYGON, N)
%   Uses N points for convexification computation. Note that the number of
%   points of CO can be lower than N.
%   
%   CAUTION : The result will be valid only for convex polygons.
%
%
%   ---------
%
%   author : David Legland 
%   INRA - TPV URPOI - BIA IMASTE
%   created the 12/01/2005.
%

if length(varargin)>0
    var = varargin{1};
    if size(var, 2)==1
        h = var;
    else
        poly = var;
        N = 128;
        if length(varargin)>1
            N = varargin{2};
        end
        h = supportFunction(poly, N);
    end
else
    error('not enough input arguments');
end

N = length(h);
u = (0:2*pi/N:2*pi*(1-1/N))';
v = [cos(u) sin(u)].*[h h];

i1 = 1:N;
i2 = [2:N 1];
i3 = [3:N 1 2];

for i=1:N
    circ(i, 1:4) = createDirectedCircle(v(i1(i),:), v(i2(i),:), v(i3(i), :));
end

% remove non direct-oriented circles
circ = circ(circ(:,4)==0, :);

% keep only circles seen several times
dp = diff(circ(:,1:2));
dp = sum(dp.*dp, 2);
ind1 = [1; find(dp<1e-10)+1];
circ = circ(ind1, :);

% keep only one instance of each circle
dp = diff(circ(:,1:2));
dp = sum(dp.*dp, 2);
ind = [1; find(dp>1e-10)+1];
co = 2*circ(ind, 1:2);

% eventually remove the last point if it is the same as the first one
if distancePoints(co(1,:), co(end, :))<1e-10 && size(co, 1)>1
    co = co(1:end-1,:);
end


