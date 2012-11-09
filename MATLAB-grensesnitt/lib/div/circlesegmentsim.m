function csegsim(n,demo)   %created 1/5/2006
%criclesegmentsim A simulation to estimate the probability of a success
%                 for the following situation.
%
%                 In a circle of radius 1 centered at the origin randomly
%                 choose two point within the circle. Determine the
%                 probability that the distance between the pair of points
%                 is less than 1.
%
%    Use in the form   circlesegmentsim(n,demo)
%                    where 
%                          n is the number of trials
%                          demo is any number
%                    in this case the result of the first 10 trials is
%                    shown graphically in the circle
%
%    OR  use in the form  circlesegmentsim(n)
%                    here no graphics are displayed
%
%    OUTPUT: in either case shows number of successes, number of trials,
%            and their ratio
%
%       By: David R. Hill, Math Dept, Temple University,
%           Philadelphia, Pa. 19122     Email: dhill001@temple.edu

if nargin==1,demo=0;end %no graphics mode
if nargin==2,demo=10;end %show first 10 trial grahically
if n<=0 | fix(n)~=n
    disp('ERROR: The number of trials must be a positive integer.')
    return
end
rand('state',sum(100*clock)) %seeding random number generator
success=0; %initializing
t=0:.01:2*pi+.1; %Setting up the unit circle
cx=cos(t);
cy=sin(t);
if demo==10;
    figure('color','white')
    plot(cx,cy,'k-','erasemode','none')
    axis(axis)
    axis('square')
    hold on,gca;plot(xlim,[0 0],'-k',[0 0],ylim,'-k','linewidth',2);hold off
    hold on,gca;plot(xlim,[0 0],'-k',[0 0],ylim,'-k','linewidth',2);hold off
    hold on
end
%the trial loop
for jj=1:n
    x1=2;y1=2;
    distpoint1=x1^2+y1^2;
    while distpoint1>=1
       x1=rnumh(-1,1,0);y1=rnumh(-1,1,0);
      distpoint1=x1^2+y1^2;
    end
    if demo==10 & jj<11
        pt1=plot(x1,y1,'*r');
    end    
    x2=2;y2=2;
    distpoint2=x2^2+y2^2;
    while distpoint2>=1
       x2=rnumh(-1,1,0);y2=rnumh(-1,1,0);
       distpoint2=x2^2+y2^2;
    end
    if demo==10 & jj<11
        pt2=plot(x2,y2,'*r');
        seg=plot([x1 x2],[y1 y2],'r-');
    end    
    dist=(x1-x2)^2+(y1-y2)^2;
    if dist<1,success=success+1;end
    if demo==10 & jj<11
        if dist<1
            mess=xlabel('SUCCESS','color','red');
        else
            mess=xlabel('FAILURE');
        end    
    end
    if demo==10 & jj<11
        count=title([num2str(success) ' SUCCESSES'],'color','red');
        pause(3)
        delete(pt1)
        delete(pt2)
        delete(seg)
        delete(mess)
        if jj<n,delete(count),end
        if jj==10,pause(3),close(gcf);end
    end
end
disp(['Number of successes = ' num2str(success)])
disp(['Number of trials = ' num2str(n)])
disp(['RATIO = ' num2str(success/n)])
function rval=rnumh(k,j,state)     %<<last updated 5/31/96>>
%RNUMH  Generate a uniformly distributed random number between k
%       and j. If there are only two arguments the number is to
%       be an integer. If there is a third argument, then a real
%       value is returned.
%
%   Use in the form  ==>  rnumh(k,j)  or  rnumh(k,j,0)  <==
%
%       By: David R. Hill, Math Dept, Temple University,
%           Philadelphia, Pa. 19122     Email: dhill001@temple.edu
if nargin==1
   disp('Only 1 argument; ERROR.')
   return
end
%rand
if nargin==2
   st='int';
else
   st='rea';
end
if k<j
   xsm=k;xbig=j;
else
   xsm=j;xbig=k;
end
if st=='int',xbig=xbig +1;end  %the next statement never gives 1
x=rand;                       % thus we view this in terms of
rval=xsm*(1-x)+xbig*x;        % subintervals from k to j+1 each
if st=='int'                  % each of length 1
   if xsm==0 & xbig==1
      if rval<.5,rval=0;else,rval=1;end
   else
      rval=fix(xsm*(1-x)+(xbig)*x);
   end
   if rval>xbig,rval=xbig;end
end