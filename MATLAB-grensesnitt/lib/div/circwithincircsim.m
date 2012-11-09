function circwithincircsim(n,demo)   %created 1/5/2006 lastupdated 2/5/06
%circwithincircsim A simulation to estimate the probability of a success
%                 for the following situation.
%
%                 In a circle of radius 1 centered at the origin randomly
%                 choose two point within the circle. Determine the
%                 probability that a circle centered at the midpoint
%                 of segment between the points with radius 
%                 (1/2) * distance between the points lies within the unit
%                 circle.
%
%    Use in the form   circwithincircsim(n,demo)
%                    where 
%                          n is the number of trials
%                          demo is any number
%                    in this case the result of the first 10 trials is
%                    shown graphically in the circle
%
%    OR  use in the form  circwithincircsim(n)
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
success=0; %initializing
numvalidtrials=0;dist = 2;
jj=numvalidtrials;
while numvalidtrials<n 
    if dist > 1
        x1=2;y1=2;
        distpoint1=sqrt(x1^2+y1^2);
        while distpoint1>=1
            x1=rnumh(-1,1,0);y1=rnumh(-1,1,0);
            distpoint1=sqrt(x1^2+y1^2);
        end
        x2=2;y2=2;
        distpoint2=sqrt(x2^2+y2^2);
        while distpoint2>=1
            x2=rnumh(-1,1,0);y2=rnumh(-1,1,0);
            distpoint2=sqrt(x2^2+y2^2);
        end
        dist=sqrt((x1-x2)^2+(y1-y2)^2);
    end
    %++++++++++++++++++++++++++++++++++++++++++++++
    %have pair of pts in unit circle so 
    %that length of segment < 1
    %so I have a valid trial
    numvalidtrials=numvalidtrials+1;
    jj=numvalidtrials;
       %compute midpoint of segment 
       xmpt=(x1+x2)/2;
       ympt=(y1+y2)/2;
       %compute dist from origin to midpoint
       middist=sqrt(xmpt^2+ympt^2);
       %compute dist from origin to midpt + (1/2) dist between point
       magicdist=middist+.5*dist;
        if magicdist<1
           success=success+1;
        end
        if demo==10 & jj<11
            pt1=plot(x1,y1,'*r');
            pt2=plot(x2,y2,'*r');
            seg=plot([x1 x2],[y1 y2],'r-');
           if magicdist<1
              mess=xlabel('SUCCESS','color','red');
           else
              mess=xlabel('FAILURE');
           end 
           %draw the circle with center at midpoint
           xcirc=xmpt+(.5*dist)*cx;
           ycirc=ympt+(.5*dist)*cy;
           newcirc=plot(xcirc,ycirc,'-g');
           count=title([num2str(success) ' SUCCESSES'],'color','red');
           pause(3)
           delete(pt1)
           delete(pt2)
           delete(seg)
           delete(mess)
           delete(newcirc)
           if jj<n,delete(count),end
           if jj==10,pause(3),close(gcf);end
        end
    dist=2; %resetting dist so that first if loop fails
            %thus we get a new pair of points in the unit circle
            %so that the distance between them is < 1
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