clear;
clc;
v1 = load('dataset1.txt');
a = v1(1);
ranges = v1(2:a+1);
voltages = v1(a+2:a+a+1);

plot(voltages,ranges,'.');

%fit curve, piecewise
[P,S] = polyfit(voltages, ranges,10);

x = 350:1:1750;
yfit = polyval(P,x);

%Now compare y with yfit
hold on
plot(x,yfit,'k');

yfit = yfit';

x2 = 800:1:1750;
[P2,S2] = polyfit(voltages(1:19), ranges(1:19),9);
yfit2 = polyval(P2,x2);
plot(x2,yfit2,'r');

x3 = 350:1:800;
[P3,S3] = polyfit(voltages(19:72), ranges(19:72),10);
yfit3 = polyval(P3,x3);
plot(x3,yfit3,'b');

%making lookup table with 1401 entries for sensordata >= 350 && <=1750

yfit_full = [yfit3(1:450) yfit2];

yfit_full = yfit_full';

rangeLookupTable = dataset(yfit_full);
export(rangeLookupTable,'WriteVarNames',false);

hold off

 
 
 
 
    
     
 
 

 
 
 
 
