% Circles_Intersection - function for calculation the region of the overlapping
% of the collection of circles, result is presented in vector format 
% 
% ci_example  - test of function Circles_Intersection with GUI
%  
% arcs2region - converter for transformation result of the execution
%               function Circles_Intersection in format aproximating 
%               set polygonal figure
% reg_patch - function for visualization result 
%
% auxiliary functions:
% InsCompToRegn
% InsLineToComp
% 
% files with test data:
% d1.mat, d2.mat, d3err.mat, d16.mat, d50err.mat
%
%
% example:
% c=Circles_Intersection([0 40 35; 30 -20 35; -30 -20 35]);
% f=arcs2region(c,1,1);
% h=reg_patch(f);
%
% example:
% c=Circles_Intersection([0 40 35; 30 -20 35; -30 -20 35]);
% 
% result:
% 
% c = struct with fields:
%      G: [3x3 double]    
%      P: [12x2 double]
%     Ng: 3
%     Np: 12
%      K: 2
%      w: [1x2 struct]
% 
% c.G =
%      0    40    35
%     30   -20    35
%    -30   -20    35
% 
% c.P =
%    35.0000   40.0000
%   -35.0000   40.0000
%    65.0000  -20.0000
%    -5.0000  -20.0000
%     5.0000  -20.0000
%   -65.0000  -20.0000
%     6.0557    5.5279
%    23.9443   14.4721
%   -23.9443   14.4721
%    -6.0557    5.5279
%          0   -1.9722
%    -0.0000  -38.0278
% 
% c.w = 1x2 struct array with fields:
%     sk
%     S
%     nk
% 
% c.w(1) =  (region of 1 and more circles overlapped)          c.w(2) = (region of 2 and more circles overlapped)
%     sk: [1x2 struct]                                             sk: [1x3 struct]
%      S: 1.1223e+004            (area of this region)         S: 322.1710         (area of this region)
%     nk: 2  (number of closed curves in border of it region)  nk: 3     (  =length(sk)  )
% 
% c.w(1).sk = 1x2 struct array with fields:                    c.w(2).sk = 1x3 struct array with fields:
%     data                                                         data
%     S                                                            S
% 
% if S<0, it is inner border, if s>0 - it is outer border
%
% c.w(1).sk(1) =                                               c.w(2).sk(1) =
%     data: [4x4 double]                                           data: [3x4 double]
%        S: -36.8600                                                  S: 39.0713
% 
% c.w(1).sk(1).data =                                          c.w(2).sk(1).data =
%     1.0000   10.0000    4.5385    4.8863                         1.0000    9.0000    3.9590    4.5385
%     2.0000    7.0000    2.3242    2.6005                         3.0000   10.0000    0.8174    1.3969
%     3.0000   11.0000    0.5411    0.8174                         1.0000    9.0000    3.9590    4.5385
%     1.0000   10.0000    4.5385    4.8863
% 
% c.w(1).sk(2) =                                               c.w(2).sk(2) = 
%     data: [8x4 double]                                           data: [3x4 double]
%        S: 1.1260e+004                                               S: 39.0713
% 
% c.w(1).sk(2).data =                                          c.w(2).sk(2).data =
%     1.0000    8.0000    5.4658         0                         1.0000    7.0000    4.8863    5.4658
%     1.0000    1.0000         0    3.1416                         2.0000    8.0000    1.7447    2.3242
%     1.0000    2.0000    3.1416    3.9590                         1.0000    7.0000    4.8863    5.4658
%     3.0000    9.0000    1.3969    3.1416
%     3.0000    6.0000    3.1416    5.7421
%     2.0000   12.0000    3.6827         0
%     2.0000    3.0000         0    1.7447
%     1.0000    8.0000    5.4658         0
% 
%                                                              c.w(2).sk(3) = 
%                                                                  data: [5x4 double]
%                                                                     S: 244.0285
% 
%                                                              c.w(2).sk(3).data =
%                                                                  2.0000   11.0000    2.6005    3.1416
%                                                                  2.0000    4.0000    3.1416    3.6827
%                                                                  3.0000   12.0000    5.7421         0
%                                                                  3.0000    5.0000         0    0.5411
%                                                                  2.0000   11.0000    2.6005    3.1416
%    c.w(2).sk(3).data contains parameters of the set of the arcs     
%    c.w(2).sk(3).data(i,:) contains [ NumCirc  NumPoint   BeginAngle   EndAngle]
%    NumCirc    -  number of the circle in array G 
%    NumPoint   -  number of the point in array P
%    BeginAngle -  value of the angle, under which begins arc
%    EndAngle   -  value of the angle, under which ends arc
%                (point, in which begins current arc is a point, in which finishes
%                 previous, for first arc previous - a last arc)
%
%
%   Author:  Alexander Vakulenko
%   e-mail:  dspt@yandex.ru
%   Last modified: 20050207
%   
