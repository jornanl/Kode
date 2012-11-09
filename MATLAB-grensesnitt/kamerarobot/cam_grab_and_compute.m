function [lines I] = cam_grab_and_compute(cam_path, myHandles)
%henter ut frame fra webcam og gjør grått
I = imread(cam_path);
I = rgb2gray(I);

%edge detection ved hjelp av sobels metode

BW = edge(I,'sobel');
% BW = edge(I,'prewitt');
%BW = edge(I,'canny');
% bruker hougs metode for å finne sammenhengende rette linjer

[H,T,R] = hough(BW);

P  = houghpeaks(H,5);

lines = houghlines(BW,T,R,P,'FillGap',20,'MinLength',50)


figure(myHandles.kamerafigur);

imshow(I), hold on

   for k = 1:length(lines)
       if (lines(k).theta> -8 && lines(k).theta<8)
         xy = [lines(k).point1; lines(k).point2];

         %tegner inn kanter som grønne streker i bildet
         plot(xy(:,1),xy(:,2),'LineWidth',3,'Color','green');
       end
       
       if ((lines(k).theta> -60 && lines(k).theta<-20) || (lines(k).theta> 20 && lines(k).theta<20) )
            xy2 = [lines(k).point1; lines(k).point2];
            plot(xy2(:,1),xy2(:,2),'LineWidth',3,'Color','red');
       
       end
   end
   
   
