function [trueFalse intersectionPointX intersectionPointY]=vectorCrossingChecker(vectA,vectB)
%this function desides if vectA intersects vectB
% return value -1 = error
% return value  1 = they intersect
% return value  2 = they don't intersect


%debug    
%    vectA = [rand(1)*10 rand(1)*10 rand(1)*10 rand(1)*10];
%    vectB = [rand(1)*10 rand(1)*10 rand(1)*10 rand(1)*10]; 
%     figure(10);clf;hold on;
%     plot( [vectA(1) vectA(3)],[vectA(2) vectA(4) ],'g','LineWidth',2.7);
%     plot( vectA(1),vectA(2),'kx','LineWidth',2,'MarkerSize',9);
%     plot( vectA(3),vectA(4),'kx','LineWidth',2,'MarkerSize',9);                    
%     plot( [vectB(1) vectB(3)],[vectB(2) vectB(4) ],'g','LineWidth',2.7);
%     plot( vectB(1),vectB(2),'kx','LineWidth',2,'MarkerSize',9);
%     plot( vectB(3),vectB(4),'kx','LineWidth',2,'MarkerSize',9);                    
%debug


intersectionPointX=0;
intersectionPointY=0;
trueFalse=2;
    
    
if( numel(vectA)~=4 || size(vectA,2)~=4 || numel(vectB)~=4 || size(vectB,2)~=4 )
    warning 'wrong input parameter to vectorCrossingChecker.m';
    trueFalse=-1;
    return;
end;
    
if( vectA(1)==vectA(3) && vectA(2)==vectA(4) ) %% null vector
    trueFalse=2;
    return;
end;
if( vectB(1)==vectB(3) && vectB(2)==vectB(4) ) %% null vector
    trueFalse=2;
    return;
end;

    
warning off;
axDnum = vectA(1)-vectA(3);
ayNum  = vectA(2)-vectA(4);
deltaALine = ayNum/axDnum;
 
bxDnum = vectB(1)-vectB(3);
bxNum  = vectB(2)-vectB(4); 
deltaBLine = bxNum/bxDnum;
warning on;

intersectionPointX=[];
intersectionPointY=[];
if(abs(deltaALine) == Inf && abs(deltaBLine) ==Inf )
    %parallell
elseif(deltaALine==deltaBLine)
    %parallell
elseif(abs(deltaALine) == Inf && abs(deltaBLine) ~= Inf )
    intersectionPointX=vectA(1);
    intersectionPointY=deltaBLine*(intersectionPointX-vectB(1))+vectB(2);
elseif(abs(deltaALine) ~= Inf && abs(deltaBLine) == Inf )
    intersectionPointX=vectB(1);
    intersectionPointY=deltaALine*(intersectionPointX-vectA(1))+vectA(2);
elseif(abs(deltaALine) ~=Inf && abs(deltaBLine) ~=Inf )
    intersectionPointX = (-deltaBLine*vectB(1)+vectB(2)+deltaALine*vectA(1)-vectA(2))/(deltaALine-deltaBLine);
    intersectionPointY = deltaALine*(intersectionPointX-vectA(1))+vectA(2); 
end;


vect=[vectA;vectB];
inside=[0,0];

if (size(intersectionPointX,1)> 0 && size(intersectionPointY,1)>0),
    for i=1:2,
        v=vect(i,:);
        if (v(1) <= v(3) && (intersectionPointX >= v(1) && intersectionPointX <= v(3))) ||...
                (v(3) <= v(1) && (intersectionPointX >= v(3) && intersectionPointX <= v(1))) ||...
                v(1)==v(3),
            if (v(2) < v(4)),                
                if intersectionPointY >= v(2) && intersectionPointY <= v(4),
                    inside(i)=1;
                end
            elseif (v(4) < v(2) && (intersectionPointY >= v(4) && intersectionPointY <= v(2))),
                inside(i)=1;
            elseif v(4)==v(2),
                inside(i)=1;
            end
         
        end
    end
end
if inside(1) && inside(2),
    trueFalse=1;
end
