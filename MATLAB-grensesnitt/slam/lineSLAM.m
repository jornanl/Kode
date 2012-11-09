%% SLAM which produces a global map based on lines

function lineSLAM(myCon,myHandles,myLineParams)
global ROBOTPOSE;
global REALPOSE;
global RUNNING;

REALPOSE=[0 0 0];

myLineParams.robot.class = 'robotdd';

dispmode = myLineParams.displayGlobal;
display = myLineParams.displayLocal;
displaying= myLineParams.displayMatch;

% ----- Slam ----- %
% optional axis vector for global map figure. Useful with infinite lines
myLineParams.axisvec = [-2 2 -2 2];

G = initmap(0,'global map',myLineParams);
myLineParams.robot.xsensor= [0 0 0];
IRdata.params.xs=myLineParams.robot.xsensor;
IRdata.params.stdrho = myLineParams.sensor.stdrho;


istep = 0;

navData.robotPath=[];
navData.state=1; 
navData.backTrackPoint=[];
navData.STARTbackTrackPoint=[];
navData.backTrackPointAtGap=[];
navData.allBackTrackPoints=0;
navData.backTrackPointAtGap=[];
navData.lastHeading=0;

scndata=[];
fullscan=1;
aL_total=0;
aR_total=0;
oldPose=[0 0 0];
clc;

 %figure('KeyPressFcn',@pressedKey);

 %function pressedKey(src,evnt)
     %global RUNNING;  
     %if (evnt.Character == 'p') || (evnt.Character == 's'),
    %      RUNNING=2;
    %elseif length(evnt.Modifier) == 1 && strcmp(evnt.Modifier{:},'control') && evnt.Key == 'b'
    %     RUNNING=0;
    %end
 %end%% main while loop for mapping

while(RUNNING)
    
    pause(0.01);
    if RUNNING==2,
        while ~(RUNNING==1 || RUNNING==0)
            pause(0.1);
        end
        if ~RUNNING,
            break;
        end
    end
    
  
    drawnow;%keep the GUI active    
    
    set(G,'time',istep);
    Gin = G;
    
    [myNewpose,error]=getRobotPose(myCon,myHandles);
    if(error)
        disp('error fetching robot pose');
        continue;
    end;
    
    % display current position in Gui
    set(myHandles.xPosText,'String',num2str(myNewpose(1)) );
    set(myHandles.yPosText,'String',num2str(myNewpose(2)) );
    set(myHandles.tPosText,'String',num2str(myNewpose(3)*180/pi) );    
    if strcmp(myCon,'simulator'),
        set(myHandles.edit_realposeX,'String',num2str(REALPOSE(1)*10 ));
        set(myHandles.edit_realposeY,'String',num2str(REALPOSE(2)*10 ));
        set(myHandles.edit_realposeTheta,'String',num2str(myNewpose(3)*180/pi )); 
    end
    myNewpose=[myNewpose(1)/1000 myNewpose(2)/1000 myNewpose(3)];
    
    utover= get(Gin,'x');
    utover{1,1}=set( utover{1,1},'x', myNewpose' );
    Gin=set(Gin,'x',utover);
    
    % --------------------
    % --- State prediction
    % --------------------
    [aL,aR]=getAngularDispl(myNewpose,oldPose, 0.04,0.04,0.2);
    aL_total=aL_total+aL/500;
    aR_total=aR_total+aR/500;
    
    enc.params.kr=0.02;
    enc.params.kl=enc.params.kr;
    enc.steps(istep+1).data1=aL_total;
    enc.steps(istep+1).data2=aR_total;
    %enc.steps(istep+1).data1=istep/100;
    %enc.steps(istep+1).data2=istep/100;
   % ENC.PARAMS.KL: error growth coefficient of left wheel
%            ENC.PARAMS.KR: error growth coefficient of right wheel
%               with unit in [1/m]. 
%            ENC.STEPS(i).DATA1: angular displacements of left wheel
%            ENC.STEPS(i).DATA2: angular displacements of right wheel
%               in [rad] and monotonically increasing
    
        % Get the robot object
        r = getrobot(Gin);
        
        % Given the robot at its initial pose and the encoder data in sensors(i)
        % the predict method performs the state prediction of the robot. It re-
        % turns the new pose, its covariance matrix, the Jacobian of the process
        % model derived with respect to the robot and a 3xn matrix PATH of all
        % poses between xr and xrout
        [r,Frout,path] = predict(r,enc);
      
        % Predict the map given the predicted robot and the Jacobian FROUT
        Gin = setrobot(Gin,r);
        
        % Predict the map given the predicted robot and the Jacobian FROUT
        Gin = robotdisplacement(Gin,get(r,'x'),get(r,'C'),Frout);
    
    
    
    

    IRdata.steps.data=[];
% -----performing a full scan-------- %
    if fullscan,
        %set robot pose in map

        
    
        [error,data]=fullScan(myCon,myHandles);
        data(:,2:3)=data(:,2:3)/1000; % converting from mm to meter
        
        %figure(66);clf;hold on;axis equal;
        %plot(data(:,2), data(:,3),'yx');
        
        % rotate the global x,y points received from robot to local x,y points
        % in order to use them in extractlines
        rotmat=[cos(myNewpose(3)) sin(myNewpose(3)); -sin(myNewpose(3)) cos(myNewpose(3)) ];
        
        data(:,2)=data(:,2)-myNewpose(1);
        data(:,3)=data(:,3)-myNewpose(2);
        
        data(:,2:3)=(rotmat*(data(:,2:3))')';
        
        %data(find(-0.1<data(:,3) & data(:,3)<0.1),:)=[];

        IRdata.steps.data=[];
        NS=4;
        for sens=1:1:NS,
            tmpData=data(data(:,1)==sens,:);
        
            IRdata.steps.data=vertcat(IRdata.steps.data,tmpData);
        end
        %plot(IRdata.steps.data(:,2), IRdata.steps.data(:,3),'mx');
        
        
        
        
        
% ------- processing data from driving------- %   
    elseif numel(scndata)>0, % there are data collected during driving which need to be considered
        myNewpose(1:2)=oldPose(1:2);
        
    
        utover= get(Gin,'x');
        utover{1,1}=set( utover{1,1},'x', myNewpose' );
        Gin=set(Gin,'x',utover);
    
        
        %[poseAfterDriving,error]=getRobotPose(myCon,myHandles);
        %get new heading 
%        poseAfterDriving=[poseAfterDriving(1)/1000 poseAfterDriving(2)/1000 poseAfterDriving(3)]; 
        
    
        % rotate the global x,y points received from robot to local x,y points
        % in order to use them in extractlines
        rotmat=[cos(myNewpose(3)) sin(myNewpose(3));-sin(myNewpose(3)) cos(myNewpose(3)) ];
        
        scndata(:,2)=scndata(:,2)-myNewpose(1);
        scndata(:,3)=scndata(:,3)-myNewpose(2);
        scndata(:,2:3)=(rotmat*(scndata(:,2:3))')';
        
        scndata(find(-0.1<scndata(:,3) & scndata(:,3)<0.1),:)=[];
        for sens=2:2:5,
        
            tmpData=scndata(scndata(:,1)==sens,:);
            % artifical scandata..
            [artifx artify]=pol2cart((-1:0.2:1)*pi/8 ,0.02);
            artif=zeros(length(artifx),3);
            artif(:,2:3)=horzcat(artifx',artify');
            tmpData=vertcat(tmpData,artif);
        
            IRdata.steps.data=vertcat(IRdata.steps.data,tmpData);
        end
        

    
    end %fullscan
    
    %plotting raw data
%    xR = IRdata.steps.data2;
%    yR = IRdata.steps.data3;
%    [phi,rho] = cart2pol(xR,yR);
%    scan = [phi, rho];
    
%    figure(3); clf; hold on; set(gca,'Box','on'); axis equal;
%    plot(scan(:,2).*cos(scan(:,1)),scan(:,2).*sin(scan(:,1)),...
%          'b.',...%'Color',[.6 .6 .6],
%          'MarkerSize',6);
%    drawreference(zeros(3,1),'R',1.0,'k');
    IRdata.steps.data1=[];
    IRdata.steps.data2=[];
    IRdata.steps.data3=[];
    if size(IRdata.steps.data,1)>0,
        IRdata.steps.data1=IRdata.steps.data(:,1);
        IRdata.steps.data2=IRdata.steps.data(:,2);
        IRdata.steps.data3=IRdata.steps.data(:,3);
    
    
    
%% Manipulate raw sensor data in order to make a global map
%% Line extraction
    
    
 
    
    % Extract features from the raw measurements in data structure given the 
    % extraction algorithm parameters in myPARAMS. The local map L is returned
    % With the flag DISPLAY = 1, the extraction result is plotted in
    % a new figure window.    
    if fullscan,
        [L,rawSegs,lines] = extractlines( IRdata ,myLineParams,myNewpose,display);
    else
        myLineParams.cyc=0;
        [L,rawSegs,lines] = extractlines2( IRdata ,myLineParams,myNewpose,display);
        myLineParams.cyc=1;
    end
    disp('oooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooo');
    disp(rawSegs);
    % --------------------------
    % --- Measurement Prediction
    % --------------------------
        
    % Predict the map G at the current robot pose, its uncertainty and the
    % robot-to-sensor displacement in PARAMS.
    [Gpred,Hpred] = predictmeasurements(Gin);
         
    % ------------
    % --- Matching
    % ------------

    
    % Apply the nearest neighbor standard filter to match the observations in
    % the local map L to the predicted measurements GPRED and accept the 
    % pairing on the level alpha in PARAMS.
    [nu,R,H,associations,matches_ID_G_L] = matchnnsfLines(Gpred,Hpred,L,myLineParams,displaying);
   
    % --------------
    % --- Estimation
    % --------------

    % Update the map with an extended Kalman filter based on the stacked
    % innovation vector nu and the stacked observation covariance matrix R.
    %Bjørn S
    G = estimatemapLines(Gin,nu,R,H,matches_ID_G_L,myNewpose,rawSegs);
    %Bjørn S
    
    % ------------------------------
    % --- Integrate new observations
    % ------------------------------
    % Add the non-associated observations marked by a -1 as the global matching 
    % feature index in ASSOCIATIONS to the map and return the augmented map
    G = integratenewobs(G,L,associations);
    
    
    
% ---------post processing of map after line-extraction----------- %
    
%FIXCORNERS. Nearby segments that make up a corner gets linked.
    %The segments must be at least minLength long and have endpoints
    %cloaser than maxDist to be liked.
    maxDist= 0.1; minLength=0.1;
    G=fixCorners(G, maxDist, minLength);    
    


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Map post-processing    
    % ---------------------------------
    % --- Map post-prosessing, Bjørn S
    % ---------------------------------
    % This is the place to analyse the map. Find gaps in the wall,
    % corners, dead-ends,etc,etc

    
    
    
    %record robot path
    % added by Trond M
    navData.robotPath(istep+1,1:1:3) = myNewpose;
    
    poseM(1:istep,1:2)=navData.robotPath(1:istep,1:2);
    poseN(1:istep,1:2)=navData.robotPath(2:istep+1,1:2);
    totalDist = sum(sqrt((poseN(:,1)-poseM(:,1)).^2+(poseN(:,2)-poseM(:,2)).^2));
    navData.totalDist=totalDist;
    

    
    
    
    %GAPFINDER. finds gaps/openings on single lines in the map
    % and other types of gaps...
    % Will be replaced ASAP.... 03.11.07
    minGap=0.25; maxGap=0.7;
    
    initGaps=[];
    gaps = gapFinder3(G,minGap,maxGap,navData.robotPath,initGaps);
    
%% Visualize mapping step
    % ------------------
    % --- Visualize mapping step
    % ------------------
    
    cla(myHandles.mainPlotWindow);
    if dispmode >= 1,
     
        axes(myHandles.mainPlotWindow);
        hold(myHandles.mainPlotWindow,'on');
        
        set(gca,'Box','On'); axis equal;
        title(['Global map at step ',int2str(istep),...
            '. Total distance: ',num2str(totalDist,3),' m']);
        draw(G,0,1,0,'k');  % draw lines(infinite length) and robot
        
     
        if isfield(myLineParams,'axisvec');
            axis(myLineParams.axisvec);
        end;
        if (dispmode == 2) || (dispmode == 4),
            
        end;
        
        % draw segments
        globalMap=get(G,'x');
        % only consider arline features
        f=length(globalMap);
        while f>1,
            if ~strcmp(get(globalMap{f},'Type'),'alpha,r line feature'),
                globalMap(f)=[];
            end
            f=f-1;
        end
        endpoints=[];
        for i=2:length(globalMap)        
            startnstopp=get(globalMap{1,i},'ss');
            for j=1:4:size(startnstopp,2)             
                plot([startnstopp(j) startnstopp(j+2)],[startnstopp(j+1) startnstopp(j+3)],'LineWidth',2.7,'Color',[1 0 0]);
                plot( startnstopp(j),startnstopp(j+1),'kx','LineWidth',2,'MarkerSize',9);
                plot( startnstopp(j+2),startnstopp(j+3),'kx','LineWidth',2,'MarkerSize',9);
                endpoints=vertcat(endpoints,[startnstopp(j) startnstopp(j+1);startnstopp(j+2) startnstopp(j+3)]);
            end;
        end;
        
        %draw gaps
        if numel(gaps) > 1,
            for i=1:1:size(gaps,1)
                for j=1:4:size(gaps,2)
                    plot([gaps(i,j) gaps(i,j+2)],[gaps(i,j+1) gaps(i,j+3)],'y','LineWidth',2.7);
                end;         
            end;
        end; 
        
        % ---draw/concatenate points to edges---- %
        globalMap=get(G,'x');
        % only consider arline features
        f=length(globalMap);
        points=[];
        while f>1,
            if ~strcmp(get(globalMap{f},'Type'),'point feature'),
                globalMap(f)=[];
            end
            f=f-1;
        end
        for i=2:length(globalMap)        
            points(i-1,1:2)=get(globalMap{1,i},'x');
        end
        points=vertcat(endpoints,points);
        
        edges=linkPoints(points,0.15,1);
        
        % unknown area detection function
        newDetect=1;
        if newDetect
            myHandles.checkTable=getUnknownAreas(G,myHandles,navData.robotPath,edges);
        end
        
        % draw robot path        
        for i=1:1:size( navData.robotPath,1 )-1
            plot([navData.robotPath(i,1) navData.robotPath(i+1,1)],[navData.robotPath(i,2) navData.robotPath(i+1,2)],'g','LineWidth',1);
            plot( navData.robotPath(i,1),navData.robotPath(i,2),'kx','LineWidth',2,'MarkerSize',5);
        end;
        
    end;
    
    end%% Navigation
    
 % --- Navigation step --- %
    
    % produce points from checkTable in 2D plane
            cT=myHandles.checkTable;
            szcT=size(cT,1);
            rad=myHandles.maxPerceptRadius; 
            res=myHandles.angularRes; %
            NOP=360/res;
            THETA=linspace(0,2*pi,NOP);
            unknown=[];
            for pos=1:szcT,
                tmpAngles=THETA(find(cT(pos,2:end)==0))';
                [tmpX tmpY]=pol2cart(tmpAngles,ones(length(tmpAngles),1)*rad);
                tmpX=tmpX + repmat(navData.robotPath(cT(pos,1),1),length(tmpAngles),1);
                tmpY=tmpY + repmat(navData.robotPath(cT(pos,1),2),length(tmpAngles),1);
                unknown=vertcat(unknown,[tmpX tmpY]);
            end

            [unknownGaps,edges]=linkPoints2(unknown,0.06,1);

            gaps=vertcat(gaps,unknownGaps);

    
    if fullscan,  
        if( myLineParams.navigation == 1 )
            lastState=navData.state;     
            
            [navError,myNavData,myHandles,scndata]=leftWallFollowerToBackTrackerLines(G,myCon,navData,gaps,myHandles);
            navData=myNavData;
            oldPose=myNewpose;
            if(navError)
                disp('Navigation error');
            end;
            
            
        elseif( myLineParams.navigation == 2 )
        
        else
    
        end
        if navData.state==1 && lastState==1 && myHandles.driveScan==1,
            fullscan=0;
        end
        
        istep = istep+1;
    else
        fullscan=1;
    end
    
    
    if ~myLineParams.simulation,
        pause(0.1); %this line is not active during simulation
        
        % read data left on COM port
        
        if myCon.bytesAvailable,
            data = fread(myCon);
            disp('leftover data on serialconnection between timesteps. Timestep: ')
            disp(istep)
            disp('data: ')
            disp(data)
        end
    end
    
end % main while loop
