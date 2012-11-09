function beaconSLAM(myCon,myHandles,myBeaconParams,myLineParams)
global RUNNING;
myHandles.driveScan=0;
% initial robot start pose [m][m][rad]
%myBeaconParams.robot.x = [0.5 0.5 0]';
% initial robot pose covariance
%myBeaconParams.robot.C = 0.0001*eye(3);

myBeaconParams.robot.class = 'robotdd';
%myBeaconParams.robot.formtype = 4;
%      TYPE = 0 draws only a cross with orientation theta
%      TYPE = 1 is a differential drive robot without contour
%      TYPE = 2 is a differential drive robot with round shape
%      TYPE = 3 is a round shaped robot with a line at theta
%      TYPE = 4 is a differential drive robot with rectangular shape
%      TYPE = 5 is a rectangular shaped robot with a line at theta
% robot-to-sensor transform expressed in the
% robot frame with units [m] [m] [rad]
%myBeaconParams.robot.xsensor = [0; 0; 0.0];

dispmode = myBeaconParams.displayGlobal;
display = myBeaconParams.displayLocal;
displaying= myBeaconParams.displayMatch;
%    MODE is between 0 and 4. With 0 the algorithm runs through, with 1 the
%    global map is drawn at each step, and paused if DISPLAYMODE = 2. with
%    DISPLAYMODE = 3, the global and the local map is drawn, and paused if
%    DISPLAYMODE = 4.

% ----- Feature Extraction ----- %
% Minimum distance between beacon's

% constant beacon covariance matrix
myBeaconParams.Cb = 0.01*eye(2);
% minimal nof raw points on reflector in order to be valid
myBeaconParams.minnpoints = 1;
% significance level for NNSF matching

myBeaconParams.axisvec = [-2 2 -2 2];
myBeaconParams.robot.xsensor = [0 0 0];
G = initmap(0,'global map',myBeaconParams);
IRdata.params.xs= myBeaconParams.robot.xsensor;
istep = 0;



navData.robotPath=[];
navData.state=0; 
navData.backTrackPoint=[];
navData.STARTbackTrackPoint=[];
navData.backTrackPointAtGap=[];
navData.allBackTrackPoints=[0];
navData.backTrackPointAtGap=[];
navData.lastHeading=0;
gaps=[];
cnt=0;
fullscan=1;
clc;


while(RUNNING)
    
    pause(0.01);
    if RUNNING==2,
        while ~(RUNNING==1)
            pause(0.1);
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
    myNewpose=[myNewpose(1)/1000 myNewpose(2)/1000 myNewpose(3)]; 

    IRdata.steps.data=[];
    if fullscan,
        %set robot pose in map

        utover= get(Gin,'x');
        utover{1,1}=set( utover{1,1},'x', myNewpose' );
        Gin=set(Gin,'x',utover);
    
    
    
% -----performing a full scan-------- %
    
        [error,data]=fullScan(myCon,myHandles);
    
        % postprocessing data
        NS=4;
        for sens=1:NS,
        
            tmpData=data(data(:,1)==sens,:);
        
            IRdata.steps.data=vertcat(IRdata.steps.data,tmpData);
        end
% ------- processing data from driving------- %   
    elseif numel(scndata)>0, % there are data collected during driving which need to be considered
        
        delta=sqrt((myNewpose(1)-oldPose(1))^2 + (myNewpose(2)-oldPose(2))^2);
        utover= get(Gin,'x');
        utover{1,1}=set( utover{1,1},'x', myNewpose' );
        Gin=set(Gin,'x',utover);
    
    
        % rotate the global x,y points received from robot to local x,y points
        % in order to use them in extractlines
        rotmat=[cos(-myNewpose(3)) -sin(-myNewpose(3));sin(-myNewpose(3)) cos(-myNewpose(3)) ];
        
        scndata(:,2:3)=(rotmat*(scndata(:,2:3))')';
        scndata(:,2)=scndata(:,2) - delta;
        scndata(:,3)=scndata(:,3);
        scndata(find(-0.1<scndata(:,3) & scndata(:,3)<0.1),:)=[];
        for sens=2:2:5,
    
            tmpData=scndata(scndata(:,1)==sens,:);
            % artifical scandata..
            [artifx artify]=pol2cart((-1:0.2:1)*pi/8 ,0.03);
            artif=zeros(length(artifx),3);
            artif(:,2:3)=horzcat(artifx',artify');
            tmpData=vertcat(tmpData,artif);
     
            IRdata.steps.data=vertcat(IRdata.steps.data,tmpData);
        end
   
    end %fullscan
    

    IRdata.steps.data1=IRdata.steps.data(:,1);
    IRdata.steps.data2=IRdata.steps.data(:,2);
    IRdata.steps.data3=IRdata.steps.data(:,3);
    
% produce a perception map P containing only pointfeatures in a radius equal to maxperceptionRaduis.
    % Gin contains pointfeatures outside perception radius and all other
    % features not considered during pointextraction
    [P,Gin]=GtoPconv9(Gin,myHandles);
    
    
    % use only rawData which haven't contributed to line extraction, see
    % comment earlier..
    %figure(4); clf;
    
    for k=1:size(IRdata.steps.data,1),
        Pin=P;
        
        
        x=IRdata.steps.data(k,2);
        y=IRdata.steps.data(k,3);
        IRdata.steps.data1=3;
        IRdata.steps.data2=[x;x;x];
        IRdata.steps.data3=[y;y;y];    
        % Extract features from the raw measurements in data structure given the 
        % extraction algorithm parameters in myBeaconParams. The local map L is returned
        % With the flag DISPLAY = 1, the extraction result is plotted in
        % a new figure window.
        
        L = extractbeacons2( IRdata ,myBeaconParams,display,myNewpose);
        
        % --------------------------
        % --- Measurement Prediction
        % --------------------------
    
        % Predict the map G at the current robot pose, its uncertainty and the
        % robot-to-sensor displacement in PARAMS.
        
        %[Gpred,Hpred] = predictmeasurements(Gin);
        [Ppred,Hpred] = predictmeasurements(Pin);
        %[Gpred,Hpred] = predictmeasurementsBeacons(Gin);
         
        % ------------
        % --- Matching
        % ------------
         
        % Apply the nearest neighbor standard filter to match the observations in
        % the local map L to the predicted measurements GPRED and accept the 
        % pairing on the level alpha in PARAMS.
        
        [nu,R,H,associations] = matchnnsf(Ppred,Hpred,L,myBeaconParams,displaying);
                 
        % --------------
        % --- Estimation
        % --------------
         
        % Update the map with an extended Kalman filter based on the stacked
        % innovation vector nu and the stacked observation covariance matrix R.
        
        P = estimatemap(Pin,nu,R,H);
         
        % ------------------------------
        % --- Integrate new observations
        % ------------------------------
        % Add the non-associated observations marked by a -1 as the global matching 
        % feature index in ASSOCIATIONS to the map and return the augmented map
        P = integratenewobs(P,L,associations);
        
        
    
        if(~RUNNING)
            break;
        end;
    
    end;% for "angle"
    
    % merge map P and Gin
    G=PtoGconv(P,Gin);
    
%% Map post-processing    
    
    
    %record robot path
    navData.robotPath(istep+1,1:1:3) = myNewpose;
    
    poseM(1:istep,1:2)=navData.robotPath(1:istep,1:2);
    poseN(1:istep,1:2)=navData.robotPath(2:istep+1,1:2);
    totalDist = sum(sqrt((poseN(:,1)-poseM(:,1)).^2+(poseN(:,2)-poseM(:,2)).^2));
    %disp('total distance travelled:');
    %disp(totalDist);
    %%%%%%%%%%%%%%%%%%%
    
    
    
    
%% Visualize mapping step
    % ------------------
    % --- Visualize mapping step
    % ------------------
    cla(myHandles.mainPlotWindow);
    if dispmode>= 1,
        
        axes(myHandles.mainPlotWindow);
        hold(myHandles.mainPlotWindow,'on');
        
        set(gca,'Box','On'); axis equal;
        title(['Global map at step ',int2str(istep),'. Total distance: ',num2str(totalDist,3),' m'])
        draw(G,1,1,0,'k');
        if isfield(myBeaconParams,'axisvec');
            axis(myBeaconParams.axisvec);
        end;
        if (dispmode == 2) || (dispmode == 4),
            
        end;
        
        % try new unknown area detection function
        newDetect=0;
        if newDetect,
            myHandles.checkTable=getUnknownAreasBeacons(G,myHandles,navData.robotPath);
        end
        
        
        %plot robot path
        for i=1:1:size( navData.robotPath,1 )-1
            plot([navData.robotPath(i,1) navData.robotPath(i+1,1)],[navData.robotPath(i,2) navData.robotPath(i+1,2)],'g','LineWidth',1);
            plot( navData.robotPath(i,1),navData.robotPath(i,2),'kx','LineWidth',2,'MarkerSize',5);
        end;
    end;

    
    
%% Navigation
    % ------------------------------
    % --- Navigation step, Bjørn s
    % -------------------------------
    if fullscan,  
        if( myBeaconParams.navigation == 1 )
            lastState=navData.state;
            [navError,myNavData,myHandles,scndata]=leftWallFollowerToBackTrackerBeacons(G,myCon,navData,gaps,myHandles);
            navData=myNavData;
            oldPose=myNewpose;
            if(navError)
                disp('Navigation error');
            end;
            
            
        elseif( myBeaconParams.navigation == 2 )
        
        else
    
        end
        if navData.state==1 && lastState==1 && myHandles.driveScan,
            fullscan=0;
        end
        
        istep = istep+1;
    else
        fullscan=1;
    end
    
    
end % main while loop
