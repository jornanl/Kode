%% SLAM which produces a global map based on lines

function lineBeaconSLAM(myCon,myHandles,myLineParams,myBeaconParams)
global ROBOTPOSE;
global REALPOSE;
global RUNNING;

 

REALPOSE=[0 0 0];
% "robothouse" parameters
%WIDTH=0.515; % [meter]
%DEPTH=0.408; % [meter]


myLineParams.robot.class = 'robotdd';

dispmode = myLineParams.displayGlobal;
display = myLineParams.displayLocal;
displaying= myLineParams.displayMatch;

goHome = myLineParams.goHome;
timeLeftToGoHome = myHandles.goHometext;

% ----- Slam ----- %
% optional axis vector for global map figure. Useful with infinite lines
myLineParams.axisvec = [-2.2 2.2 -2.2 2.2];

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

 
%% main while loop for mapping

while(RUNNING)
    
  
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
%        set(myHandles.edit_realposeTheta,'String',num2str(myNewpose(3)*180/pi ));
        set(myHandles.edit_realposeTheta,'String',num2str(REALPOSE(3) ));
    end

    pause(0.01);
    if RUNNING==2,
        while ~(RUNNING==1 || RUNNING==0)
            pause(0.1);
        end
        if ~RUNNING,
            break;
        end
    end

    myNewpose=[myNewpose(1)/1000 myNewpose(2)/1000 myNewpose(3)];
    
    utover= get(Gin,'x');
    utover{1,1}=set( utover{1,1},'x', myNewpose' );
    Gin=set(Gin,'x',utover);
    
    % --------------------
    % --- State prediction
    % --------------------
    
    % Changed to allow for different setups
    if strcmp(myCon, 'NXT_kamera')
        RAD_LEFT_WHEEL=0.028;       % As printed on tyre
        RAD_RIGHT_WHEEL=0.028;      % As printed on tyre
        WHEEL_BASE=0.120;           % Measured
        EGC=0.200;                  % error growth coefficient on left and right wheel in [1/m].
        [aL,aR]=getAngularDispl(myNewpose,oldPose, RAD_LEFT_WHEEL,RAD_RIGHT_WHEEL,WHEEL_BASE);
        aL_total=aL_total+aL/500; % Angular displacements of left wheel in [rad]
        aR_total=aR_total+aR/500; % Angular displacements of right wheel in [rad]
    else
        RAD_LEFT_WHEEL=0.0248;  % Changed from 0.02149 because the wheels are replaced
        RAD_RIGHT_WHEEL=0.0248; % Changed from 0.02149 because the wheels are replaced
        WHEEL_BASE=0.210;    % Changed from 0.1894938 because the wheels are replaced
        EGC=0.004;           % error growth coefficient on left and right wheel in [1/m]. Decreased from 0,03 because error correction is now implemented on microcontroller level.
        [aL,aR]=getAngularDispl(myNewpose,oldPose, RAD_LEFT_WHEEL,RAD_RIGHT_WHEEL,WHEEL_BASE);
        aL_total=aL_total+aL/500; % Angular displacements of left wheel in [rad]
        aR_total=aR_total+aR/500; % Angular displacements of right wheel in [rad]
    end
    
    if strcmp(myCon,'simulator') && ~myHandles.poseError, % simulation without position error: set errors to zero
        EGC=0; 
        aL_total=0;
        aR_total=0;
    end
    
%    ENC.PARAMS.KL: error growth coefficient of left wheel with unit in [1/m].
%    ENC.PARAMS.KR: error growth coefficient of right wheel with unit in [1/m]. 
%    ENC.STEPS(i).DATA1: angular displacements of left wheel in [rad] and
%    monotonically increasing.
%    ENC.STEPS(i).DATA2: angular displacements of right wheel in [rad] and
%    monotonically increasing. 

    enc.params.kr=EGC;
    enc.params.kl=enc.params.kr;
    enc.steps(istep+1).data1=aL_total;
    enc.steps(istep+1).data2=aR_total;


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
        
        %Because cam robot often finds no data points, this condition is
        %necessary:
        if(~isempty(data)),
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

            if strcmp(myCon,'NXT_kamera')
                NS=1;   %Just one "sensor"
            else
                NS=4;   %Four sensors
            end
            for sens=1:1:NS,
                tmpData=data(data(:,1)==sens,:);

                IRdata.steps.data=vertcat(IRdata.steps.data,tmpData);
            end
            %plot(IRdata.steps.data(:,2), IRdata.steps.data(:,3),'mx');
        else
            IRdata.steps.data=[];
        end
        
        
        
        
        
% ------- processing data from driving------- %   
    elseif size(scndata,1)>0, % there are data collected during driving which need to be considered
        myNewpose(1:2)=oldPose(1:2);
        
    
        utover= get(Gin,'x');
        utover{1,1}=set( utover{1,1},'x', myNewpose' );
        Gin=set(Gin,'x',utover);
        
    
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
    
    
    IRdata.steps.data1=[];
    IRdata.steps.data2=[];
    IRdata.steps.data3=[];
    if size(IRdata.steps.data,1)>0, % If there are sensordata
        IRdata.steps.data1=IRdata.steps.data(:,1);
        IRdata.steps.data2=IRdata.steps.data(:,2);
        IRdata.steps.data3=IRdata.steps.data(:,3);
    
    
    
%% Manipulate raw sensor data in order to make a global map
%% Line extraction
    
    % Exclude point features here
    [Gin,E]=include(Gin,'alpha,r line feature');
    
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
    %Bj�rn S
    G = estimatemapLines(Gin,nu,R,H,matches_ID_G_L,myNewpose,rawSegs);
    %Bj�rn S
    
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
    



% delete pointfeatures close to segments
    Tmp=get(E,'x');
    Ctmp=get(E,'c');
    if ~isempty(Tmp),   
        xyp=[];
        Xg=get(G,'x');
        %Cg=get(G,'c');
        for p=1:length(Tmp)
            xyp(p,1:2)=get(Tmp{p},'x');
            %Cp(p+1:2*p,)=Tmp{p,'c'};
        end
        segs=[];
        % find segments
        for i=2:length(Xg)
            ss=get(Xg{1,i},'ss');
            tmp=[];
            for j=1:4:size(ss,2),
                tmp=vertcat(tmp,ss(j:j+3));
            end
    
            for t=1:size(tmp,1),
                if tmp(t,1)> tmp(t,3), % sort with respect to x
                    tmp(t,1:4)=[tmp(t,3) tmp(t,4) tmp(t,1) tmp(t,2)];
                end
            end
            segs=vertcat(tmp,segs);
        end
        seg=size(segs,1);
        while seg>0,
            if ~isempty(xyp),
            dist = distancePointEdge(xyp, segs(seg,:));
            indInte=dist<0.03;
            Tmp(indInte)=[];
            % update covarianse matrix
            Ctmp(indInte,:)=[];
            Ctmp(:,indInte)=[];
            
            xyp(indInte,:)=[];
            end
            seg=seg-1;
        end
    end
    
    
%% preparation for beacon extraction
    
    
    %find raw data not integrated in global map
    xR=IRdata.steps.data(:,2);
    yR=IRdata.steps.data(:,3);
    
    rawData=[xR yR];
    if size(rawSegs,2)>0,    
        segs=rawSegs(13:16,:)';
        
        for seg=1:size(segs,1),
            dist = distancePointEdge(rawData, segs(seg,:));
            indInte=dist<0.03;
            rawData(indInte,:)=[];
        end
            
    end

    
    % -----include pointfeatures to map----- %
    Xg=get(G,'x');
    Cg=get(G,'c');
    Xnew=horzcat(Xg,Tmp);

    totalL=size(Cg,2)+size(Ctmp,2);
    Cnew=Cg;
    if size(Ctmp,2)>0,
        Cnew(size(Cg,2)+1:totalL,size(Cg,2)+1:totalL)=Ctmp;
        % add zeros to covarianse matrix
        for e=size(Cg,2)+1:totalL,
            Cnew(e,1).C=eye(2,3)*zeros(3,3);
            Cnew(1,e).C=zeros(3,3)*eye(3,2);
            for h=2:size(Cg,2),
                Cnew(h,e).C=zeros(2,2);
                Cnew(e,h).C=zeros(2,2);
            end 
        end
    end
    % put all features back to map G
    G=set(G,'x',Xnew,'c',Cnew);
    
    
    
%% Beacon extraction..
    if fullscan, % extract points from a fullscan only
    
    % check for beacon extraction
    % constant beacon covariance matrix
    myBeaconParams.Cb = 0.01*eye(2);
    % minimal nof raw points on reflector in order to be valid
    myBeaconParams.minnpoints = 3;
    
    % produce a perception map P containing only pointfeatures in a radius equal to maxperceptionRaduis.
    % Gin contains pointfeatures outside perception radius and all other
    % features not considered during pointextraction
    [P,Gin]=GtoPconv9(G,myHandles);
    
    
    % use only rawData which haven't contributed to line extraction
    for k=1:size(rawData,1),
        Pin=P;
        
        
        x=rawData(k,1);
        y=rawData(k,2);
        IRdata.steps.data1=3;
        IRdata.steps.data2=[x;x;x];
        IRdata.steps.data3=[y;y;y];
        
        
        % beacon extraction functions
        
        L = extractbeacons2( IRdata ,myBeaconParams,0,myNewpose);
        [Ppred,Hpred] = predictmeasurements(Pin);
        [nu,R,H,associations] = matchnnsf(Ppred,Hpred,L,myBeaconParams,displaying);
        P = estimatemap(Pin,nu,R,H);
        P = integratenewobs(P,L,associations);
    
    end
    
    % merge updated perception map P and map Gin containing the other
    % features
    
    G=PtoGconv9(P,Gin);
    

    end % fullscan
    


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Map post-processing    
        
    
    %record robot path
    navData.robotPath(istep+1,1:1:3) = myNewpose;
    
    poseM(1:istep,1:2)=navData.robotPath(1:istep,1:2);
    poseN(1:istep,1:2)=navData.robotPath(2:istep+1,1:2);
    totalDist = sum(sqrt((poseN(:,1)-poseM(:,1)).^2+(poseN(:,2)-poseM(:,2)).^2));
    navData.totalDist=totalDist;
    %disp('total distance travelled:');
    %disp(totalDist);
    %%%%%%%%%%%%%%%%%%%
    
% ---------post processing of map after point-extraction----------- %
    % deleting points overlapping linesegments
    G=deletePointsFromMap(G);   
% ----------------------------------------------------------------- %
    
    
    
    %GAPFINDER. finds gaps/openings on single lines in the map
    % and other types of gaps...

    minGap=0.25; maxGap=0.7;
    
    initGaps=[];
    gaps = gapFinder3(G,minGap,maxGap,navData.robotPath,initGaps);
    
    
    end %if there are sensordata 
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
                plot([startnstopp(j) startnstopp(j+2)],[startnstopp(j+1) startnstopp(j+3)],'LineWidth',2.7,'Color',[1 0 0]); % Red segments
                plot( startnstopp(j),startnstopp(j+1),'kx','LineWidth',2,'MarkerSize',9); % Endpoint on red segment, marked with x
                plot( startnstopp(j+2),startnstopp(j+3),'kx','LineWidth',2,'MarkerSize',9); % Endpoint on red segment, marked with x
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
        globalMap1=get(G,'x');
        % only consider arline features
        f=length(globalMap1);
        points=[];
        while f>1,
            if ~strcmp(get(globalMap1{f},'Type'),'point feature'),
                globalMap1(f)=[];
            end
            f=f-1;
        end
        for i=2:length(globalMap1)        
            points(i-1,1:2)=get(globalMap1{1,i},'x');
        end
        points=vertcat(endpoints,points);
        
        edges=linkPoints(points,0.15,1);
        
        % unknown area detection function
        newDetect=1;
        if newDetect && fullscan
            myHandles.checkTable=getUnknownAreas(G,myHandles,navData.robotPath,edges);
        end
        
        % draw robot path        
        for i=1:1:size( navData.robotPath,1 )-1
            plot([navData.robotPath(i,1) navData.robotPath(i+1,1)],[navData.robotPath(i,2) navData.robotPath(i+1,2)],'g','LineWidth',1);
            plot( navData.robotPath(i,1),navData.robotPath(i,2),'kx','LineWidth',2,'MarkerSize',5);
        end;
        
    end;
    
%% Go Home

        if goHome == 1;
            timeLeftToGoHome = timeLeftToGoHome -1; 
            set(myHandles.goHomeText,'String',timeLeftToGoHome);
        end;
        %Set robot to go home after a chosen value or when it is finished
        %mapping
        if timeLeftToGoHome == 0 || navData.state == 9 ||  get(myHandles.Recharge_checkbox,'Value') == 1;
            set(myHandles.statusText,'String','Calculating path home.');
            
            if exist('globalMap1') == 0;
                sp = computeShortestPath(globalMap,navData.robotPath);
            else
                sp = computeShortestPathPoints(globalMap,globalMap1,navData.robotPath);    
            end
            
            % Creating x and y vector
            x = [];
            y = [];
            for i=1:2:size(sp,2)
                x = [x sp(i)];
                y = [y sp(i+1)];
            end
            hold(myHandles.mainPlotWindow,'on');
            plot(x,y)
            hold(myHandles.mainPlotWindow,'off');
            
            sp = flipdim(sp,2);
            t=1;
         
            [poses,myError]=getRobotPose(myCon,myHandles);

            orgPos = [poses(1)/1000 poses(2)/1000]; %[m]
            oldPos = orgPos;
            set(myHandles.statusText,'String','Path found, going home!');
            
            for i=1:2:size(sp,2)
            % use cartesian coordinates to go to target
            x=sp(t+1)*1000; % x in [mm]
            y=sp(t)*1000; % y in [mm] 

            error=goToPose2(myCon,x,y,myHandles);
            
            [poses,myError]=getRobotPose(myCon,myHandles); %[mm]
            newPos = [poses(1)/1000 poses(2)/1000]; %[m]
       
         
            % draw robot path
            hold(myHandles.mainPlotWindow,'on');
            plot([oldPos(1) newPos(1)],[oldPos(2) newPos(2)],'g','LineWidth',1);
            plot( newPos(1),newPos(2),'kx','LineWidth',2,'MarkerSize',5);
            hold(myHandles.mainPlotWindow,'off');
            oldPos = newPos;
            t = t+2;        
            
            end
            %Not active during simulation
            if ~myLineParams.simulation;
                [error]=dock(myCon,myHandles,myLineParams,navData);
                if(exist('chargeTime','var')==0)
                    chargeTime = 5;     %charge time in secs, if no charge time received from robot
                end
                while (chargeTime > 0)  %charging
                    set(myHandles.statusText,'String',sprintf('Robot has docked and is recharging. %d s left.', chargeTime));
                    pause(1);
                    chargeTime = chargeTime -1;
                end
                set(myHandles.statusText,'String','Finished recharging.');
                if( serialAsyncWrite(myCon,'>') )
                    set(myHandles.statusText,'String','Recharge complete, NO MESSAGE TO ROBOT' ); %robot doesn't know..
                else
                    set(myHandles.statusText,'String','Recharge complete' );
                end
            end;
            set(myHandles.Recharge_checkbox,'Value',0);
            
            
            %% Go back
            %If the robot is not finished mapping the reason it was going
            %back was to update the position.
            if navData.state ~= 9;
                navData.state = 93;
                %go back to start posistion
                [poses,myError]=getRobotPose(myCon,myHandles); %[mm]
                newPos = [poses(1) poses(2)]; %[m]
                x = newPos(1) + 150; %messured number from the wall to starting position
                y = newPos(2);
                error=goToPose2(myCon,x,y,myHandles);
                error=setRobotPose(myCon,myHandles, [0 0 0]);
                sp = flipdim(sp,2);
                sp(1:2) = [];
                sp = [sp orgPos(1) orgPos(2)];
                sp = sp*1000;  
                t=1;
                set(myHandles.statusText,'String','Returning to previous position..');
                for i=1:2:size(sp,2)  
                % use cartesian coordinates to go to target
                x=sp(t); % x in [mm]
                y=sp(t+1); % y in [mm]   
                
                error=goToPose2(myCon,x,y,myHandles);
                [poses,myError]=getRobotPose(myCon,myHandles); %[mm]
                newPos = [poses(1)/1000 poses(2)/1000]; %[m]
                
              
                % draw robot path 
                hold(myHandles.mainPlotWindow,'on');
                plot([oldPos(1) newPos(1)],[oldPos(2) newPos(2)],'g','LineWidth',1);
                plot( newPos(1),newPos(2),'kx','LineWidth',2,'MarkerSize',5);
                hold(myHandles.mainPlotWindow,'off');
                oldPos = newPos;
                t = t+2;    
                end
            %Setting the robot state back to mapping
            set(myHandles.statusText,'String','Arrived, continuing mapping');
            navData.state = 1;    
            end
        end   
%% Navigation
    
 % --- Navigation step --- %
    
    % produce points from checkTable in 2D plane
            cT=myHandles.checkTable;
            szcT=size(cT,1);
            rad=myHandles.maxPerceptRadius; 
            res=myHandles.angularRes; %
            NOP=360/res;
            THETA=linspace(0, 2*pi, NOP);
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
            
            [navError,myNavData,myHandles,scndata]=leftWallFollowerToBackTracker(G,myCon,navData,gaps,myHandles);
            navData=myNavData;
            oldPose=myNewpose;
            if(navError)
                disp('Navigation error');
            end
        end
        if navData.state==1 && lastState==1 && myHandles.driveScan==1, % driveScan is a checkbox in the GUI
            fullscan=0;
        end
        
        istep = istep+1;
    else
        drawRectangle(G,myNavData.b,myNavData.l);
        fullscan=1;
    end
    
    
    if ~myLineParams.simulation && ~strcmp(myCon,'NXT_kamera'),
        pause(0.1); %this line is not active during simulation
        
        % read data on COM port
        
        if myCon.bytesAvailable,
            data = fread(myCon);
            disp('leftover data on serialconnection between timesteps. Timestep: ')
            disp(istep)
            disp('data: ')
            disp(data)
        end
    end
%% BATTERY RECHARGE CHECK
    
    % --- Set recharge checkbox if robot needs charging --- %
    
    %Not if simulator is active
    if ~myLineParams.simulation,
        if strcmp(myCon,'NXT_kamera') %NXT camera robot
            %get battery data
            [recharge chargeTime] = nxt_get_battery_state();
            if(recharge)
                disp('battery capacity LOW, setting recharge flag')
                set(myHandles.statusText,'String','BATTERY LOW! Recharge initiated.');
                % Setting battery recharge flag
                set(myHandles.Recharge_checkbox,'Value',1);
            else
                % Battery capacity OK
                disp('battery capacity OK');
            end
        else
            %Asking for battery state
            if( serialAsyncWrite(myCon,'?') )
                set(myHandles.statusText,'String','Recharge check: Unable to send serial data' );
            else
                [myError,data]=serialSyncRead(myCon,1);
                if(myError)
                    disp('slam/LineBeaconSLAM: Recharge check: no data received..')
                elseif data=='='
                    % Battery capacity OK
                    disp('battery capacity OK')
                elseif data=='<'
                    disp('battery capacity LOW, setting recharge flag')
                    set(myHandles.statusText,'String','BATTERY LOW! Recharge initiated.');
                    % Setting battery recharge flag
                    set(myHandles.Recharge_checkbox,'Value',1);
                    %Receiving robot charging time [seconds]
                    [myError,timeVector]=serialSyncRead(myCon,2);
                    if(myError)
                        disp('slam/LineBeaconSLAM: Error receiving battery charging time from robot')
                    else
                        %converting 2 x 8 bit unsigned to 16 bit unsigned
                        chargeTime = timeVector(1)*256+timeVector(2);
                    end
                end
            end
        end
    end

end % main while loop
