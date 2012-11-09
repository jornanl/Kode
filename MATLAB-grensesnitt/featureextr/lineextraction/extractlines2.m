%EXTRACTLINES Extract alpha,r-lines from range data.
%   L = EXTRACTLINES(SCNDATA,PARAMS,DISPLAY) extracts line segments from
%   uncertain range data given in polar coordinates. The lines, repre-
%   sented in the Hessian alpha,r-model, and their associated covariance
%   matrices are returned.
%
%   The structure SCNDATA holds the raw range readings according to the
%   output of the read function READONESTEP. PARAMS is a structure with
%   the algorithm parameters. With the flag DISPLAY being 1, the extrac-
%   tion result is displayed in a new figure.
%
%   [L,SEGS,LINES] = EXTRACTLINES(SCNDATA,PARAMS,DISPLAY) returns also
%   the matrices SEGS and LINES. Their definition is given below.
%
%   The algorithm uses a sliding window technique where a window of
%   constant size sweeps over the (ordered) set of range readings. At
%   each step the model (here: the line) is fitted to the points within
%   the window and a model fidelity measure is calculated. The sought
%   objects in the scan are found by a condition on the model fidelity.
%   
%   This produces an oversegmented range image in many cases since each
%   outlier can give rise to several small segments. Under the condition
%   of collinearity (on a significance level alpha), adjacent segments
%   are fused in a final step.
%
%   For the fit, perpendicular errors from the points onto the line are
%   minimized, which, in spite of being a non-linear regression problem
%   with points in polar coordinates, has an analytical solution.
%
%   The algorithm parameters in detail:
%      PARAMS.WINDOWSIZE : size of the sliding window in number of
%                          points (odd number, >=3)
%      PARAMS.THRESHFIDEL: threshold on model fidelity
%      PARAMS.FUSEALPHA  : significance level for segment fusion
%      PARAMS.MINLENGTH  : minimal length in [m] a segment must have
%      PARAMS.CYCLIC     : 1: range data are cyclic, 0: non-cyclic
%      PARAMS.COMPENSA   : heurististic compensation term to account
%                          for correlations between the raw data. Is 
%                          added to all alpha's after extraction, [rad]
%      PARAMS.COMPENSR   : same as above, for r values, in [m]
%
%   L is a map object according to the definition of the map class. It
%   holds the information of the infinite lines of LINES, treating the
%   line parameters alpha,r as the feature states.
%
%   SEGS is a matrix which holds information about the (finite-length)
%   segments and has the following elements:
%          1st row : identifier of segment
%          2nd row : identifier of line the segment belongs to
%          3rd row : number of contributing raw segments
%          4th row : number of contributing data points
%        5,6th row : line parameters alpha, r
%      7,8,9th row : covariance matrix elements saa, srr, sar
%         10th row : trace of covariance matrix
%      11,12th row : begin- and end-index of segment
%      13,14th row : Cartesian coordinates of segment start point
%      15,16th row : Cartesian coordinates of segment end point
%
%   LINES is a matrix which holds information about the (infinite)
%   lines and has the following elements:
%          1st row : identifier of line
%          2nd row : number of contributing segments
%          3rd row : number of contributing raw segments
%          4th row : number of contributing data points
%       5-10th row : same as above: parameters, cov. matrix, trace
%       >=11th row : identifiers of contributing segments
%
%   Reference:
%      K.O. Arras, "Feature-Based Robot Navigation in Known and Unknown
%      Environments", Ph.D. dissertation, Nr. 2765, Swiss Federal Insti-
%      tute of Technology Lausanne, Autonomous Systems Lab, June 2003.
%
%   See also FITLINEPOLAR, READONESTEP, MAP.

% This file is part of the CAS Robot Navigation Toolbox.
% Please refer to the license file for more infos.
% Copyright (c) 2004 Kai Arras, ASL-EPFL
% v.1.0, 1995, Kai Arras, IfR-ETHZ, diploma thesis
% v.2.0-v.4.0, 1997, Kai Arras, ASL-EPFL: probabilistic version
% v.4.1.1, 1.99/6.00/7.00, Kai Arras, ASL-EPFL 
% v.4.2, 05.12.02, Kai Arras, ASL-EPFL 
% v.4.3-4.4, Dec.2003, Kai Arras, CAS-KTH: minor adaptations for toolbox

function varargout = extractlines2(scndata,params,roboPose,displocal)

if ((nargout==1) || (nargout==3)) && ~isempty(scndata) && ...
    isstruct(params) && ((displocal==0) || (displocal==1)),
  
  % First: {S} --> {R} here
  xS = scndata.steps.data2;
  yS = scndata.steps.data3;
  sint = sin(scndata.params.xs(3));
  cost = cos(scndata.params.xs(3));
  xR = xS*cost - yS*sint + scndata.params.xs(1);
  yR = xS*sint + yS*cost + scndata.params.xs(2);
  [phi,rho] = cart2pol(xR,yR);
  stdrho = ones(length(xR),1)*scndata.params.stdrho;
  scan = [phi, rho, stdrho];
  l = size(scan,1);
  
  % ------------------------------------------------------------- %
  %%%%%%%%%%%%%%%%%%%%%%%%   Constants   %%%%%%%%%%%%%%%%%%%%%%%%%%
  % ------------------------------------------------------------- %
  % Constants of algorithm
  winsize     = params.windowsize;  % window size in points
  threshfidel = params.threshfidel; % model fidelity threshold
  minlength   = params.minlength;   % minimal segment length in [m]
  cyc         = params.cyclic;      % cyclic data or not
  saacompens  = params.compensa;    % heurist. compens. for raw data corr., [rad]
  srrcompens  = params.compensr;    % heurist. compens. for raw data corr., [m]
  mode        = 1;                  % multi-segment mode by default
  fuselevel   = chi2invtable(params.fusealpha,2); % signific. level for segment fusion
  
  % Constants for graphic output
  MUE = 50;									 	      % blows up infinite (alpha,r)-lines in figures
  TXTSHIFT = 0.1;						        % shifts text away from the denoted object in [m]
  
  % ---------------------------------------------------------- %
  %%%%%%%%%%% Slide Window And Fit Lines --> alpharC %%%%%%%%%%%
  % ---------------------------------------------------------- %
  halfWin = (winsize-1)/2;
  ibeg = 1 + ~cyc*halfWin;
  iend = l - ~cyc*halfWin;
  windowvec = ibeg:iend;
  for imid = windowvec,	
    for j = imid-halfWin:imid+halfWin,
      k = mod(j-1,l) + 1; 
      windowpts(j-imid+halfWin+1,:) = scan(k,:); 
    end;
    [p,C] = fitlinepolar(windowpts);
    ifillin = mod(imid-1,l) + 1; 
    alpharC(1,ifillin) = p(1,1);			 % alpha
    alpharC(2,ifillin) = p(2,1);		   % r
    alpharC(3,ifillin) = C(1,1);		   % sigma aa
    alpharC(4,ifillin) = C(2,2);			 % sigma rr
    alpharC(5,ifillin) = C(1,2);            % sigma ar
   end;
    
  % --------------------------------------------------------------------------- %
  %%%%%%%% Calculate Model Fidelity (=Compactness) Measure -> compactness %%%%%%%
  % --------------------------------------------------------------------------- %
  ibeg = 1 + ~cyc*(1+halfWin);   % taking always 3 adjacent points in model space
  iend = l - ~cyc*(1+halfWin);	 % keep track of shrinking valid range at array ends if cyc=1
  compactvec  = ibeg:iend;
  windowlines = zeros(5,3);
  compactness = zeros(1,l);
  for imid = compactvec,
    for j = imid-1:imid+1,
      k = mod(j-1,l) + 1;
      windowlines(:,j-imid+2) =  alpharC(:,k);
    end;
    compactness(mod(imid-1,l)+1) = calccompactness(windowlines);
  end;
  
  % ------------------------------------------------------------------- %
  %%%%%%%%% Apply Model Fidelity Threshold --> rawSegs(1/2/3,:) %%%%%%%%%
  % ------------------------------------------------------------------- %
  nSegBlowup  = halfWin;
  rSegIndices = findregions2(compactness(compactvec),threshfidel,'<',cyc);
  nRawSegs    = size(rSegIndices,1);
  
  if rSegIndices(1,1) >= 0,		% check case rSegIndices = -1, see help findregions
    %trwseg = cputime;
    % Test here for wraparound segment kernels
    if cyc && (nRawSegs > 1),														        % ingle segm. requires no cyclic treatment
      o1 = (rSegIndices(1,3) < rSegIndices(1,2));							  % does first segment wrap around?
      o2 = (rSegIndices(nRawSegs,3) < rSegIndices(nRawSegs,2));	% does last segment wrap around?
      if (o1 && o2) || (rSegIndices(1,2)-rSegIndices(nRawSegs,3) + ~(o1 | o2)*l <= 1),
        rSegIndices(1,2) = rSegIndices(nRawSegs,2);             % ibeg of last one is ibeg of new cyclic segm.
        rSegIndices = rSegIndices(1:nRawSegs-1,:);						  % delete last (=first) raw segment
        nRawSegs = nRawSegs - 1;
      end;
    end;
    % Blow up segments. Note: if cyc=0...
    % ...add (1+halfWin) to all indices since they are valid for the *shrinked* cmpct-vector
    for i = 1:nRawSegs,
      rSegIndices(i,2) = mod2(rSegIndices(i,2)-nSegBlowup+~cyc*(1+halfWin),l);
      rSegIndices(i,3) = mod2(rSegIndices(i,3)+nSegBlowup+~cyc*(1+halfWin),l);
    end;
    rawSegs = rSegIndices'; 
    
    % --------------------------------------------------------------------------------- %
    %%%%%%%%% Line Fit To Points In Homogenous Regions -> rawSegs(4/5/6/7/8/9,:) %%%%%%%%
    % --------------------------------------------------------------------------------- %
    for i = 1:nRawSegs,
      segPoints = 0;
      ibeg = rawSegs(2,i);
      iend = rawSegs(3,i);
      if iend < ibeg,                   % discontinuity lies within the segment
        segPoints = [scan(ibeg:l,:); scan(1:iend, :)];
      else															% normal case: discontinuity lies not within the segment
        segPoints = scan(ibeg:iend, :);
      end;
      [p,C] = fitlinepolar(segPoints);
      rawSegs(4,i) = length(segPoints);	% number of raw segment points
      rawSegs(5,i) = p(1,1);						% alpha
      rawSegs(6,i) = p(2,1);						% r
      rawSegs(7,i) = C(1,1);						% sigma_aa
      rawSegs(8,i) = C(2,2);						% sigma_rr
      rawSegs(9,i) = C(2,1);						% sigma_ar
      xybeg = [scan(ibeg,2)*cos(scan(ibeg,1)), scan(ibeg,2)*sin(scan(ibeg,1))];
      xyend = [scan(iend,2)*cos(scan(iend,1)), scan(iend,2)*sin(scan(iend,1))];
      Pe1 = calcendpoint(xybeg,rawSegs(5:6,i));
      Pe2 = calcendpoint(xyend,rawSegs(5:6,i));
      rawSegs(10:13,i) = [Pe1, Pe2]';	  % Cartesian endpoint coordinates of raw segment
    end;
    
    % --------------------------------------------------------------------------- %
    %%%%%%%%  Agglomerative Hierarchical Clustering --> D,M,rawSegs,nLines %%%%%%%%
    % --------------------------------------------------------------------------- %
    nLines = nRawSegs;
    if nRawSegs > 1,
      % Fill in distance matrix and set diagonal elements to zero
      for i=1:nRawSegs-1,
        for j=i+1:nRawSegs,
          D(i,j) = mahalanobisar(rawSegs(5:9,i),rawSegs(5:9,j));
        end;
      end;
      for i=1:nRawSegs, D(i,i) = 0; end;
      M = zeros(nRawSegs);		  % M holds the markers, whether a min in D is valid or not
      
      ahcmode  = 2;             % mode 2: coll,neigh,over; mode 1: coll,neigh; mode 0: coll
      terminate = 0; 
      while ~terminate,
        % Find minimum
        dmin = Inf;
        for i = 1:nRawSegs-1,   % finds always first (lowest index) instance of segment in question
          for j = i+1:nRawSegs,	
            if ~((D(i,j) < 0) || ((ahcmode==1)&&(M(i,j)==1)) || ((ahcmode==2)&&(M(i,j)==2))),
              if D(i,j) < dmin,
                dmin = D(i,j);
                minrow = i;     % row in D which holds the current minimal distance
                mincol = j;     % colon in D which holds the current minimal distance
              end;
            end;
          end;
        end;
        
        if dmin < fuselevel,
          if isneighbour(rawSegs, minrow, mincol, ahcmode-1, cyc) || (ahcmode==0),
            % Fusion of identified segments in 'rawSegs'
            nLines = nLines - 1; 
            rejectClustNr = rawSegs(1,mincol);
            for i = 1:nRawSegs,
              if rawSegs(1,i) == rejectClustNr,
                rawSegs(1,i) = rawSegs(1,minrow); % mark all segments by overwriting their old ID
              end;
            end;
            for i = 1:nRawSegs,
              if rawSegs(1,i) == rawSegs(1,minrow),
                % First: mark all raw points which belong to the segments in question -> scan(:,5)
                % Simple, less efficient but general solution. 
                ibeg = rawSegs(2,i);
                iend = rawSegs(3,i);
                if iend < ibeg,
                  scan(ibeg:l,5) = rawSegs(1,minrow);
                  scan(1:iend,5) = rawSegs(1,minrow);
                else
                  scan(ibeg:iend, 5) = rawSegs(1,minrow);
                end;
              end;
            end;
            % Second: go through the scan and collect them -> segPoints
            segPoints = [-1 -1 -1];
            segPointCounter = 0;
            for i = 1:l,
              if scan(i,5) == rawSegs(1,minrow),
                segPointCounter = segPointCounter + 1;
                segPoints = [segPoints; scan(i,1:3)];		% accumulate points
              end;
            end;
            % Third: fit the line with the combined set of points -> new cluster center
            [p,C] = fitlinepolar(segPoints(2:segPointCounter+1,:));		% remove -1-init-vector at index 1.
            rawSegs(5:9,minrow) = [p(1,1) p(2,1) C(1,1) C(2,2) C(1,2)]';
            rawSegs(5:9,mincol) = [p(1,1) p(2,1) C(1,1) C(2,2) C(1,2)]';
            
            % Calculate new distance in minrow-th row and column
            for i = minrow+1:nRawSegs,
              if D(minrow,i) > 0,
                D(minrow,i) = mahalanobisar(rawSegs(5:9,minrow),rawSegs(5:9,i));
                M(minrow,i) = 0;   % reset isneighbourhip, v.4.2
              end;
            end;
            for i = 1:minrow-1,
              if D(i,minrow) > 0,
                D(i,minrow) = mahalanobisar(rawSegs(5:9,minrow),rawSegs(5:9,i));
                M(i,minrow) = 0;   % reset isneighbourhip, v.4.2
              end;
            end;
            % Remove mincol-th row and column by marking them with -1
            for i = mincol+1:nRawSegs,
              D(mincol,i) = -1; M(mincol,i) = 0;
            end;	% mark D and unmark M
            for i = 1:mincol-1,
              D(i,mincol) = -1; M(i,mincol) = 0;
            end;	% mark D and unmark M
          else
            M(minrow, mincol) = ahcmode; % mark minrow, mincol
          end;
        else	% dmin >= fuselevel
          if ahcmode == 2,
            if mode == 1,               % mode is changed only if line mode
              ahcmode = 1;              % is enabled...
            else
              terminate = 1;            % ... or terminate otherwise
            end;
          elseif ahcmode == 1,
            ahcmode = 0;                % mode change 2 -> 1
          else
            terminate = 1;              % terminate if there are no collinear segments anymore
          end;
        end;
      end;
    end; % if nRawSegs > 1      
    
    % ---------------------------------------------------------- %
    %%%%%%%%%% Generate Data Structures --> segs, lines %%%%%%%%%%
    % ---------------------------------------------------------- %
    j = 1;
    for i = min(rawSegs(1,:)):max(rawSegs(1,:));
      idxvec = find(rawSegs(1,:)==i);
      if ~isempty(idxvec),
        nRawSegs = sum(idxvec~=0);						% segIndices holds:
        segIndices(j,1:length(idxvec)+2) = [i nRawSegs idxvec]; % 1st column: ID of line in 'rawSegs' 
        j = j + 1;												% 2nd column: number of raw segments
      end;														% 3:n column: indices of raw segm. in 'rawSegs'
    end;
    li = 0; si = 0;
    for i = 1:nLines,                           % for all lines
      lineID = segIndices(i,1);                 % get ID
      for j = 1:segIndices(i,2);              % for all raw segments constituting the line lineID
        ibeg = rawSegs(2,segIndices(i,2+j));  % get their begin- and end-indices which,
        iend = rawSegs(3,segIndices(i,2+j));  % in general, overlap the adjacent segments
        if iend < ibeg, 						% discontinuity lies within segment
          scan(ibeg:l,5) = lineID; 				% mark raw points
          scan(1:iend,5) = lineID;
        else  									% normal case: discontinuity lies not within segment
          scan(ibeg:iend,5) = lineID;         % mark raw points
        end;
      end;
      nLinePoints = (sum(scan(:,5) == lineID));
      jSegIndices = findregions(scan(:,5),lineID,'==',cyc);		% find joint segments
      nJointSegs  = size(jSegIndices, 1);
      
      lmax = -Inf; imax = 0;
      for j = 1:nJointSegs,
        ibeg = jSegIndices(j,2); iend = jSegIndices(j,3);
        [pBeg(1),pBeg(2)] = pol2cart(scan(ibeg,1),scan(ibeg,2));
        [pEnd(1),pEnd(2)] = pol2cart(scan(iend,1),scan(iend,2));
        Pe1 = calcendpoint(pBeg,rawSegs(5:6,segIndices(i,3))); % Cartesian projection
        Pe2 = calcendpoint(pEnd,rawSegs(5:6,segIndices(i,3)));
        len = sqrt(sum((Pe1-Pe2).^2));
        if len > lmax,
          lmax = len; imax = j;
        end;
      end;
      
      if lmax >= minlength,
        li = li + 1;
        lines(1,li) = lineID;                     % line ID
        lines(2,li) = nJointSegs;                 % number of contributing joint segments
        lines(3,li) = segIndices(i,2);				    % number of contributing raw segments
        lines(4,li) = nLinePoints;                % number of contributing raw data points
        lines(5:6,li) = rawSegs(5:6,segIndices(i,3));           % alpha,r
        lines(7,li) = rawSegs(7,segIndices(i,3)) + saacompens;  % saa
        lines(8,li) = rawSegs(8,segIndices(i,3)) + srrcompens;  % srr
        lines(9,li) = rawSegs(9,segIndices(i,3));               % sar
        lines(10,li) = sum(rawSegs(7:8,segIndices(i,3))); % trace of covariance matrix
        for j = 1:nJointSegs,
          si = si + 1;
          if j>1,
              disp('extractlines2: j>1')
          end
          lines(10+j,li) = si; 				  	        % IDs of contributing joint segments
          segs(1,si) = si; 					              % ID of joint segment
          segs(2,si) = lineID; 					          % ID of line the segment belongs to
          segs(3,si) = segIndices(i,2);	          % number of contributing raw segments
          nPoints = mod2(jSegIndices(j,3)-jSegIndices(j,2)+1, l);
          segs(4,si) = nPoints;					          % number of contributing raw data points
          segs(5:10, si) = lines(5:10,li);				% copy alpha,r and cov-matrix
          segs(11:12,si) = jSegIndices(j,2:3)';   % begin, end index of joint segment
          ibeg = jSegIndices(j,2);
          iend = jSegIndices(j,3);
          [pBeg(1),pBeg(2)] = pol2cart(scan(ibeg,1), scan(ibeg,2));
          [pEnd(1),pEnd(2)] = pol2cart(scan(iend,1), scan(iend,2));
          Pe1 = calcendpoint(pBeg,lines(5:6,li)); % Cartesian projection
          Pe2 = calcendpoint(pEnd,lines(5:6,li));
          segs(13:16,si) = [Pe1, Pe2]'; % Cartesian coordinates of endpoints
          % added by Trond M
          % delete segments close to robot origo...not valid segments
          DIST=0.10;
          %Pe=[Pe1 Pe2];
          d = distancePointEdge([0 0],segs(13:16,si)' );
          if d<DIST,
          %if any([sum(Pe(1:2).^2), sum(Pe(3:4).^2)] < DIST^2),
              segs(:,si)=[];
              si=si-1;
              % delete also corresponding line
              lines(:,li)=[];
              li=li-1;
              break
          end
          % end added
          
        end;
        
      end;
    end;
    nJointSegs = si;
    nLines = li;
    
    % Resort 'segs' such that it is sorted acc. to real succession of segments in scan
    if nLines > 0
      [dummy,jSegsisort] = sortrows(segs',11);
      tmp=[];
      for i = 1:nJointSegs,
        tmp(:,i) = segs(:,jSegsisort(i));
      end;
      segs = tmp;
      
      % ---------------------------------------------------------- %
      %%%%%%%%%%%  Generate Final Data Structures --> L  %%%%%%%%%%%
      % ---------------------------------------------------------- %
      
      % Put everything into the new map object L
      created = 0; iline = 1;
      for i = 1:nLines;
        % create map if not yet done
        if ~created,
          L = map('local map',0);
          created = 1;
        end;
        % get alpha-r values
        id    = lines(1,i);
        alpha = lines(5,i);
        r     = lines(6,i);
        saa   = lines(7,i);
        srr   = lines(8,i);
        sar   = lines(9,i);
        % create line feature

% added by Bj�rn Syvertsen
        hitIndex=0;
        for myIndex=1:size(segs,2)
            if( segs(2,myIndex) == id )
                hitIndex=myIndex;
                break;
            end;
        end;
                
        roboPoseCart = [roboPose(1) roboPose(2)];
        roboAngle    = roboPose(3);
        rotMatrix    = [cos(roboAngle) -sin(roboAngle);sin(roboAngle) cos(roboAngle)];
%         
%         ut1=rotMatrix
%         ut2=segs(13:14,myIndex)
%         ut3 = roboPoseCart
        
        startPoint   = (rotMatrix*segs(13:14,myIndex))'+roboPoseCart;
        endPoint     = (rotMatrix*segs(15:16,myIndex))'+roboPoseCart;
       
% end added by Bj�rn Syvertsen                   


        l = arlinefeature(id,[alpha;r],[saa sar; sar srr],[startPoint endPoint]);
        % add it to local map
        L = addentity(L,l);
      end;
      
      if created,
        X = get(L,'x');
        nL = length(X);
      else
        L = []; nL = 0;
      end;
      
%       if nL == 0, str1 = 'no'; str2 = 'lines';
%       elseif nL == 1, str1 = '1'; str2 = 'line';
%       else str1 = int2str(nL); str2 = 'lines';
%       end; disp([' ',str1,' ',str2,' extracted']);
      
      % Assign output
      if nargout == 1;
        varargout{1} = L;
      else
        varargout{1} = L;
        varargout{2} = segs;
        varargout{3} = lines;
      end;  
      
      
      % -------------------------------------- %
      %%%%%%%%%%  Plot final lines  %%%%%%%%%%%%
      % -------------------------------------- %
      
      if displocal,
        figure(3); clf; hold on; set(gca,'Box','on'); axis equal;
        xlabel('x [m]'); ylabel('y [m]'); title('Local map: extracted lines');
        plot(scan(:,2).*cos(scan(:,1)),scan(:,2).*sin(scan(:,1)),...
          'b.',...%'Color',[.6 .6 .6],
            'MarkerSize',6);
        drawreference(zeros(3,1),'R',1.0,'k');
          axisvec = axis;
          for i = 1:nLines,
            x1 = lines(6,i)*cos(lines(5,i)) - MUE*sin(lines(5,i));
            y1 = lines(6,i)*sin(lines(5,i)) + MUE*cos(lines(5,i));
            x2 = lines(6,i)*cos(lines(5,i)) + MUE*sin(lines(5,i));
            y2 = lines(6,i)*sin(lines(5,i)) - MUE*cos(lines(5,i));
            plot([x1 x2],[y1 y2],'LineWidth',1,'Color',[.4 .4 .4],'LineStyle','-.');
            %xtext = lines(6,i)*cos(lines(5,i)) + TXTSHIFT;
            %ytext = lines(6,i)*sin(lines(5,i)) + TXTSHIFT;
            %text(xtext,ytext,int2str(lines(1,i)));
          end;
          si = 0;
          for i = 1:nLines,                 % calc endpoints and plot final (joint) segments
            for j = 1:lines(2,i);           % for all joint segment of line i
              si = si + 1;
              Pe1 = segs(13:14,si)';
              Pe2 = segs(15:16,si)';
              
              
              plot([Pe1(1) Pe2(1)],[Pe1(2) Pe2(2)],'LineWidth',2.7,'Color',[.9 .5 .0]);
              plot(Pe1(1),Pe1(2),'kx','LineWidth',2,'MarkerSize',9);
              plot(Pe2(1),Pe2(2),'kx','LineWidth',2,'MarkerSize',9);
            end;
          end;
          % restore and enlarge axis vector slightly
          axis(1.1*axisvec);
      end;
        
    else
%        disp('extractlines: All found segments too short');
        if nargout == 3, varargout{2} = []; varargout{3} = []; end; varargout{1} = [];
    end;
   else % if nRawSegs > 0...
%      disp('extractlines: No lines found');
      if nargout == 3, varargout{2} = []; varargout{3} = []; end; varargout{1} = [];
   end;
else	
    disp('extractlines: Wrong input. Check your arguments');
    if nargout == 3, varargout{2} = []; varargout{3} = []; end; varargout{1} = [];
end;