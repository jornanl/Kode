function G=deletePointsFromMap(Gin)

G=Gin;

% delete pointfeatures close to segments
    X=get(Gin,'x');
    C=get(Gin,'c');
    
    
    % only consider point features here
%    [Gin,E]=exclude(Gin,'alpha,r line feature');
   
    
    pointf=X;
    Xg=X;
    %pointC=Cg;
    pointf(1)=[];
    %pointC(1,:)=[];
    %pointC(:,1)=[];
    f=length(Xg);
    indexf=[];
    while f>1,
        type=get(Xg{f},'Type');
        if ~strcmp(type,'point feature'),
            pointf(f-1)=[];
            
            %update covarians matrix 
            %pointC(f,:)=[];
            %pointC(:,f)=[];
        else
            Xg(f)=[];
            indexf(length(indexf)+1)=f;
            %update covarians matrix 
            %Cg(f,:)=[];
            %Cg(:,f)=[];
        end
        f=f-1;
        
    end
    indexf=sort(indexf);
    
    if ~isempty(pointf),   
        xyp=[];
        %Cg=get(G,'c');
        for p=1:length(pointf)
            xyp(p,1:2)=get(pointf{p},'x');
        end

        % fetch all segments from G
        segs=[];
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
            indInte=dist<0.05;
            pointf(indInte)=[];
            % update covarianse matrix
            %one=find(indInte==1);
            org=indexf(indInte);
            lenOrg=length(org);
            for o=1:length(org),  
                C(org(o),:)=[];
                C(:,org(o))=[];
            
                X(org(o))=[];
                org=org-1;
            end
            xyp(indInte,:)=[];
            
            indexf(length(indexf)-lenOrg+1:length(indexf))=[];
                 
            seg=seg-1;
            else
                break;
            end
        end
    end    
    
    G=set(G,'x',X,'c',C);
    