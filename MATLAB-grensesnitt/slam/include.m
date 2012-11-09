function [G,E]=include(Gin,string)
%INCLUDE
%   Include takes in a map Gin and include features in Gin named as string.
%   String can be either 'alpha,r line feature' or 'point feature'. Include
%   returns a updated map G with the included features and a map E with the excluded features. 

    
if isa(Gin,'map') && (strcmp(string,'alpha,r line feature') || strcmp(string,'point feature')), 
    E=Gin;
    X=get(Gin,'x');
    C = get(Gin,'c');
    
    Tmp=X;
    Ctmp=C;
    Tmp(1)=[];
    Ctmp(1,:)=[];
    Ctmp(:,1)=[];
    f=length(X);
    while f>1,
        type=get(X{f},'Type');
        if ~strcmp(type,string),
            X(f)=[];
            %update covarians matrix 
            C(f,:)=[];
            C(:,f)=[];
            
        else
            Tmp(f-1)=[];
            %update covarians matrix
            Ctmp(f-1,:)=[];
            Ctmp(:,f-1)=[];
        end
        f=f-1;
    end
    E=set(E,'x',Tmp,'c',Ctmp);
    G=set(Gin,'x',X,'c',C);
else
   error('exclude: Wrong input arguments')
end