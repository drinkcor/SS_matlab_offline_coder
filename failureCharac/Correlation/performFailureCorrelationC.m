% performFailureCorrelationC
% This function will compare the failure mean ratio of the current
% training/testing example and see if it falls within the ratio boundaries
% of all other exemplars: MyR, MzR1, MzR23, FzA1, FzA2, FzA3. If the current mean
% ratio does fall in those boundaries, a 0 will be returned, otherwise a 1
% will be returned. You'd expect a 0 corresponding to the deviation
% direction that are executed.
%
%--------------------------------------------------------------------------
% Inputs
%--------------------------------------------------------------------------
% currMean      - mean computed for current exemplar. 
% MyR           - MyR 4x2 struc. that measures divergence in XDir
% MzR           - MzR 8x2 struc. that measures divergence in YDir
% FzA           - FzA 12x2 struc. that measures divergence in xYall Dir.
% isTrainStruc  - [isTrainingFailure?,XDirTrainingFlag,YDirTrainingFlag,xYallDirTrainingFlag]
% whichAxis     - Passes a flag indicating whether we are analyzing My for
%                 XDir divergence, Mz for YDirDivergence, or FzA for xDirDivergence.
%--------------------------------------------------------------------------
% Outputs
%--------------------------------------------------------------------------
% All outputs of this function are correlation parameters. They indicate
% whether the current mean is within the bounds of trained exemplars (MyR,
% MzR1, MzR23, FzA1, FzA2, and FzA3). If a value of 0, it indicates that
% the current mean is within bounds of that exemplars. You should normally
% see one zero and many ones.
%-----------------------------------------------------------------------------------------------------------------
function [MyR1,MzR1,MzR23,FzA1,FzA2,FzA3]=performFailureCorrelationC(currMean,MyR,MzR,FzA,isTrainStruc,whichAxis)

    %% Create index values for historical averaged data: counters, means, upper_bounds, and lower_bounds
    devSum=sum(isTrainStruc(2:4));
    [~,~,col]=returnDivergenceMeanSubGroupDataC(devSum,whichAxis);
    
    % MyR
    %MyRc=MyR(1,col); 
    MyRm=MyR(2,col); MyR_UB=MyR(3,col)/MyRm; MyR_LB=MyR(4,col)/MyRm;

    % MzR
    % 1D
    %MzR1c=MzR(1,col);  
    MzR1m=MzR(2,col);  MzR1_UB=MzR(3,col)/MzR1m;  MzR1_LB=MzR(4,col)/MzR1m;
    % 2D or 3D
    %MzR23c=MzR(5,col); 
    MzR23m=MzR(6,col); MzR23_UB=MzR(7,col)/MzR23m; MzR23_LB=MzR(8,col)/MzR23m;

    % FzA
    % 1D
    %FzA1c=FzA(1,col); 
    FzA1m=FzA(2,col);  FzA1_UB=FzA(3,col)/FzA1m;  FzA1_LB=FzA(4,col)/FzA1m;
    
    %FzA2c=FzA(5,col); 
    FzA2m=FzA(6,col);  FzA2_UB=FzA(7,col)/FzA2m;  FzA2_LB=FzA(8,col)/FzA2m;
    
    %FzA3c=FzA(9,col); 
    FzA3m=FzA(10,col); FzA3_UB=FzA(11,col)/FzA3m; FzA3_LB=FzA(12,col)/FzA3m;

    notCorrelated   =0;
    Correlated      =1;
    %-----------------------------------------------------------------------------------------------------------------------------------------------
    ratio=currMean/MyRm;        if( ratio>MyR_UB   || ratio < MyR_LB );       MyR1=notCorrelated;         else MyR1=Correlated;        end; 
    %-----------------------------------------------------------------------------------------------------------------------------------------------
    ratio=currMean/MzR1m;       if( ratio>MzR1_UB  || ratio < MzR1_LB  );     MzR1=notCorrelated;         else MzR1=Correlated;        end;  
    ratio=currMean/MzR23m;      if( ratio>MzR23_UB || ratio < MzR23_LB );     MzR23=notCorrelated;        else MzR23=Correlated;       end;
    %-----------------------------------------------------------------------------------------------------------------------------------------------
    ratio=currMean/FzA1m;       if( ratio>FzA1_UB  || ratio < FzA1_LB );      FzA1=notCorrelated;         else FzA1=Correlated;        end;  
    ratio=currMean/FzA2m;       if( ratio>FzA2_UB  || ratio < FzA2_LB );      FzA2=notCorrelated;         else FzA2=Correlated;        end;
    ratio=currMean/FzA3m;       if( ratio>FzA3_UB  || ratio < FzA3_LB );      FzA3=notCorrelated;         else FzA3=Correlated;        end;  
    %-----------------------------------------------------------------------------------------------------------------------------------------------
end