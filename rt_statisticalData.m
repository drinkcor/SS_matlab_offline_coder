% Return a variety of statistical data from the plot that is saved as a
% primitive or segment. The data structure is:
% statData[dAvg dMax dMin dStart dFinish dGradient dLabel]. 
%
% The label element can contain any of the following: [bpos,mpos,spos,bneg,mneg,sneg,cons,pimp,nimp,none]
%
% Inputs:
% wStart        - what force element do we want to start off with, Fx=1, Mz=6.
% wFinish       - what force element do we want to finish with.
% Data          - is the y-values of the segmented linear fit curve
% domain        - is the difference between the maximum and minimum y-value for the whole
%                 force data for that axis 
% polyCoeffs    - coefficients that describe the fitted line
% StrategyType  - tells whether using PA10-PivotApproach or HIRO-SideApproach
% forceAxisIndex- what force axes am i on.
%
% Outputs:
% dAvg          - average int value of a primitive. ie. (maxval-minval)/2.
% dMax          - max int value in a primitive.
% dMin          - min int value in a primitive.
% dStart        - time at which primitive starts
% dEnd          - time at which primitive endds
% dGradient     - gradient value of primitive
% dLabel        - gradient integer label
function [dAvg,dMax,dMin,dStart,dFinish,dGradient,dLabel]=rt_statisticalData(wStart,    wFinish,...
                                                                          Data,      polyCoeffs,...
                                                                          FolderName,StrategyType,forceAxisIndex)
    
%% Compute Statistical Parameters of the poly fitted Data   
    dMax        = max(Data);                            % 2)
    dMin        = min(Data);                            % 3)
    if(dMax>=0 && dMin >=0 || dMax<=0 && dMin<=0)       % 1)
        dAvg = (dMax+dMin)/2;                
    % One number positive, the other negative.         
    else
        dAvg = dMax - ( (abs(dMax)+abs(dMin))/2 ); 
    end
    dStart      = wStart;                               % 4)
    dFinish     = wFinish;                              % 5)
    dGradient   = polyCoeffs(1);                        % 6)
    
%% Label Data   
    % Label each segment based on gradient magnitude:
    % [pimp,bpos,mpos,spos,const,sneg,mneg,bneg,nimp]
    dLabel = rt_GradientClassification(dGradient, ...
                                    FolderName,StrategyType,forceAxisIndex);  % 7)    
    
%% Convert dLabel from string to int
    dLabel = gradLbl2gradInt(dLabel);
end