%% ************************** Documentation *******************************
% The primitive clean-up phase filters primitives based on: (1) repeated primitives,
% and, (2) time duration.
%
% (1) Repeated Primitives: merge as many primitives in a row as necessary
% if the label is repeated. Then deleted the elements who were merged
% up-front in the array.
%
% (2) Time Duration & Amplitude Context: for two contiguous primitives, if one 
%  primitive is 5x longer and 2x larger than the other then we absorb it 
% (except for pimp/nimp impulses).
%
% The reason for this, is that if there is a "big" jump in amplitude even if
% it is of short duration it is important. We have learned that not only
% contacts but also also signals of smaller gradients can be significant.
%
%--------------------------------------------------------------------------
% For Reference: Structures and Labels
%--------------------------------------------------------------------------
% Primitives = [bpos,mpos,spos,bneg,mneg,sneg,cons,pimp,nimp,none]      % Represented by integers: [1,2,3,4,5,6,7,8,9,10]  
% statData   = [dAvg dMax dMin dStart dFinish dGradient dLabel]
%--------------------------------------------------------------------------
% actionLbl  = ['a','i','d','k','pc','nc','c','u','n','z'];             % Represented by integers: [1,2,3,4,5,6,7,8,9,10]  
% motComps   = [nameLabel,avgVal,rmsVal,amplitudeVal,
%               p1lbl,p2lbl,
%               t1Start,t1End,t2Start,t2End,tAvgIndex]
%--------------------------------------------------------------------------
% llbehLbl   = ['FX' 'CT' 'PS' 'PL' 'AL' 'SH' 'U' 'N'];                 % Represented by integers: [1,2,3,4,5,6,7,8]
% llbehStruc:  [actnClass,...
%              avgMagVal1,avgMagVal2,AVG_MAG_VAL,
%              rmsVal1,rmsVal2,AVG_RMS_VAL,
%              ampVal1,ampVal2,AVG_AMP_VAL,
%              mc1,mc2,
%              T1S,T1_END,T2S,T2E,TAVG_INDEX]
%--------------------------------------------------------------------------
%
% Inputs:
% stateData:        - time at which states start. First entry (out of four)
%                     indicates the time at which the second state starts.
%                     Assumes the 5 states of the Pivot Approach.
% gradLabels        - column string vector containing all of the primitive labels
%**************************************************************************
function [hasnew_pc,data_new,lookForRepeat,numberRepeated,marker_pc] = rt_primitivesCleanUp(statData,lookForRepeat,numberRepeated,marker_pc)

%% Initialization

    
%%  GRADIENT PRIMITIVES

%   % CONSTANTS FOR gradLabels (defined in fitRegressionCurves.m)
    BPOS            = 1;        % big   pos gradient
    MPOS            = 2;        % med   pos gradient
    SPOS            = 3;        % small pos gradient
    BNEG            = 4;        % big   neg gradient
    MNEG            = 5;        % med   neg gradient
    SNEG            = 6;        % small neg gradient
    CONST           = 7;        % constant  gradient
    PIMP            = 8;        % large pos gradient 
    NIMP            = 9;        % large neg gradient
    %NONE            = 10;       % none
    
%     gradLabels = { 'bpos',   ... % big   pos grads
%                    'mpos',   ... % med   pos grads
%                    'spos',   ... % small pos grads
%                    'bneg',   ... % big   neg grads
%                    'mneg',   ... % med   neg grads
%                    'sneg',   ... % small neg grads
%                    'const',  ... % constant  grads
%                    'pimp',   ... % large pos grads
%                    'nimp',   ... % large neg grads
%                    'none'};    

%   % primitives Structure Indeces
%   AVG_MAG_VAL      = 1;   % average value of primitive
%   MAX_VAL          = 2;   % maximum value of a primitive
%   MIN_VAL          = 3;   % minimum value of a primitive   

    % Time Indeces
    T1S = 4; T1E = 5;     
    
    % Amplitude Indeces
    mxAmp=2;  minAmp=3;
    GRAD_LBL    = 7;
%%  DURATION VARIABLES    
    % Threshold for merging two primitives according to lengthRatio
    lengthRatio     = 5;  % Empirically set
    amplitudeRatio = 5;

    checkRatio = 0;     % flag, 0 means that no need to check ratio, 1 means need to check.
    
    hasnew_pc = 0;
    data_new = [0 0 0 0 0 0 0];
    
%% (1) Delete repeated primitives    
 % no repeatition flag
    i = marker_pc;
    j = i+1;
            
%%  Compare labels (int type's) of contiguous primitives)
    if (lookForRepeat)
        
        if( intcmp(statData(i,7),statData(j,7)) )
            lookForRepeat   = true;
            numberRepeated  = numberRepeated+1;
            marker_pc       = marker_pc+1;
        else
            % Merge as many primitives as are repeated
            nextPrimitive   = numberRepeated;
            startPrimitive  = i-numberRepeated;
            data_new        = rt_MergePrimitives(startPrimitive,statData,nextPrimitive); % The third argument is gradLabels but it is not used.
            hasnew_pc       = 1;
            
            lookForRepeat   = false;
            numberRepeated  = 0;
            marker_pc       = marker_pc+1;
        end 
    else
        if( intcmp(statData(i,7),statData(j,7)) )
            lookForRepeat   = true;
            numberRepeated  = numberRepeated+1;
            marker_pc       = marker_pc+1;
        else
            % Go ahead to check (2):Time duration context
            lookForRepeat   = false;
            numberRepeated  = 0;
            checkRatio      = 1;
        end         

    end
    
    
    
%%  (2) TIME DURATION CONTEXT - MERGE AND MODIFY Primitives
    % If a primitive's amplitude and time duration are both 5x longer than the adjacent one, merge the adjacent one to it.
    
	if(checkRatio) 
        
        % If it is not a contact label compare the times.
        if( ~intcmp(statData(i,GRAD_LBL),PIMP) && ...
                ~intcmp(statData(i,GRAD_LBL),NIMP) )                
            
            % (1) Get Amplitude of Primitives
            amp1 = abs(statData(i,mxAmp)-statData(i,minAmp));       % Absolute value of amplitude difference of first primitive
            amp2 = abs(statData(i+1,mxAmp)-statData(i+1,minAmp));   % bsolute value of amplitude difference of second primitive
            
            % Compute ratio of 2nd primitive vs 1st primitive
            ampRatio = amp1/amp2;
                         
            if ( ampRatio >= amplitudeRatio && ampRatio~=inf  )                
                % (2) Get Duration of primitives inside compositions
                p1time = statData(i,T1E)-statData(i,T1S);       % Get duration of first primitive
                p2time = statData(i+1,T1E)-statData(i+1,T1S);   % Get duration of second primitive    
                
                ratio = p1time/p2time;

                % Merge according to the ratio            
                if(ratio~=0 && ratio~=inf && ratio > lengthRatio)
                    thisPrim = 0;            % First primitive is longer
                    data_new  = rt_MergePrimitives(i,statData,thisPrim);
                    marker_pc = marker_pc+2;
                    hasnew_pc = 1;
                else
                    data_new  = statData(i,:);
                    marker_pc = marker_pc+1;
                    hasnew_pc = 1;
                end 
            elseif ( ampRatio <= inv(amplitudeRatio) && ampRatio~=0  )
                % (2) Get Duration of primitives inside compositions
                p1time = statData(i,T1E)-statData(i,T1S);       % Get duration of first primitive
                p2time = statData(i+1,T1E)-statData(i+1,T1S);   % Get duration of second primitive    
                
                ratio = p1time/p2time;
                
                % Merge according to the ratio 
                if(ratio~=0 && ratio~=inf && ratio < inv(lengthRatio))
                    nextPrim = 1;            % Second primitive is longer
                    data_new  = rt_MergePrimitives(i,statData,nextPrim);
                    marker_pc = marker_pc+2;
                    hasnew_pc = 1;
                else
                    data_new  = statData(i,:);
                    marker_pc = marker_pc+1;
                    hasnew_pc = 1;
                end     
            else
                data_new  = statData(i,:);
                marker_pc = marker_pc+1;
                hasnew_pc = 1;
            end
        else
            data_new  = statData(i,:);
            marker_pc = marker_pc+1;
            hasnew_pc = 1;
        end
    end

end