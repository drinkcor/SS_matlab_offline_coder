%% ************************** Documentation *******************************
% The clean-up phase consists of three steps that filter less significant signals. 
% To do so, compositions are analyzed under a couple of contexts (in a state??): 
% (1) composition�s time duration, (2) composition�s amplitude magnitude, and (3) 
% repletion patterns. 
%
% Time Duration Context
% 1.	If there is a composition where one of the two primitives that compose it is 
%       5 times smaller than the other (except for contacts c,pc,nc), then merge 
%       by changing the action composition label to correspond to the longer primitive:
%           o bpos/mpos/spos becomes �i'. 
%           o bneg/mneg/sneg becomes �d�. 
%           o const becomes �k�.
%
% Amplitude Value Context
% The analysis of amplitude value context pertains to the formation of alignment signals. 
% 1.	If there are primitives of types PC or NC AND if: 
%           o their amplitude is 10x smaller than the large amplitude registered in the 
%             assembly, then treat them as increase or decrease.
% 2.	If there is an adjustment followed by either an increase or a decrese with similar 
%       average value, merge them into an adjustment. 
% 3.	If there is a d||i||k with similar amplitude and an average value
%       within 50% of each other, merge as constant. 
%
% Repeated Compositions
% 1.	For all compositions, that are adjustments, if they are repeated in states 
%       3 or 4, then merge them. 
% 2.    For any actions that are not adjustments, if they repeat, merge them.
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
% Input Parameters:
% strategyType      - customize code according to strategies
% motComps:         - [actnClass,...
%                      avgMagVal,RMS_VAL,AMPLITUDE_VAL,...
%                      glabel1,glabel2,...
%                      T1S,T1_END,T2S,T2E,TAVG_INDEX]
%
% stateData:        - time at which states start. First entry (out of four)
%                     indicates the time at which the second state starts.
%                     Assumes the 5 states of the Pivot Approach.
% gradLabels        - vector array of ints [-4,-3,-2,-1,0,1,2,3,4] -- modified July 2012. Old>>cell array containing all of the primitive labels
% actionLbl         - vector array of ints [10,20,30,40,50,60,70,80] -- modified July 2012. Old>>cell array containing all the actions motion
%                     compositions
%**************************************************************************
function [hasNew_cmc, data_new, lookForRepeat, numRepeat, maxAmplitude, marker_cmc] = rt_MotCompsCleanUp(motComps,lookForRepeat, numRepeat, maxAmplitude, marker_cmc)

%% Initialization
    

    % Threshold values to examine whether the average magnitude value and
    % amplitude value for clean up's of adjustment/increase/decrease
    % combinations
    AMP_WINDOW_AD_ID = 0.50;
    MAG_WINDOW_AD_ID = 0.50; 
    
    % Threshold used for combs of increase/constant/decrease
    AMP_WINDOW_IKD = 0.9;
    MAG_WINDOW_IKD = 1.00; 
    
    % Threshold used for combs of absolutely dominate
    AMP_WINDOW_DOM = 0.8;
    MAG_WINDOW_DOM = 0.8; 
    TIME_WINDOW_DOM = 0.8;
    
    TIME_DURATION  = 0;
    
    % Threshold used for combs of pimp/nimp
    AMP_WINDOW_PC_NC = 0.1; 
    
%%  GRADIENT PRIMITIVES

    % CONSTANTS FOR gradLabels (defined in fitRegressionCurves.m)
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
%%  MOTION COMPOSITION

    % Action Indeces
    adjustment      = 1;    % a==1
    increase        = 2;    % i==2
    decrease        = 3;    % d==3
    constant        = 4;    % k==4
    pos_contact     = 5;    % pc==5
    neg_contact     = 6;    % nc==6
    contact         = 7;    % c==7
    %unstable        = 8;   % u==8

    % mot Comps Structure Indeces
    ACTN_LBL         = 1;   % action class
    AVG_MAG_VAL      = 2;   % average value
    RMS_VAL          = 3;   % rms value, which is max value too
    AMPLITUDE_VAL    = 4;   % amplitude value 
    
    % Labels
    P1LBL = 5; P2LBL = 6;   % label indeces for both primitives
    
    % Time Indeces
    T1S = 7; T1E = 8;
    T2S = 9; T2E = 10;    
    TAVG_INDEX   = 11;
       
%%  DURATION VARIABLES 
   
    % Threshold for merging two primitives according to lengthRatio
    lengthRatio = 5;  % Empirically set
    startIndex = 0;

    hasNew_cmc   = 0;
    data_new    = [0 0 0 0 0 0 0 0 0 0 0];
    
    
    % Create string array:
    gradLabels = [ 'bpos';   ... % big   pos grads
                   'mpos';   ... % med   pos grads
                   'spos';   ... % small pos grads
                   'bneg';   ... % big   neg grads
                   'mneg';   ... % med   neg grads
                   'sneg';   ... % small neg grads
                   'cons';  ... % constant  grads
                   'pimp';   ... % large pos grads
                   'nimp';   ... % large neg grads
                   'none'];
               
    actionLbl       = [1,2,3,4,5,6,7,8];                  % This array has been updated to be an int vector
        
%%  Repeated Compositions

%% 1) REPEATED ADJUSTMENT'S {aaa...)
%%  Inintialization     
    
    % Set the simulation time step

    % SIM_TIME_STEP   = 1/rate;                % Uses OpenHRP3.0 version

    checkPairs      = 0;
    keepCheck       = true;
            
    % Do this until there are no more repitions in the entire data
    % (multiple loops)    
    
 %% (1) Delete repeated MC    
    i     = marker_cmc;
    match = i+1;
    
    
    %%  AMPLITUDE VALUE CONTEXT
    if(motComps(i,AMPLITUDE_VAL)>maxAmplitude)
        maxAmplitude = motComps(i,AMPLITUDE_VAL);
    end

%%  1) If there are pos_contacts or neg_contacts whose amplitude is 10x
    % smaller than the current largest amplitude convert them into 'i' or 'd'
    % correspondingly.  
    
    % We avoid directly modifying the raw data. Because we don't want to affect  
    % raw data(motComps)'s index, on which we depended to do real-time processing. 
    % Thus, although we modify motComps here, we don't return(save) it back. But 
    % the effect of the modifying was recorded, like repeat_Lbl and other actionLbl_index, 
    % and then sent to merge composition function to generate a correct result.

    % Find positive contacts
    if(intcmp(motComps(i,ACTN_LBL),actionLbl(pos_contact)))
        if(abs(motComps(i,AMPLITUDE_VAL))<AMP_WINDOW_PC_NC*abs(maxAmplitude))
            motComps(i,ACTN_LBL)=actionLbl(increase);
        end

    % Find negative contacts
    elseif(intcmp(motComps(i,ACTN_LBL),actionLbl(neg_contact)))
        if(abs(motComps(i,AMPLITUDE_VAL))<AMP_WINDOW_PC_NC*abs(maxAmplitude))
            motComps(i,ACTN_LBL)=actionLbl(decrease);
        end            
    end
    
    
%%  Compare labels (int type's) of contiguous MCs)
    if (lookForRepeat)
        
        if( intcmp(motComps(i,ACTN_LBL),motComps(match,ACTN_LBL)) )
            lookForRepeat   = true;
            numRepeat       = numRepeat+1;
            marker_cmc      = marker_cmc+1;
        else
            % Merge as many primitives as are repeated
            nextPrimitive   = numRepeat;
            startPrimitive  = i-numRepeat;
 
            repeat_Lbl      = motComps(i,ACTN_LBL);
            data_new        = rt_MergeCompositions(startPrimitive,motComps,actionLbl,repeat_Lbl,nextPrimitive); % The third argument is gradLabels but it is not used.
            hasNew_cmc      = 1;
            marker_cmc       = marker_cmc+1;
            
            lookForRepeat   = false;
            numRepeat       = 0;    
        end 
    else
        if( intcmp(motComps(i,ACTN_LBL),motComps(match,ACTN_LBL)) )
            lookForRepeat   = true;
            numRepeat       = numRepeat+1;
            marker_cmc      = marker_cmc+1;
        else
            % Go ahead to check (2):Time duration context
            lookForRepeat   = false;
            numRepeat       = 0;
            checkPairs      = 1;
        end         

    end
    
    
    if(checkPairs)
        
        % if we need to check pair, we also need to check the i+1 amplitude context        
        if(motComps(match,AMPLITUDE_VAL)>maxAmplitude)
            maxAmplitude = motComps(match,AMPLITUDE_VAL);
        end
        
        if(intcmp(motComps(match,ACTN_LBL),actionLbl(pos_contact)))
            if(abs(motComps(match,AMPLITUDE_VAL))<AMP_WINDOW_PC_NC*abs(maxAmplitude))
                motComps(match,ACTN_LBL)=actionLbl(increase);
            end
        elseif(intcmp(motComps(match,ACTN_LBL),actionLbl(neg_contact)))
            if(abs(motComps(match,AMPLITUDE_VAL))<AMP_WINDOW_PC_NC*abs(maxAmplitude))
                motComps(match,ACTN_LBL)=actionLbl(decrease);
            end            
        end
       
        %%	1) For patterns of a+i or a+d or d+a or d+i, merge if similar. 
        % If there are contiguous i+d or d+i with similar average values
        % (within 10% of each other),merge them into an adjustment.    

        % If 'a+i' or 'a+d' or 'i+a' or 'd+a' or 'i+d' or 'd+i'
        if( (intcmp(motComps(i,ACTN_LBL),actionLbl(adjustment)) && intcmp(motComps(match,ACTN_LBL),actionLbl(increase))) || ...
                (intcmp(motComps(i,ACTN_LBL),actionLbl(adjustment)) && intcmp(motComps(match,ACTN_LBL),actionLbl(decrease))) || ...
                (intcmp(motComps(i,ACTN_LBL),actionLbl(increase)) && intcmp(motComps(match,ACTN_LBL),actionLbl(adjustment))) || ...
                (intcmp(motComps(i,ACTN_LBL),actionLbl(decrease)) && intcmp(motComps(match,ACTN_LBL),actionLbl(adjustment))) || ...
                (intcmp(motComps(i,ACTN_LBL),actionLbl(decrease))&& intcmp(motComps(match,ACTN_LBL),actionLbl(increase))) || ...
                (intcmp(motComps(i,ACTN_LBL),actionLbl(increase))&& intcmp(motComps(match,ACTN_LBL),actionLbl(decrease))) )
            
            % If they have the similar amplitude (50%)
            perc1 = rt_computePercentageThresh(motComps,i,AMPLITUDE_VAL,AMP_WINDOW_AD_ID);
            if(perc1)
                
                % If their average value is within 100% of each other
                perc2 = rt_computePercentageThresh(motComps,i,AVG_MAG_VAL,MAG_WINDOW_AD_ID);
                if(perc2)
                    
                    data_new    = rt_MergeCompositions(i,motComps,actionLbl,adjustment,1);
                    hasNew_cmc  = 1;
                    marker_cmc  = marker_cmc+2;
                    
                    keepCheck   = false;
                end
            end
        end
    
        
        %% 2) ik/ki/dk/kd compositions that are contiguous that have similar amplitude and their avg value is within 50% to each other, merge as constant
        if(keepCheck)
            if( (intcmp(motComps(i,ACTN_LBL),actionLbl(increase))&& intcmp(motComps(match,ACTN_LBL),actionLbl(constant))) || ...
                    (intcmp(motComps(i,ACTN_LBL),actionLbl(constant))&& intcmp(motComps(match,ACTN_LBL),actionLbl(increase))) || ...
                    (intcmp(motComps(i,ACTN_LBL),actionLbl(decrease))&& intcmp(motComps(match,ACTN_LBL),actionLbl(constant))) || ...
                    (intcmp(motComps(i,ACTN_LBL),actionLbl(constant))&& intcmp(motComps(match,ACTN_LBL),actionLbl(decrease))) )
                
                % If they have the similar amplitude (150%)
                perc1 = rt_computePercentageThresh(motComps,i,AMPLITUDE_VAL,AMP_WINDOW_IKD);
                if(perc1)
                    % If their average value is within 100% of each other
                    perc2 = rt_computePercentageThresh(motComps,i,AVG_MAG_VAL,MAG_WINDOW_IKD);
                    if(perc2)
                        data_new    = rt_MergeCompositions(i,motComps,actionLbl,constant,1);
                        hasNew_cmc  = 1;
                        marker_cmc  = marker_cmc+2;
                        
                        keepCheck   = false;
                    end
                end
            end
        else
            % Do nothing here
        end
        
        
        %% 3) TIME DURATION CONTEXT - MERGE AND MODIFY Composite Actions
        if(keepCheck)
            % If it is not a contact label compare the times.
            if(~intcmp(motComps(i,ACTN_LBL),actionLbl(pos_contact)) && ~intcmp(motComps(match,ACTN_LBL),actionLbl(pos_contact)) &&...
                    ~intcmp(motComps(i,ACTN_LBL),actionLbl(neg_contact)) && ~intcmp(motComps(match,ACTN_LBL),actionLbl(neg_contact)) &&...
                    ~intcmp(motComps(i,ACTN_LBL),actionLbl(contact)) && ~intcmp(motComps(match,ACTN_LBL),actionLbl(contact)) )
                
                [perc1,biggerOne_amp] = rt_computePercentageThresh(motComps,i,AMPLITUDE_VAL,AMP_WINDOW_DOM);
                % If their amplitude isn't similar
                if(~perc1)
                    [perc2,biggerOne_mag] = rt_computePercentageThresh(motComps,i,AVG_MAG_VAL,MAG_WINDOW_DOM);
                    % If their average value isn't similar
                    if(~perc2)
                        [perc3,biggerOne_time] = rt_computePercentageThresh(motComps,i,TIME_DURATION,TIME_WINDOW_DOM);
                        % If their time duration isn't similar
                        if(~perc3)
                            % If one MC is absolutely dominating
                            if((biggerOne_amp==biggerOne_mag)&&(biggerOne_mag==biggerOne_time))
                                if(biggerOne_amp==-1)
                                    action_Lbl = motComps(i,ACTN_LBL);
                                elseif (biggerOne_amp==1)
                                    action_Lbl = motComps(match,ACTN_LBL);
                                end
                                data_new    = rt_MergeCompositions(i,motComps,actionLbl,action_Lbl,1);
                                hasNew_cmc  = 1;
                                marker_cmc  = marker_cmc+2;
                                
                                keepCheck   = false;
                                
                            end
                        end                       
                    end
                end
            end
        else
            % Do nothing here
        end
        
        if(keepCheck==true)        
                % Can't find matched pairs, save the current MC
                data_new    = motComps(i,:);
                hasNew_cmc  = 1;
                marker_cmc  = marker_cmc+1;            
        end
             
end