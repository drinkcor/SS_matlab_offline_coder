%% ************************** Documentation *******************************
% The clean-up phase consists of three steps that filter less significant signals. 
% To do so, compositions are analyzed under three contexts: 
% 1) Duration Context: if one llb is 5 times longer than another merge, as long as the magnitude of 
% the other is not 5 times bigger either; 
% 2) Composition's Amplitude Magnitude, and 
% 3) Repetition patterns. 
%
% Duration Value Context
%
% Amplitude Value Context
% The analysis of amplitude value context pertains to the formation of alignment signals. 
% 1.	If there is push-pull or pull-push of similar amplitude and similar average value, then turn into a ALIGN.
% 2.	If there is a shift/alignment followed by a shift/alignment of smaller amplitude respectively, convert into an alignment.
% 3.	If there is a pattern ALIGN/PS, ALIGN/PL or in reverse-order which have about the same amplitude and average value, merge as align. 
%   	a. Same as above with SHIFT. 
% 4.    If there is a push-fix/fix-push or pull-fix/fix-pull of similar amplitude and similar average value then turn into a fix.
%
% Repeated Patterns
% 1.	If there are any repeated signals merge. 
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
% Input Parameters:
% llbehStruc:       - [actnClass,...
%                      avgMagVal1,avgMagVal2,AVG_MAG_VAL,
%                      rmsVal1,rmsVal2,AVG_RMS_VAL,
%                      ampVal1,ampVal2,AVG_AMP_VAL,
%                      mc1,mc2,
%                      T1S,T1_END,T2S,T2E,TAVG_INDEX]
%
% actionLbl:        - different possible actions for motion compositions.
%**************************************************************************
function [hasNew_llbc,data_new,lookForRepeat,numRepeat,marker_llbc] = rt_llbehCompositionCleanUp(llbehStruc,lookForRepeat,numRepeat,marker_llbc)

%% Initialization

    % Get dimensions of llbehStruc
    r = size(llbehStruc);    

%%  Motion Composition Actions
%    adjustment      = 1;    % a
%    increase        = 2;    % i
%    decrease        = 3;    % d
%    constant        = 4;    % k
%    pos_contact     = 5;    % pc
%    neg_contact     = 6;    % nc
%    contact         = 7;    % c
     unstable        = 8;    % u
%    noise           = 9;    % n
     none            = 10;   % z
%    actionLbl       = ('a','i','d','k','pc','nc','c','u','n','z');  % String representation of each possibility in the actnClass set.                     
     actionLbl       = [1,2,3,4,5,6,7,8,9,10];                  % This array has been updated to be an int vector
     
%%  Low-Level Behaviors
	FIX     = 1;        % Fixed in place
    CONTACT = 2;        % Contact
    PUSH    = 3;        % Push
    PULL    = 4;        % Pull
    ALIGN   = 5;        % Alignment
    SHIFT   = 6;        % Shift
   %UNSTABLE= 7;        % Unstable
   %NOISE   = 8;        % Noise
   %llbehLbl= ('FX' 'CT' 'PS' 'PL' 'AL' 'SH' 'U' 'N'); % ('fix' 'cont' 'push' 'pull' 'align' 'shift' 'unstable' 'noise');
    llbehLbl= [ 1,   2,   3,   4,   5,   6,   7,  8];
    
    % llbehStruc Indeces
    behLbl          = 1;   % action class
%   averageVal1     = 2;   % averageVal1
%   averageVal2     = 3;
    AVG_MAG_VAL     = 4;
%   rmsVal1         = 5;
%   rmsVal2         = 6;
%   AVG_RMS_VAL     = 7;
%   ampVal1         = 8;
%   ampVal2         = 9;
    AVG_AMP_VAL     = 10;
%   mc1             = 11;
    mc2             = 12;    
    T1S             = 13; 
    T1E             = 14;
%   T2S             = 15; 
    T2E             = 16;    
%   TAVG_INDEX      = 17;
       
%%  DURATION VARIABLES 
   
    % Threshold for merging two primitives according to lengthRatio
    lengthRatio = 5;  % Empirically set
    
%% WINDOW THRESHOLDS

    % For amplitude windows and average value windows for combinations of ALIGN/PS/PL
    AMP_WINDOW_ALIGN_PS_PL = 0.5;
    MAG_WINDOW_ALIGN_PS_PL = 1.0;
    
    % For push pull combinations
    AMP_WINDOW_PS_PL = 1.5;
    MAG_WINDOW_PS_PL = 1.0;
    
    % For FIX push pull combinations
    AMP_WINDOW_FIX = 0.75;
    MAG_WINDOW_FIX = 0.75;
     
    hasNew_llbc    = 0;
    data_new       = [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0];
    
    checkPairs      = 0;
    keepCheck       = true;
            
    % Do this until there are no more repitions in the entire data
    % (multiple loops)    
    
 %% (1) Delete repeated MC    
    index   = marker_llbc;
    match   = index+1;
    
    %%  1) Merge repeated signals 
    if (lookForRepeat)
        if( intcmp(llbehStruc(index,behLbl),llbehStruc(match,behLbl)) )
            lookForRepeat   = true;
            numRepeat       = numRepeat+1;
            marker_llbc     = marker_llbc+1;
        else
            % Merge as many primitives as are repeated
            nextPrimitive   = numRepeat;
            startPrimitive  = index-numRepeat;
 
            repeat_Lbl      = llbehStruc(index,behLbl);
            data_new        = rt_MergeLowLevelBehaviors(startPrimitive,llbehStruc,llbehLbl,repeat_Lbl,nextPrimitive); % The third argument is gradLabels but it is not used.
            hasNew_llbc     = 1;
            marker_llbc     = marker_llbc+1;
            
            lookForRepeat   = false;
            numRepeat       = 0;    
        end 
    else
        if( intcmp(llbehStruc(index,behLbl),llbehStruc(match,behLbl)) )
            lookForRepeat   = true;
            numRepeat       = numRepeat+1;
            marker_llbc     = marker_llbc+1;
        else
            % Go ahead to check (2):Time duration context
            lookForRepeat   = false;
            numRepeat       = 0;
            checkPairs      = 1;
        end         

    end
    
    if(checkPairs)
        
        %% 2) TIME DURATION CONTEXT - MERGE AND MODIFY
        %% For all behavior pairs that are not contact behaviors or have
        %% unstable or noisy behaviors (has a none or 'z' composition) inside of them.
        if((~intcmp(llbehStruc(index,behLbl),llbehLbl(CONTACT)) && ~intcmp(llbehStruc(match,behLbl),llbehLbl(CONTACT))) && ...
                (~intcmp(llbehStruc(index,mc2),actionLbl(unstable)) && ~intcmp(llbehStruc(match,mc2),actionLbl(unstable))) && ...
                (~intcmp(llbehStruc(index,mc2),actionLbl(none)) && ~intcmp(llbehStruc(match,mc2),actionLbl(none))) )
            
            % (1) Get Amplitude of Primitives
            amp1 = abs(llbehStruc(index,AVG_AMP_VAL));              % Absolute value of amplitude difference of first LLB
            amp2 = abs(llbehStruc(match,AVG_AMP_VAL));         % Absolute value of amplitude difference of second LLB
            
            % Compute ratio of 2nd primitive vs 1st primitive
            ampRatio = amp1/amp2;
            
            if ( ampRatio >= lengthRatio && ampRatio~=inf  )
                % (2) Get Duration of primitives inside compositions
                p1time = llbehStruc(index,T2E)-llbehStruc(index,T1S);   % Get duration of first primitive
                p2time = llbehStruc(match,T2E)-llbehStruc(match,T1S);   % Get duration of second primitive
                
                ratio = p1time/p2time;
                
                % Merge according to the ratio
                if(ratio~=0 && ratio~=inf && ratio > lengthRatio)
                    MATCH_FLAG  = 1;            % First primitive is longer
                    LLBEH_LBL   = llbehStruc(index,behLbl);
                    data_new    = rt_MergeLowLevelBehaviors(index,llbehStruc,llbehLbl,LLBEH_LBL,MATCH_FLAG);
                    marker_llbc = marker_llbc+2;
                    hasNew_llbc = 1;
                    keepCheck   = false;
                else
                    % Don't do any thing, go ahead matching
                end
            elseif ( ampRatio <= inv(lengthRatio) && ampRatio~=0  )
                % (2) Get Duration of primitives inside compositions
                p1time = llbehStruc(index,T2E)-llbehStruc(index,T1S);   % Get duration of first primitive
                p2time = llbehStruc(match,T2E)-llbehStruc(match,T1S);   % Get duration of second primitive
                
                ratio = p1time/p2time;
                
                % Merge according to the ratio
                if(ratio~=0 && ratio~=inf && ratio < inv(lengthRatio))
                    MATCH_FLAG  = 1;            % Second primitive is longer
                    LLBEH_LBL   = llbehStruc(match,behLbl);
                    data_new    = rt_MergeLowLevelBehaviors(index,llbehStruc,llbehLbl,LLBEH_LBL,MATCH_FLAG);
                    marker_llbc = marker_llbc+2;
                    hasNew_llbc = 1;
                    keepCheck   = false;
                else
                    % Don't do any thing, go ahead matching
                end
            else
                % Don't do any thing, go ahead matching
            end    
        end
    
        
        %%  3) AMPLITUDE VALUE CONTEXT   
        
        %   3.1) If there are PUSH/PULL OR PULL/PUSH of similar amplitude and similar
        %   average value, then merge as ALIGN.
        if(keepCheck)
            % Find push/pull or pull/push
            if( (intcmp(llbehStruc(index,behLbl),llbehLbl(PUSH)) && intcmp(llbehStruc(match,behLbl),llbehLbl(PULL))) || ...
                    (intcmp(llbehStruc(index,behLbl),llbehLbl(PULL)) && intcmp(llbehStruc(match,behLbl),llbehLbl(PUSH))) )
                
                % If similar amplitude (150%)
                perc1 = rt_computePercentageThresh(llbehStruc,index,AVG_AMP_VAL,AMP_WINDOW_PS_PL);
                if(perc1)
                    
                    % If similar average value (100%)
                    perc2 = rt_computePercentageThresh(llbehStruc,index,AVG_AMP_VAL,MAG_WINDOW_PS_PL);
                    
                    if(perc2)
                        % Merge es alignment
                        MATCH_FLAG  = 1; 
                        data_new    = rt_MergeLowLevelBehaviors(index,llbehStruc,llbehLbl,ALIGN,MATCH_FLAG);
                        marker_llbc = marker_llbc+2;
                        hasNew_llbc = 1;
                        keepCheck   = false;
                    end
                end
            end
        end
        
        
        %  3.2) If there are contiguous (SH/SH),(AL/AL),(SH/AL),(AL/SH) and the
        %  second one has a smaller amplitude merge as alignment.
        if(keepCheck)
            % Find either combination
            if((intcmp(llbehStruc(index,behLbl),llbehLbl(SHIFT)) && intcmp(llbehStruc(match,behLbl),llbehLbl(SHIFT))) || ...
                    (intcmp(llbehStruc(index,behLbl),llbehLbl(ALIGN)) && intcmp(llbehStruc(match,behLbl),llbehLbl(ALIGN))) || ...
                    (intcmp(llbehStruc(index,behLbl),llbehLbl(SHIFT)) && intcmp(llbehStruc(match,behLbl),llbehLbl(ALIGN))) || ...
                    (intcmp(llbehStruc(index,behLbl),llbehLbl(ALIGN)) && intcmp(llbehStruc(match,behLbl),llbehLbl(SHIFT))) )
                
                % If the second has a smaller amplitude
                if(llbehStruc(match,AVG_AMP_VAL)<llbehStruc(index,AVG_AMP_VAL))
                    % Merge es alignment
                    MATCH_FLAG  = 1;
                    data_new    = rt_MergeLowLevelBehaviors(index,llbehStruc,llbehLbl,ALIGN,MATCH_FLAG);
                    marker_llbc = marker_llbc+2;
                    hasNew_llbc = 1;
                    keepCheck   = false;
                end
            end          
        end
        
        
        % 3.3) ALIGN/PS, ALIGN/PL or in reverse-order which have about the same
        % amplitude and average value, merge as align. Also with shift.
        if(keepCheck)
            % Find either combination
            if( ((intcmp(llbehStruc(index,behLbl),llbehLbl(ALIGN)) || intcmp(llbehStruc(index,behLbl),llbehLbl(SHIFT))) && (intcmp(llbehStruc(match,behLbl),llbehLbl(PUSH)) || intcmp(llbehStruc(match,behLbl),llbehLbl(PULL))) ) || ...
                    ((intcmp(llbehStruc(index,behLbl),llbehLbl(PUSH)) || intcmp(llbehStruc(index,behLbl),llbehLbl(PULL))) && (intcmp(llbehStruc(match,behLbl),llbehLbl(ALIGN)) || intcmp(llbehStruc(match,behLbl),llbehLbl(SHIFT))) )  )
                
                % If they have the similar amplitude (50%)
                perc1 = rt_computePercentageThresh(llbehStruc,index,AVG_AMP_VAL,AMP_WINDOW_ALIGN_PS_PL);
                if(perc1)
                    
                    % If their average value is within 100% of each other
                    perc2 = rt_computePercentageThresh(llbehStruc,index,AVG_MAG_VAL,MAG_WINDOW_ALIGN_PS_PL);
                    if(perc2)
                        % Merge es alignment
                        MATCH_FLAG  = 1;
                        data_new    = rt_MergeLowLevelBehaviors(index,llbehStruc,llbehLbl,ALIGN,MATCH_FLAG);
                        marker_llbc = marker_llbc+2;
                        hasNew_llbc = 1;
                        keepCheck   = false;
                    end
                end
            end
        end
         
        
        
        
        % 3.4) PUSH/PULL/ALGN/SHIFT-FIX or vice-versa llBehs that are contiguous 
        % that have similar amplitude and their avg value is within 50% to each other, merge as constant
        if(keepCheck)
            
            if( (intcmp(llbehStruc(index,behLbl),llbehLbl(PUSH)) && intcmp(llbehStruc(match,behLbl),llbehLbl(FIX))) || ...
                    (intcmp(llbehStruc(index,behLbl),llbehLbl(FIX)) && intcmp(llbehStruc(match,behLbl),llbehLbl(PUSH))) || ...
                    (intcmp(llbehStruc(index,behLbl),llbehLbl(PULL)) && (intcmp(llbehStruc(match,behLbl),llbehLbl(FIX)))) || ...
                    (intcmp(llbehStruc(index,behLbl),llbehLbl(FIX)) && (intcmp(llbehStruc(match,behLbl),llbehLbl(PULL)))) || ...
                    (intcmp(llbehStruc(index,behLbl),llbehLbl(ALIGN)) && intcmp(llbehStruc(match,behLbl),llbehLbl(FIX))) || ...
                    (intcmp(llbehStruc(index,behLbl),llbehLbl(FIX)) && intcmp(llbehStruc(match,behLbl),llbehLbl(ALIGN))) || ...
                    (intcmp(llbehStruc(index,behLbl),llbehLbl(SHIFT)) && intcmp(llbehStruc(match,behLbl),llbehLbl(FIX))) || ...
                    (intcmp(llbehStruc(index,behLbl),llbehLbl(FIX)) && intcmp(llbehStruc(match,behLbl),llbehLbl(SHIFT))) )
                
                % If they have the similar amplitude (50%)
                perc1 = rt_computePercentageThresh(llbehStruc,index,AVG_AMP_VAL,AMP_WINDOW_FIX);
                if(perc1)
                    
                    % If their average value is within 100% of each other
                    perc2 = rt_computePercentageThresh(llbehStruc,index,AVG_MAG_VAL,MAG_WINDOW_FIX);
                    if(perc2)
                        % Merge es alignment
                        MATCH_FLAG  = 1;
                        data_new    = rt_MergeLowLevelBehaviors(index,llbehStruc,llbehLbl,FIX,MATCH_FLAG);
                        marker_llbc = marker_llbc+2;
                        hasNew_llbc = 1;
                        keepCheck   = false;
                    end
                end
            end
        end

        
        
        if(keepCheck==true)
            % Can't find matched pairs, save the current MC
            data_new    = llbehStruc(index,:);
            hasNew_llbc = 1;
            marker_llbc = marker_llbc+1;
        end

        
    end
     
end