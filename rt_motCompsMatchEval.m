%% ********************** Documentation ***********************************
% The LLBeh structure holds low-level behaviors for each force dimension.
%
% This level combines motion composition actions to produce lower-level behaviors in states 
% in which there is contact between parts (State 2 through end). This level compares degrees 
% of change across two motion compositions per state. There are six low-level behaviors which 
% are defined as follows:
% �	Fixed
%	Def: Occurs when the part is no longer moving and remains fixed in
%	place. If single occurrence, duration must be greater than 0.1 secs or
%	otherwise it will be considered noise.
%	Conditions: No change and a total minimum of time 0.10 seconds
%	Sequence of mot. Comps: {k, kk)
% �	Contact
%	Def: Occurs when the male part hits the back wall of the female part 
%	Conditions: any action, except for an unstable one, followed by a
%               contact, or pc/nc,nc/pc followed by any action except an unstable action. 
%	Sequence of mot. Comps: { *c*)
% �	Pushing
%	Def: Occurs when a part moves along the negative direction of motion.
%	If single occurrence, duration must be greater than 0.1 secs or otherwise it will be considered noise.
%	Conditions: action is produced by one or more decrement actions and a total minimum 
%   of time 0.25 seconds. 
%	Sequence of mot. Comps: {i,ii).
% �	Pulling
%	Def: Occurs when a part moves along the positive direction of motion.
%	If single occurrence, duration must be greater than 0.1 secs or otherwise it will be considered noise.
%	Conditions: action is produced by one or more increment actions and a total minimum 
%   of time 0.25 seconds. 
%	Sequence of mot. Comps: {d,dd).
% �	Aligning
%	Def: Occurs when contiguous adjustments have smaller amplitudes.If
%	single occurrence, duration must be greater than 0.1 secs or otherwise it will be considered noise.
%	Conditions: action is produced by increasingly smaller adjustments. 
%	Sequence of mot. Comps: {aa).
% �	Shift
%	Def: Occurs when contiguous adjustments increase in value indicated
%	small alignments over time. If single occurrence, duration must be greater than 0.1 secs or otherwise it will be considered noise.
%	Conditions: action is produced by increasingly smaller adjustments and a total minimum of time 
%   0.25 seconds. 
%	Sequence of mot. Comps: {u,uu).
% �	Noise
%	If any of the previous ones cannot be recognized.
%
% Input Parameters:
% index:        - currend for loop index iterating through data
% labelType:    - label for the kind of motion composition that it is
% motComps:     - motion composition cell array data structure. mx11.
% timeCheckFlag:- a flag that indicates whether the first composition has a
%                 duration greather than the time threshold. Used for
%                 conditions that look for repeated signals: kk,ii,dd
% windowSZ:     - the size of the window to study if there is a match for
%                 the second composition. IF 0, we will examine the next
%                 element only. If one, we will examine the next two
%                 elements. 
% 
% Output Parameters:
% llbehStruc:   - the 1x17 cell array low-level beh struc 
% index:        - the index that is traversing the motion compositions 
% llbehLbl:     - label structure to be used by the hlbehComposition function
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
%**************************************************************************
function [hasNew_llb,llbehStruc,index] = rt_motCompsMatchEval(index,labelType,motComps,timeCheckFlag,lastIteration)

%%  Motion Composition Labels
    % Labels for actions in motion compositions
    adjustment      = 1;    % a
    increase        = 2;    % i
    decrease        = 3;    % d
    constant        = 4;    % k
    pos_contact     = 5;    % pc
    neg_contact     = 6;    % nc
    contact         = 7;    % c
    unstable        = 8;    % u
    noisy           = 9;    % n
    none            = 10;    % z
   %actionLbl       = {'a','i','d','k','pc','nc','c','u','n','z');  % String representation of each possibility in the actnClass set.                 
    actionLbl       = [ 1,  2,  3,  4,  5,   6,   7,  8,  9,  10];  % Updated July 2012. Represent the actionLbl's directly by integers. Facilitates conversion into C++
%% Motion Composition Structure indeces
    aLbl    = 1;  
    avgVal  = 2;
    rmsVal  = 3;
    ampVal  = 4;
    t1S     = 7;  
    t1E     = 8;
    t2S     = 9;  
    t2E     = 10; 
    tAvg    = 11;
    
%%  Labels for low-level behaviors
	FIX     = 1;        % Fixed in place
    CONTACT = 2;        % Contact
    PUSH    = 3;        % Push
    PULL    = 4;        % Pull
    ALIGN   = 5;        % Alignment
    SHIFT   = 6;        % Shift
    UNSTABLE= 7;        % Unstable
    NOISE   = 8;        % Noise
   %llbehLbl   = {'FX' 'CT' 'PS' 'PL' 'AL' 'SH' 'U' 'N');   % {'fix' 'cont' 'push' 'pull' 'align' 'shift' 'unstable' 'noise');
   llbehLbl    = [ 1,   2,   3,   4,   5,   6,   7,  8];

   
%%  Window Parameters

    % Set the range by looking at a window after the index
    if (lastIteration)
        match    = index;
    else
        match    = index + 1; 
    end

%%  MATCHES
% Match labels passed by calling function. motComps(m,[actionLbl,avgVal,rmsVal,amplitudeVal,p1Lbl,p2Lbl,t1Start,t1End,t2Start,t2End,tAvgIndex]

%% CONSTANT LABELS - Determine FIX
    if( intcmp(labelType, actionLbl(constant)) )      
            
        %% FIRST COMPOSITE IS LONG
        
        % IF pair, then merge both
        % IF no, then assign only original index
        if(timeCheckFlag)
            
            %% CONSTANT FOLLOWED BY CONSTANT
            if( intcmp(motComps(match,aLbl), labelType) )
                
                % Set the type of the low-level behavior
                llBehClass = llbehLbl(FIX);
                
                % Set motion composition labels
                mc1 = motComps(index,aLbl);
                mc2 = motComps(match,aLbl);
                
                % Assign original index
            else
                % Set the type of the low-level behavior
                llBehClass = llbehLbl(FIX);
                
                % Set the second composition label as none.
                mc1 = motComps(index,aLbl);
                mc2 = actionLbl(none);
            end
            
            %% THE FIRST ONE IS NOT LONG. IS THERE A PAIR THAT WE CAN MERGE?
        else
            %% CONSTANT FOLLOWED BY CONSTANT
            if(intcmp(motComps(match,aLbl), labelType))
                
                % Set the type of the low-level behavior
                llBehClass = llbehLbl(FIX);
                
                % Set motion composition labels
                mc1 = motComps(index,aLbl);
                mc2 = motComps(match,aLbl);

                % We could not find a pair. Render noisy.
            else
                % Set the type of the low-level behavior
                llBehClass = llbehLbl(NOISE);
                
                % Set motion composition labels
                mc1 = motComps(index,aLbl);
                mc2 = actionLbl(none);
            end
        end
%% CONTACT LABELS - Determine CONTACT
    elseif(intcmp(labelType,actionLbl(contact)))
         
        %% CONTACT FOLLOWED BY CONTACT
        if(intcmp(motComps(match,aLbl), labelType))
            
            % Set the type of the low-level behavior
            llBehClass = llbehLbl(CONTACT);
            
            % Set motion composition labels
            mc1 = motComps(index,aLbl);
            mc2 = motComps(match,aLbl);
            
            %% PC FOLLOWED BY NC
        elseif(intcmp(motComps(index,aLbl),actionLbl(pos_contact)) && intcmp(motComps(match,aLbl),actionLbl(neg_contact)))
            
            % Set the type of the low-level behavior
            llBehClass = llbehLbl(CONTACT);
            
            % Set motion composition labels
            mc1 = motComps(index,aLbl);
            mc2 = motComps(match,aLbl);
            
            %% NC FOLLOWED BY PC
        elseif(intcmp(motComps(index,aLbl),actionLbl(neg_contact)) && intcmp(motComps(match,aLbl),actionLbl(pos_contact)))
            
            % Set the type of the low-level behavior
            llBehClass = llbehLbl(CONTACT);
            
            % Set motion composition labels
            mc1 = motComps(index,aLbl);
            mc2 = motComps(match,aLbl);
            
            
            %% Single contact. Still consider as contact. (should depend on state)
        else
            % Set the type of the low-level behavior
            llBehClass = llbehLbl(CONTACT);
            
            % Set motion composition labels
            mc1 = motComps(index,aLbl);
            mc2 = actionLbl(none);
        end

%% DECREASE LABELS - Determine PUSH
    elseif(intcmp(labelType,actionLbl(increase)))             

        %% FIRST COMPOSITE IS LONG
        % IF pair, then merge both
        % IF no, then assign only original index
        if(timeCheckFlag)
            
            %% DECREASE FOLLOWED BY DECREASE
            if(intcmp(motComps(match,aLbl), labelType))
                
                % Set the type of the low-level behavior
                llBehClass = llbehLbl(PUSH);
                
                % Set motion composition labels
                mc1 = motComps(index,aLbl);
                mc2 = motComps(match,aLbl);
                
                % No match assign the original one
            else
                % Set the type of the low-level behavior
                llBehClass = llbehLbl(PUSH);
                
                % Set motion composition labels
                mc1 = motComps(index,aLbl);
                mc2 = actionLbl(none);
           
            end
            %% THE FIRST ONE IS NOT LONG. IS THERE A PAIR THAT WE CAN MERGE?
        else
            %% DECREASE FOLLOWED BY DECREASE
            if(intcmp(motComps(match,aLbl), labelType))
                
                % Set the type of the low-level behavior
                llBehClass = llbehLbl(PUSH);
                
                % Set motion composition labels
                mc1 = motComps(index,aLbl);
                mc2 = motComps(match,aLbl);
                

                % We could not find a pair. Render noisy.
            else
                % Set the type of the low-level behavior
                llBehClass = llbehLbl(NOISE);
                
                % Set motion composition labels
                mc1 = motComps(index,aLbl);
                mc2 = actionLbl(none);
    
            end
        end
        
%% INCREASE LABELS - Determine PULL
    elseif(intcmp(labelType,actionLbl(decrease)))      

        %% FIRST COMPOSITE IS LONG
        % IF pair, then merge both
        % IF no, then assign only original index
        if(timeCheckFlag)
            
            %% DECREASE FOLLOWED BY DECREASE
            if(intcmp(motComps(match,aLbl), labelType))
                
                % Set the type of the low-level behavior
                llBehClass = llbehLbl(PULL);
                
                % Set motion composition labels
                mc1 = motComps(index,aLbl);
                mc2 = motComps(match,aLbl);
             
                % No match assign the original one
            else
                % Set the type of the low-level behavior
                llBehClass = llbehLbl(PULL);
                
                % Set motion composition labels
                mc1 = motComps(index,aLbl);
                mc2 = actionLbl(none);
                
            end
            %% THE FIRST ONE IS NOT LONG. IS THERE A PAIR THAT WE CAN MERGE?
        else
            %% DECREASE FOLLOWED BY DECREASE
            if(intcmp(motComps(match,aLbl), labelType))
                
                % Set the type of the low-level behavior
                llBehClass = llbehLbl(PULL);
                
                % Set motion composition labels
                mc1 = motComps(index,aLbl);
                mc2 = motComps(match,aLbl);
                
                
                % We could not find a pair. Render noisy.
            else
                % Set the type of the low-level behavior
                llBehClass = llbehLbl(NOISE);
                
                % Set motion composition labels
                mc1 = motComps(index,aLbl);
                mc2 = actionLbl(none);
                
            end
        end
            
%% ADJUSTMENT LABELS - Determine ALIGNMENT
    elseif(intcmp(labelType,actionLbl(adjustment))) 
        
        alignmentFlag = false;
            
        %% ADJUSTMENT FOLLOWED BY SMALLER ADJUSTMENT
        % Otherwise, single adjustment
        if(intcmp(motComps(match,aLbl), labelType))
            
            % If the second amplitude value is smaller proceed
            % Otherwise we have an shift signal
            if(motComps(match,ampVal) < motComps(index,ampVal))
                
                % Set the type of the low-level behavior
                llBehClass = llbehLbl(ALIGN);
                
                % Set motion composition labels
                mc1 = motComps(index,aLbl);
                mc2 = motComps(match,aLbl);
                
                % Set alignment flag to true
                alignmentFlag = true;

                %% Adjustments are growing shifting
            else
                % Set the type of the low-level behavior
                llBehClass = llbehLbl(SHIFT);
                
                % Set motion composition labels
                mc1 = motComps(index,aLbl);
                mc2 = motComps(match,aLbl);
                
                % Set alignment flag to true
                alignmentFlag = true;

            end     % End amplitude measurement
        end         % End Adjustment Label
        
        %% ISOLATED ALIGNMNENT.
        if(alignmentFlag==0)
            % If long alignment, still consider as ALIGN. Otherwise noisy.
            if(timeCheckFlag)
                llBehClass = llbehLbl(ALIGN);
                
                % Set motion composition labels
                mc1 = motComps(index,aLbl);
                mc2 = actionLbl(none);
            else
                llBehClass = llbehLbl(NOISE);
                
                % Set motion composition labels
                mc1 = motComps(index,aLbl);
                mc2 = actionLbl(none);
            end
        end
        
%% UNSTABLE LABELS - Determine UNSTABLE
    elseif(intcmp(labelType,actionLbl(unstable)))         
          
        %% UNSTABLE FOLLOWED BY UNSTABLE
        if(intcmp(motComps(match,aLbl), labelType))
            
            % Set the type of the low-level behavior
            llBehClass = llbehLbl(UNSTABLE);
            
            % Set motion composition labels
            mc1 = motComps(index,aLbl);
            mc2 = motComps(match,aLbl);
          
            % Single Unstable element
        else
            % Set the type of the low-level behavior
            llBehClass = llbehLbl(UNSTABLE);
            
            % Set motion composition labels
            mc1 = motComps(index,aLbl);
            mc2 = actionLbl(none);
 
        end
    
%% NOISY LABELS - Determine NOISE
    else                 
        % Set the type of the low-level behavior
        llBehClass = llbehLbl(NOISE);

        % Set motion composition labels
        mc1 = motComps(index,aLbl);
        mc2 = actionLbl(noisy);                    
    end
    
%% Compute values, time indeces, and return the llbehStruc structure    
    
    % For compositions with a match 
    if(~intcmp(mc2,actionLbl(none)) && ~intcmp(mc2,actionLbl(noisy)) )
        % Average magnitude values
        avgMagVal1  = motComps(index,avgVal);
        p1time      = motComps(index,t2E)-motComps(index,t1S);
        avgMagVal2  = motComps(match,avgVal);
        p2time      = motComps(match,t2E)-motComps(match,t1S);
        
        whole_time  = motComps(match,t2E)-motComps(index,t1S);
        
        AVG_MAG_VAL = ( avgMagVal1*p1time + avgMagVal2*p2time )/whole_time;  

        % MaxVal. 2013Sept this now replaces RMS. Variable names and index stay the same for compatibility 
        rmsVal1     = motComps(index,rmsVal);
        rmsVal2     = motComps(match,rmsVal);
        AVG_RMS_VAL = max(rmsVal1,rmsVal2);        
        
%       % Root mean square
%       rmsVal1 = motComps(index,rmsVal);
%       rmsVal2 = motComps(match,rmsVal);
%       AVG_RMS_VAL = sqrt(rmsVal1^2 + rmsVal2^2)/2;

        % Amplitude value: Take the max value. These points are adjacent. The AMPLITUDE would not decrease, could only increase.T
        amplitudeVal1   = motComps(index,ampVal);
        amplitudeVal2   = motComps(match,ampVal);
        minVal1         = motComps(index,rmsVal)-amplitudeVal1;
        minVal2         = motComps(match,rmsVal)-amplitudeVal2;
        MIN_Val         = min(minVal1,minVal2);
        
        AVG_AMP_VAL     = AVG_RMS_VAL-MIN_Val;                      

        % Compute time indeces
        t1Start = motComps(index,t1S);              % Starting time of first  part for composition 1
        t1End   = motComps(index,t2E);              % Ending   time of second part for composition 1
        
        t2Start = motComps(match,t1S);          % Starting time of first  part for composition 2
        t2End   = motComps(match,t2E);          % Ending   time of second part for composition 2

        tAvgIndex   = (motComps(index,t1S)+motComps(match,t2E))/2;

        % Enter the following data into the llbehStruc structure:
        llbehStruc=[llBehClass,...         % type of low-level behavior: "fixed","contact","push","pull","aligned","unstable","noisy"
                    avgMagVal1,...         % Magnitude of data (average value). Needs to be averaged when second match is found
                    avgMagVal2,...         % avgMagVal 2
                    AVG_MAG_VAL,...        % their average 
                    rmsVal1,...            % Root mean square value 1
                    rmsVal2,...            % rms value 2
                    AVG_RMS_VAL,...        % their average
                    amplitudeVal1,...      % Largest difference from one edge to the other              
                    amplitudeVal2,...      % the second
                    AVG_AMP_VAL,...        % their average
                    mc1,...                % Adjustment, increase, decrease, constant, contact, pos contact, neg contact, unstable
                    mc2,...                % [a,i,d,k,c,pc,nc,u]
                    t1Start,...            % time at which first composition starts
                    t1End,...              % time at which first composition ends
                    t2Start,...            % time at which second composition starts
                    t2End,...              % time at which second composition ends
                    tAvgIndex              % Avg time of both compositions
                    ];                     % [llBehClass,avgMagVal,rmsVal,mC1,mC2,t1Start,t1End,t2Start,t2End,tAvgIndex]
        hasNew_llb  = 1;
        % Update index
        index       = match+1;  
    
%%  Behaviors with only one composition. The second composition (both parts are not to be considered) 
    else 
        % Average magnitude value 
        avgMagVal = motComps(index,avgVal);

        % MaxVal. 2013Sept now replaces RMS
        rmsVal = motComps(index,rmsVal);
        
%       % Root mean square
%       rmsVal = avgMagVal;

        % Amplitude value 
        amplitudeVal = motComps(index,ampVal);

        % Compute time indeces
        t1Start = motComps(index,t1S);          % Starting time for first  part for composition 1        
        t1End   = motComps(index,t1E);          % Ending   time for second part for composition 1.
        
        t2Start = motComps(index,t2S);          % Starting time of first part for composition 2
        t2End   = motComps(index,t2E);          % Ending   time of first part for composition 2
%         else
%             t2Start = motComps(index,t1S);      % Starting time for composition 1
%             t2End   = motComps(index,t1E);      % Ending   time of second part for composition 1.            
%         end
        tAvgIndex   = (t1Start+t2End)/2;

        % Enter the following data into the llbehStruc structure:
        llbehStruc=[llBehClass,...         % type of low-level behavior: "fixed","contact","push","pull","aligned","unstable","noisy"
                    avgMagVal,...          % Magnitude of data (average value). Needs to be averaged when second match is found
                    avgMagVal,...          % Repeat
                    avgMagVal,...          % Repeat
                    rmsVal,...             % Root mean square value 1
                    rmsVal,...             % Repeat
                    rmsVal,...             % Repeat
                    amplitudeVal,...       % Largest difference from one edge to the other              
                    amplitudeVal,...       % Repeat
                    amplitudeVal,...       % Repeat
                    mc1,...                % Adjustment, increase, decrease, constant, contact, pos contact, neg contact, unstable
                    mc2,...                % [a,i,d,k,c,pc,nc,u]
                    t1Start,...            % time at which first composition starts
                    t1End,...              % time at which first composition ends
                    t2Start,...            % time at which second composition starts
                    t2End,...              % time at which second composition ends
                    tAvgIndex              % Avg time of both compositions
                    ];                     % [llBehClass,avgMagVal,rmsVal,mC1,mC2,t1Start,t1End,t2Start,t2End,tAvgIndex]
        hasNew_llb  = 1;
        index       = index+1;
    end
end