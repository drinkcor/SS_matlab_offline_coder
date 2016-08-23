%%************************ Documentation **********************************
% This function analyzes how it can match pairs of primitives. As an input, 
% this function receives a first primitive input. The function, then looks
% at the label of the next primitive (in some ocassions one can examine 
% labels even further than the next neighbor). Depending on the type of label
% sequence for primitives, a motion composition label will be given according to the table below.
%
% The operation will only go forward if the amplitude ratio between the
% primitives is not more than 2x or the duration more than 5x. 
% (Positive or negative contacts are an
% exception as they will always be large). 
%
%   If primitive 1 is:
%   Positive
%       And primitive 2 is as below, then assign...
%       Neg:    adjustment, 'a'
%       Pos:    increase,   'i'
%       Const:  increase,   'i'
%       Pimp:   pos contact,'pc'
%       Nimp:   neg contact,'nc'
%
%   Negative
%       Pos:    adjustment, 'a'
%       Neg:    decrease,   'd'
%       Const:  decrease,   'd'
%       Pimp:   pos contact,'pc'
%       Nimp:   neg contact,'nc'
%
%   Constant
%       Pos:    increase,   'i'
%       Neg:    decrease,   'd'
%       Const:  constant,   'k'
%       Pimp:   pos contact,'pc'
%       Nimp:   neg contact,'nc'
%
%   Pimp
%       Pos:    pos contact,'pc'
%       Neg:    pos contact,'pc'
%       Const:  pos contact,'pc'
%       pimp:   unstable,   'u'
%       Nimp:   contact,    'c'
%
%   Nimp
%       Pos:    neg contact,'nc'
%       Neg:    neg contact,'nc'
%       Const:  neg contact,'nc'
%       Pimp:   contact,    'c'
%       Nimp:   unstable,   'u'
%
% 
%--------------------------------------------------------------------------
% For Reference: Structures and Labels
%--------------------------------------------------------------------------
% Primitives = [bpos,mpos,spos,bneg,mneg,sneg,cons,pimp,nimp,none]      % Represented by integers: [1,2,3,4,5,6,7,8,9,10]  
% statData   = [dAvg dMax dMin dStart dFinish dGradient dLabel]
%--------------------------------------------------------------------------
% actionLbl  = ['a','i','d','k','pc','nc','c','u','n','z'];             % Represented by integers: [1,2,3,4,5,6,7,8,9,10]  
% motComps   = [nameLabel,avgVal,maxVal,amplitudeVal,                   % 2013Sept replaces maxVal for rmsVal [nameLabel,avgVal,rmsVal,amplitudeVal. Keep same variable names for compatibility
%               p1lbl,p2lbl,
%               t1Start,t1End,t2Start,t2End,tAvgIndex]
%--------------------------------------------------------------------------
% llbehLbl   = ['FX' 'CT' 'PS' 'PL' 'AL' 'SH' 'U' 'N'];                 % Represented by integers: [1,2,3,4,5,6,7,8]
% llbehStruc:  [actnClass,...
%              avgMagVal1,avgMagVal2,AVG_MAG_VAL,
%              maxVal1,maxVal2,MAX_RMS_VAL,                             % 2013Sept replaces maxVali for rmsVal1,rmsVal2,AVG_RMS_VAL,
%              ampVal1,ampVal2,AVG_AMP_VAL,
%              mc1,mc2,
%              T1S,T1_END,T2S,T2E,TAVG_INDEX]
%--------------------------------------------------------------------------
%
% Input Parameters:     
%
% index:                    - indicates what primitive segment we are on
% labelType:                - string describing whether 'positive','negative,'constant','impulse'
% szLabel:                  - string array. Indicates whether prim is b/m/s/pos/net/const/impulse/
%
% motComps(motCompsIndex)   - a 1x11 dimensional struc to hold composite primitives info
%                           - [actnClass,avgMagVal,rmsVal,glabel1,glabel2,t1Start,t1End,t2Start,t2End,tAvgIndex]
%                           - defined in CompoundMotionComposition.m
%                           - Usually extract values from:
%                             statData[avg,max,min,start_time,finish_time,gradient,gradientlbl]. 
%
% gradLabels                - gradient label classification structure,
%                             originally defined in fitRegressionCurves.m 
%                             Using the same struc throught all the m files
%                             helps to insure there is consistency across
%                             function calls
%**************************************************************************
function [hasNew_cm,motComps,index]=rt_primMatchEval(index,labelType,lbl,statData,lastIteration)
    
%% Initialization    
    
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
               
%%  DEFINE ACTION CLASS    
    % String Cell Array used to describe the kind of action (a=adjustment, i=increase, d=decrease, c=constant).
    actnClass       = '';
    
    % These variables are used for indexing actnClassLbl. 
    adjustment      = 1;    % a
    increase        = 2;    % i
    decrease        = 3;    % d
    constant        = 4;    % k
    pos_contact     = 5;    % pc
    neg_contact     = 6;    % nc
    contact         = 7;    % c
%   unstable        = 8;    % u
%   actionLbl       = ['a';'i';'d';'k';'p';'n';'c';'u'];  % String representation of each possibility in the actnClass set.                 
    actionLbl       = [1,2,3,4,5,6,7,8];                  % This array has been updated to be an int vector
    
%% Amplitude and Duration Parameters
    compositesAmplitudeRatio    = 2;
    lengthRatio                 = 5; 
%% Number of Compositions
    numCompositions = 2;    % Set this default parameter to indicate that we are working with 2 contiguous compositions. If this is false later, value changed to 1. 
    TS=4; TE=5;
%%  Window Parameters

    % Set the range by looking at a window after the index
    if (lastIteration)
        match    = index;
    else
        match    = index + 1; 
    end
    
    %Match           = false;                      % If no match look again. 
%%  MATCHES
    % statData(m,[Avg,Max,Min,Start,Finish,gradient,label]) 
    % Pending.... For now, we will only look for direct connections:
    % i.e. bpos and bneg. Not for bpos/mneg, bpos/sneg

%% POSITIVE LABELS
    if(strcmp(labelType,'positive'))
                
%%          POSITIVE LABEL folled by NEGATIVE LABEL = MATCH = ALIGNMENT
            if(strcmp(gradInt2gradLbl(statData(match,7)), gradLabels(BNEG,:)) || ...     %bneg
                    strcmp(gradInt2gradLbl(statData(match,7)), gradLabels(MNEG,:)) || ...%mneg
                        strcmp(gradInt2gradLbl(statData(match,7)), gradLabels(SNEG,:)))  %sneg. match is the index that looks ahead.                                                                 
                    
                % Set the type of the second label
                if(strcmp(gradInt2gradLbl(statData(match,7)), gradLabels(BNEG,:)));     lbl2=BNEG;
                elseif(strcmp(gradInt2gradLbl(statData(match,7)), gradLabels(MNEG,:))); lbl2=MNEG;
                else                                                                    lbl2=SNEG;
                end                

                %% Check Amplitude and Duration between Primitives
                
                % Get Duration of primitives inside compositions
                p1time = statData(index,TE)-statData(index,TS);   % Get duration of first primitive
                p2time = statData(match,TE)-statData(match,TS);   % Get duration of second primitive
                
                if(p1time == 0 || p1time==inf || p2time==inf)     % Throw away this primitive
                    % Set number of compositions to 0
                    numCompositions=0;
                    
                else
                    durationRatio=p2time/p1time;
                    % || durationRatio>lengthRatio || durationRatio < inv(lengthRatio)

                    % Get Amplitude of primitives inside compositions amplitudeVal: maxp1,minp2Max and min values of first and second primitives
                    p1max = statData(index,2); p1min = statData(index,3);
                    p2max = statData(match,2); p2min = statData(match,3); 
                    p1 = [p1max p1min]; p2 = [p2max p2min];                
                    [amplitudeVal,amp1,amp2] = computedAmplitude('pos','neg',p1,p2);

                    % Compute ratio of 2nd primitive vs 1st primitive. If ratio is bigger than "compositesAmplitudeRatio" don't combine. Otherwise do.
                    ampRatio = amp2/amp1;
                
                    % 1 Primitive Composition
                    if(ampRatio==0 || ampRatio==inf || ampRatio > compositesAmplitudeRatio || ampRatio < inv(compositesAmplitudeRatio) || durationRatio>lengthRatio || durationRatio < inv(lengthRatio)) 
                    
                        % Class: adjustment
                        actnClass = actionLbl(increase);
                    
                        % Gradient labels
                        glabel1 = gradLbl2gradInt(gradLabels(lbl,:)); 
                        glabel2 = gradLbl2gradInt(gradLabels(lbl,:)); 
                        
                        % Check amplitude between compositions
                        amp1 = statData(index,2); amp2 = statData(index,3);                                       

                        % Amplitude: either both pos/neg or one pos the other neg.
                        if(amp1>=0 && amp2>=0 || amp1<=0 && amp2<=0)
                            amplitudeVal    = abs(amp1)-abs(amp2);  % Subtract both positive or negative values to get the amplitude
                        else
                            amplitudeVal    = abs(amp1)+abs(amp2);  % Take the absolute value of both and add them
                        end
                        
                        % Set number of compositions to 1
                        numCompositions=1;
                    
                
                    % 2 Primitives Composition: the amplitude difference is small, and it's okay to combine
                    else
                    
                        % Class: adjustment
                        actnClass = actionLbl(adjustment);
                    
                        % Gradient labels
                        glabel1 = gradLbl2gradInt(gradLabels(lbl,:));
                        glabel2 = gradLbl2gradInt(gradLabels(lbl2,:));    
                    end
                end
%%          POSITIVE LABEL follwed by POSITIVE LABEL = REPEAT = INCREASE
            %  Need a flag to see if we get constant repeat or a single
            %  case for the length of the window
            elseif(strcmp(gradInt2gradLbl(statData(match,7)), gradLabels(BPOS,:)) || ...     % bpos
                        strcmp(gradInt2gradLbl(statData(match,7)), gradLabels(MPOS,:)) || ...% mpos
                            strcmp(gradInt2gradLbl(statData(match,7)), gradLabels(SPOS,:)))  % spos. match is the index that looks ahead. 

                % Set the type of the second label
                if(strcmp(gradInt2gradLbl(statData(match,7)), gradLabels(BPOS,:)));     lbl2=BPOS;
                elseif(strcmp(gradInt2gradLbl(statData(match,7)), gradLabels(MPOS,:))); lbl2=MPOS;
                else                                                                    lbl2=SPOS;
                end                                                

                %% Check Amplitude and Duration between primitives

                % Get Duration of primitives inside compositions
                p1time = statData(index,TE)-statData(index,TS);   % Get duration of first primitive
                p2time = statData(match,TE)-statData(match,TS);   % Get duration of second primitive
                if(p1time == 0 || p1time==inf || p2time==inf)     % Throw away this primitive         
                    % Set number of compositions to 0
                    numCompositions=0;
                else
                    durationRatio=p2time/p1time;
                    % || durationRatio>lengthRatio || durationRatio < inv(lengthRatio)                

                    % Get Amplitude of primitives inside compositions amplitudeVal: maxp1,minp2Max and min values of first and second primitives
                    p1max = statData(index,2); p1min = statData(index,3);
                    p2max = statData(match,2); p2min = statData(match,3); 
                    p1 = [p1max p1min]; p2 = [p2max p2min];                
                    [amplitudeVal,amp1,amp2] = computedAmplitude('pos','pos',p1,p2);

                    % Compute ratio of 2nd primitive vs 1st primitive. If ratio is bigger than "compositesAmplitudeRatio" don't combine. Otherwise do.
                    ampRatio = amp2/amp1;
                
                    % 1 Primitive Composition
                    if(ampRatio==0 || ampRatio==inf || ampRatio > compositesAmplitudeRatio || ampRatio < inv(compositesAmplitudeRatio) || durationRatio>lengthRatio || durationRatio < inv(lengthRatio)) 
                    
                    
                        % actnClass: increase
                        actnClass = actionLbl(increase);     % Increase
                
                        % Gradient labels
                        glabel1 = gradLbl2gradInt(gradLabels(lbl,:)); 
                        glabel2 = gradLbl2gradInt(gradLabels(lbl,:)); 
                
                        % Check amplitude between compositions
                        amp1 = statData(index,2); amp2 = statData(index,3);                                       

                        % Amplitude: either both pos/neg or one pos the other neg.
                        if(amp1>=0 && amp2>=0 || amp1<=0 && amp2<=0)
                            amplitudeVal    = abs(amp1)-abs(amp2);  % Subtract both positive or negative values to get the amplitude
                        else
                            amplitudeVal    = abs(amp1)+abs(amp2);  % Take the absolute value of both and add them
                        end
                        
                        % Set number of compositions to 1
                        numCompositions=1;
                    
                    % 2 Primitive Composition: the amplitude difference is small, and it's okay to combine
                    else
                    
                        % actnClass: increase
                        actnClass = actionLbl(increase);     % Increase
                
                        % Gradient labels
                        glabel1 = gradLbl2gradInt(gradLabels(lbl,:));
                        glabel2 = gradLbl2gradInt(gradLabels(lbl2,:));    
                    end
                end


%%          POSITIVE LABEL followed by CONSTANT LABEL = INCREASE
            %  Need a flag to see if we get constant repeat or a single
            %  case for the length of the window
            elseif( strcmp(gradInt2gradLbl(statData(match,7)), gradLabels(CONST,:)) )  % match is the index that looks ahead.                

                % Check Amplitude and Duration between compositions
                
                % Get Duration of primitives inside compositions
                p1time = statData(index,TE)-statData(index,TS);   % Get duration of first primitive
                p2time = statData(match,TE)-statData(match,TS);   % Get duration of second primitive
                if(p1time == 0 || p1time==inf || p2time==inf)     % Throw away this primitive 
                    % Set number of compositions to 0
                    numCompositions=0;
                else
                    durationRatio=p2time/p1time;
                    % || durationRatio>lengthRatio || durationRatio < inv(lengthRatio)                  

                    % Get Amplitude of primitives inside compositions amplitudeVal: maxp1,minp2Max and min values of first and second primitives
                    p1max = statData(index,2); p1min = statData(index,3);
                    p2max = statData(match,2); p2min = statData(match,3); 
                    p1 = [p1max p1min]; p2 = [p2max p2min];                
                    [amplitudeVal,amp1,amp2] = computedAmplitude('pos','const',p1,p2);

                    % Compute ratio of 2nd primitive vs 1st primitive. If ratio is bigger than "compositesAmplitudeRatio" don't combine. Otherwise do.
                    ampRatio = amp2/amp1;
                
                    % 1 Primitive Composition
                    if(ampRatio==0 || ampRatio==inf || ampRatio > compositesAmplitudeRatio || ampRatio < inv(compositesAmplitudeRatio) || durationRatio>lengthRatio || durationRatio < inv(lengthRatio)) 
                    

                        % Increase
                        actnClass = actionLbl(increase);                     
                                            
                        % Gradient labels
                        glabel1 = gradLbl2gradInt(gradLabels(lbl,:)); 
                        glabel2 = gradLbl2gradInt(gradLabels(lbl,:)); 
                        
                        % Check amplitude between compositions
                        amp1 = statData(index,2); amp2 = statData(index,3);                                       

                        % Amplitude: either both pos/neg or one pos the other neg.
                        if(amp1>=0 && amp2>=0 || amp1<=0 && amp2<=0)
                            amplitudeVal    = abs(amp1)-abs(amp2);  % Subtract both positive or negative values to get the amplitude
                        else
                            amplitudeVal    = abs(amp1)+abs(amp2);  % Take the absolute value of both and add them
                        end
                
                        % Set number of compositions to 1
                        numCompositions=1;
                
                    % 2 Primitive Composition: the amplitude difference is small, and it's okay to combine
                    else
                    
                        % Increase
                        actnClass = actionLbl(increase);  
                    
                        % Gradient labels
                        glabel1 = gradLbl2gradInt(gradLabels(lbl,:));              % Positive
                        glabel2 = gradLbl2gradInt(gradLabels(CONST,:));            % Constant
                    end
                end
                  
%%          POSITIVE LABEL followed by PIMP = POS_CONTACT
            %  Need a flag to see if we get constant repeat or a single
            %  case for the length of the window
            elseif( strcmp(gradInt2gradLbl(statData(match,7)), gradLabels(PIMP,:)) )     % match is the index that looks ahead.         

                %% Get Amplitude of primitives inside compositions amplitudeVal: maxp1,minp2Max and min values of first and second primitives
                p1max = statData(index,2); p1min = statData(index,3);
                p2max = statData(match,2); p2min = statData(match,3); 
                p1 = [p1max p1min]; p2 = [p2max p2min];                
                [amplitudeVal,~,~] = computedAmplitude('pos','impulse',p1,p2);  
                % Contact
                actnClass = actionLbl(pos_contact);          
                
                % Gradient labels
                glabel1 = gradLbl2gradInt(gradLabels(lbl,:));             % Positive
                glabel2 = gradLbl2gradInt(gradLabels(PIMP,:));            % Pimp
                

                
%%          POSITIVE LABEL followed by NIMP = NEG_CONTACT
            %  Need a flag to see if we get constant repeat or a single
            %  case for the length of the window
            elseif( strcmp(gradInt2gradLbl(statData(match,7)), gradLabels(NIMP,:)) )  % match is the index that looks ahead. 
                       
                %% Get Amplitude of primitives inside compositions amplitudeVal: maxp1,minp2Max and min values of first and second primitives
                p1max = statData(index,2); p1min = statData(index,3);
                p2max = statData(match,2); p2min = statData(match,3); 
                p1 = [p1max p1min]; p2 = [p2max p2min];                
                [amplitudeVal,~,~] = computedAmplitude('pos','impulse',p1,p2); 
                
                % Contact
                actnClass = actionLbl(neg_contact);
                
                % Gradient labels
                glabel1 = gradLbl2gradInt(gradLabels(lbl,:));             % Positive
                glabel2 = gradLbl2gradInt(gradLabels(NIMP,:));            % Nimp
                

                
%%          Pure Increase
            else
                actnClass       = actionLbl(increase);                  % increase
                glabel1         = gradLbl2gradInt(gradLabels(lbl,:));   % positive
                glabel2         = gradLbl2gradInt(gradLabels(lbl,:));   % positive
                
                % Check amplitude between compositions
                amp1 = statData(index,2); amp2 = statData(index,3);                                       

                % Amplitude: either both pos/neg or one pos the other neg.
                if(amp1>=0 && amp2>=0 || amp1<=0 && amp2<=0)
                    amplitudeVal    = abs(amp1)-abs(amp2);  % Subtract both positive or negative values to get the amplitude
                else
                    amplitudeVal    = abs(amp1)+abs(amp2);  % Take the absolute value of both and add them
                end
                
                % Set number of compositions to 1
                numCompositions=1;
                
            end % End combinations

%% IF NEGATIVE
    elseif(strcmp(labelType,'negative'))        
         
%%          NEGATIVE LABEL followed by POSITIVE LABELS = MATCH = ALIGNMENT
            if(strcmp(gradInt2gradLbl(statData(match,7)), gradLabels(BPOS,:)) || ...     %bpos
                    strcmp(gradInt2gradLbl(statData(match,7)), gradLabels(MPOS,:)) || ...%mpos
                        strcmp(gradInt2gradLbl(statData(match,7)), gradLabels(SPOS,:)) )  %spos.match is the index that looks ahead.                                                                 
                
                % Set the type of the second label
                if(strcmp(gradInt2gradLbl(statData(match,7)), gradLabels(BPOS,:)));     lbl2=BPOS;
                elseif(strcmp(gradInt2gradLbl(statData(match,7)), gradLabels(MPOS,:))); lbl2=MPOS;
                else                                                                    lbl2=SPOS;
                end
                                
                %% Check Amplitude and Duration between compositions
                
                %% Get Duration of primitives inside compositions
                p1time = statData(index,TE)-statData(index,TS);   % Get duration of first primitive
                p2time = statData(match,TE)-statData(match,TS);   % Get duration of second primitive
                if(p1time == 0 || p1time==inf || p2time==inf)     % Throw away this primitive 
                    % Set number of compositions to 0
                    numCompositions=0;
                else
                    durationRatio=p2time/p1time;
                    % || durationRatio>lengthRatio || durationRatio < inv(lengthRatio)

                    %% Get Amplitude of primitives inside compositions amplitudeVal: maxp1,minp2Max and min values of first and second primitives
                    p1max = statData(index,2); p1min = statData(index,3);
                    p2max = statData(match,2); p2min = statData(match,3); 
                    p1 = [p1max p1min]; p2 = [p2max p2min];                
                    [amplitudeVal,amp1,amp2] = computedAmplitude('neg','pos',p1,p2);

                    % Compute ratio of 2nd primitive vs 1st primitive. If ratio is bigger than "compositesAmplitudeRatio" don't combine. Otherwise do.
                    ampRatio = amp2/amp1;
                
                    %% 1 Primitive Composite
                    if(ampRatio==0 || ampRatio==inf || ampRatio > compositesAmplitudeRatio || ampRatio < inv(compositesAmplitudeRatio) || durationRatio>lengthRatio || durationRatio < inv(lengthRatio) ) 
                    

                        % Class
                        actnClass = actionLbl(decrease);                    % Decrease

                        % Gradient labels
                        glabel1 = gradLbl2gradInt(gradLabels(lbl,:)); 
                        glabel2 = gradLbl2gradInt(gradLabels(lbl,:)); 
                        
                        % Check amplitude between compositions
                        amp1 = statData(index,2); amp2 = statData(index,3);                                       

                        % Amplitude: either both pos/neg or one pos the other neg.
                        if(amp1>=0 && amp2>=0 || amp1<=0 && amp2<=0)
                            amplitudeVal    = abs(amp1)-abs(amp2);  % Subtract both positive or negative values to get the amplitude
                        else
                            amplitudeVal    = abs(amp1)+abs(amp2);  % Take the absolute value of both and add them
                        end
                               
                        % Set number of compositions to 1
                        numCompositions=1;
                    
                    %% 2 Primitive Composite: the amplitude difference is small, and it's okay to combine
                    else
                    
                        % Class
                        actnClass = actionLbl(adjustment);                 % Alignment
                
                        % Gradient labels
                        glabel1 = gradLbl2gradInt(gradLabels(lbl,:));
                        glabel2 = gradLbl2gradInt(gradLabels(lbl2,:)); 
                    end
                end
                

%%          NEGATIVE LABEL followed by NEGATIVE LABELS = REPEAT = DECREASE
            %  Need a flag to see if we get constant repeat or a single
            %  case for the length of the window
            elseif(strcmp(gradInt2gradLbl(statData(match,7)), gradLabels(BNEG,:)) || ...
                        strcmp(gradInt2gradLbl(statData(match,7)), gradLabels(MNEG,:)) || ...
                            strcmp(gradInt2gradLbl(statData(match,7)), gradLabels(SNEG,:)))  % match is the index that looks ahead. 
                        
                % Set the type of the second label
                if( strcmp(gradInt2gradLbl(statData(match,7)), gradLabels(BNEG,:)) );     lbl2=BNEG;
                elseif( strcmp(gradInt2gradLbl(statData(match,7)), gradLabels(MNEG,:)) ); lbl2=MNEG;
                else                                                 lbl2=SNEG;
                end
                
                %% Check Amplitude and Duration between compositions
                
                %% Get Duration of primitives inside compositions
                p1time = statData(index,TE)-statData(index,TS);   % Get duration of first primitive
                p2time = statData(match,TE)-statData(match,TS);   % Get duration of second primitive
                if(p1time == 0 || p1time==inf || p2time==inf)     % Throw away this primitive 
                    % Set number of compositions to 0
                    numCompositions=0;
                else
                    durationRatio=p2time/p1time;
                    % || durationRatio>lengthRatio || durationRatio < inv(lengthRatio)

                    %% Get Amplitude of primitives inside compositions amplitudeVal: maxp1,minp2Max and min values of first and second primitives
                    p1max = statData(index,2); p1min = statData(index,3);
                    p2max = statData(match,2); p2min = statData(match,3); 
                    p1 = [p1max p1min]; p2 = [p2max p2min];                
                    [amplitudeVal,amp1,amp2] = computedAmplitude('neg','neg',p1,p2);

                    %% Compute ratio of 2nd primitive vs 1st primitive. If ratio is bigger than "compositesAmplitudeRatio" don't combine. Otherwise do.
                    ampRatio = amp2/amp1;
                
                    %% 1 Primitive Composition
                    if(ampRatio==0 || ampRatio==inf || ampRatio > compositesAmplitudeRatio || ampRatio < inv(compositesAmplitudeRatio) || durationRatio>lengthRatio || durationRatio < inv(lengthRatio) ) 
                    
                        % Class
                        actnClass = actionLbl(decrease);    % Decrease                    
                    
                        % Gradient labels
                        glabel1 = gradLbl2gradInt(gradLabels(lbl,:)); 
                        glabel2 = gradLbl2gradInt(gradLabels(lbl,:)); 
                        
                        % Check amplitude between compositions
                        amp1 = statData(index,2); amp2 = statData(index,3);                                       

                        % Amplitude: either both pos/neg or one pos the other neg.
                        if(amp1>=0 && amp2>=0 || amp1<=0 && amp2<=0)
                            amplitudeVal    = abs(amp1)-abs(amp2);  % Subtract both positive or negative values to get the amplitude
                        else
                            amplitudeVal    = abs(amp1)+abs(amp2);  % Take the absolute value of both and add them
                        end
                        
                        % Set number of compositions to 1
                        numCompositions=1;
                    
                
                    %% 2 Primitives Composition: the amplitude difference is small, and it's okay to combine
                    else
                    
                        % Class
                        actnClass = actionLbl(decrease);    % Decrease 
                    
                        % Gradient labels
                        glabel1 = gradLbl2gradInt(gradLabels(lbl,:));     % Negative
                        glabel2 = gradLbl2gradInt(gradLabels(lbl2,:));    % Negative
                    end
                end                
                
%%          NEGATIVE LABEL followed by CONSTANT LABEL = DECREASE
            %  Need a flag to see if we get constant repeat or a single
            %  case for the length of the window
            elseif(strcmp(gradInt2gradLbl(statData(match,7)), gradLabels(CONST,:)))        % match is the index that looks ahead.                             

                %% Check Amplitude and Duration between compositions
                
                %% Get Duration of primitives inside compositions
                p1time = statData(index,TE)-statData(index,TS);   % Get duration of first primitive
                p2time = statData(match,TE)-statData(match,TS);   % Get duration of second primitive
                if(p1time == 0 || p1time==inf || p2time==inf)     % Throw away this primitive 
                    % Set number of compositions to 0
                    numCompositions=0;
                else
                    durationRatio=p2time/p1time;
                    % || durationRatio>lengthRatio || durationRatio < inv(lengthRatio)

                    %% Get Amplitude of primitives inside compositions amplitudeVal: maxp1,minp2Max and min values of first and second primitives
                    p1max = statData(index,2); p1min = statData(index,3);
                    p2max = statData(match,2); p2min = statData(match,3); 
                    p1 = [p1max p1min]; p2 = [p2max p2min];                
                    [amplitudeVal,amp1,amp2] = computedAmplitude('neg','const',p1,p2);

                    % Compute ratio of 2nd primitive vs 1st primitive. If ratio is bigger than "compositesAmplitudeRatio" don't combine. Otherwise do.
                    ampRatio = amp2/amp1;
                
                    %% 1 Primitive Composition
                    if(ampRatio==0 || ampRatio==inf || ampRatio > compositesAmplitudeRatio || ampRatio < inv(compositesAmplitudeRatio) || durationRatio>lengthRatio || durationRatio < inv(lengthRatio) ) 
                    
                        % Class
                        actnClass = actionLbl(decrease);                    % Decrease
                

                    
                        % Gradient labels
                        glabel1 = gradLbl2gradInt(gradLabels(lbl,:)); 
                        glabel2 = gradLbl2gradInt(gradLabels(lbl,:)); 
                        
                        % Check amplitude between compositions
                        amp1 = statData(index,2); amp2 = statData(index,3);                                       

                        % Amplitude: either both pos/neg or one pos the other neg.
                        if(amp1>=0 && amp2>=0 || amp1<=0 && amp2<=0)
                            amplitudeVal    = abs(amp1)-abs(amp2);  % Subtract both positive or negative values to get the amplitude
                        else
                            amplitudeVal    = abs(amp1)+abs(amp2);  % Take the absolute value of both and add them
                        end
 
                        % Set number of compositions to 1
                        numCompositions=1;
                    %% 2 Primitives Composition: The amplitude difference is small, and it's okay to combine
                    else
                    
                        % Class
                        actnClass = actionLbl(decrease); 
                
                        % Gradient labels
                        glabel1 = gradLbl2gradInt(gradLabels(lbl,:));              % Negative
                        glabel2 = gradLbl2gradInt(gradLabels(CONST,:));            % Constant
                    end
                end
 
%%          NEGATIVE LABEL followed by PIMP = POS_CONTACT
            %  Need a flag to see if we get constant repeat or a single
            %  case for the length of the window
            elseif(strcmp(gradInt2gradLbl(statData(match,7)), gradLabels(PIMP,:)))  % match is the index that looks ahead.           

                
                %% Get Amplitude of primitives inside compositions amplitudeVal: maxp1,minp2Max and min values of first and second primitives
                p1max = statData(index,2); p1min = statData(index,3);
                p2max = statData(match,2); p2min = statData(match,3); 
                p1 = [p1max p1min]; p2 = [p2max p2min];                
                [amplitudeVal,~,~] = computedAmplitude('neg','impulse',p1,p2);                
                
                % Class: contact
                actnClass = actionLbl(pos_contact);                % pos_contact   
                
                % Gradient labels
                glabel1 = gradLbl2gradInt(gradLabels(lbl,:));             % Negative
                glabel2 = gradLbl2gradInt(gradLabels(PIMP,:));            % Pimp
                
                
                
%%          NEGATIVE LABEL followed by NIMP = NEG_CONTACT
            %  Need a flag to see if we get constant repeat or a single
            %  case for the length of the window
            elseif(strcmp(gradInt2gradLbl(statData(match,7)), gradLabels(NIMP,:)))  % match is the index that looks ahead. 

                %% Get Amplitude of primitives inside compositions amplitudeVal: maxp1,minp2Max and min values of first and second primitives
                p1max = statData(index,2); p1min = statData(index,3);
                p2max = statData(match,2); p2min = statData(match,3); 
                p1 = [p1max p1min]; p2 = [p2max p2min];                
                [amplitudeVal,~,~] = computedAmplitude('neg','impulse',p1,p2); 
                % Class
                actnClass = actionLbl(neg_contact);                % neg_contact 
                
                % Gradient labels
                glabel1 = gradLbl2gradInt(gradLabels(lbl,:));             % Positive
                glabel2 = gradLbl2gradInt(gradLabels(NIMP,:));            % Nimp
                
                             
                
%%          NONE
            else
                actnClass   = actionLbl(decrease);                                      % pure decrease
                %amplitudeVal    = statData(index,2)-statData(index,3);                 % max-min
                glabel1 = gradLbl2gradInt(gradLabels(lbl,:));                           % NEG
                glabel2 = gradLbl2gradInt(gradLabels(lbl,:));                           % NEG
                
                % Check amplitude between compositions
                amp1 = statData(index,2); amp2 = statData(index,3);                                                                
                
                % Amplitude: either both pos/neg or one pos the other neg.
                if(amp1>=0 && amp2>=0 || amp1<=0 && amp2<=0)
                    amplitudeVal    = abs(amp1)-abs(amp2);  % Subtract both positive or negative values to get the amplitude
                else
                    amplitudeVal    = abs(amp1)+abs(amp2);  % Take the absolute value of both and add them
                end   
                
                % Set number of compositions to 1
                numCompositions=1;
                
            end % End combinations

            
%% IF CONSTANT: only looks at the next index
    elseif(strcmp(labelType,'constant'))
               
%%          CONSTANT WITH INCREASE
            if(strcmp(gradInt2gradLbl(statData(match,7)), gradLabels(BPOS,:)) || ...
                    strcmp(gradInt2gradLbl(statData(match,7)), gradLabels(MPOS,:)) || ...
                        strcmp(gradInt2gradLbl(statData(match,7)), gradLabels(SPOS,:)))            % CONSTANT + POSITIVE
                                   
                %% Check Amplitude and Duration between compositions
                
                %% Get Duration of primitives inside compositions
                p1time = statData(index,TE)-statData(index,TS);   % Get duration of first primitive
                p2time = statData(match,TE)-statData(match,TS);   % Get duration of second primitive
                if(p1time == 0 || p1time==inf || p2time==inf)     % Throw away this primitive 
                    % Set number of compositions to 0
                    numCompositions=0;
                else
                    durationRatio=p2time/p1time;
                    % || durationRatio>lengthRatio || durationRatio < inv(lengthRatio)

                    %% Get Amplitude of primitives inside compositions amplitudeVal: maxp1,minp2Max and min values of first and second primitives
                    p1max = statData(index,2); p1min = statData(index,3);
                    p2max = statData(match,2); p2min = statData(match,3); 
                    p1 = [p1max p1min]; p2 = [p2max p2min];                
                    [amplitudeVal,amp1,amp2] = computedAmplitude('const','pos',p1,p2);

                    % Compute ratio of 2nd primitive vs 1st primitive. If ratio is bigger than "compositesAmplitudeRatio" don't combine. Otherwise do.
                    ampRatio = amp2/amp1;
                
                    %% 1 Primitive Composition
                    if(ampRatio==0 || ampRatio==inf || ampRatio > compositesAmplitudeRatio || ampRatio < inv(compositesAmplitudeRatio) || durationRatio>lengthRatio || durationRatio < inv(lengthRatio) ) 

                        % Class
                        actnClass = actionLbl(constant);                               % Constant                    

                        % Gradient labels
                        glabel1 = gradLbl2gradInt(gradLabels(CONST,:)); 
                        glabel2 = gradLbl2gradInt(gradLabels(CONST,:)); 
                        
                        % Check amplitude between compositions
                        amp1 = statData(index,2); amp2 = statData(index,3);                                       

                        % Amplitude: either both pos/neg or one pos the other neg.
                        if(amp1>=0 && amp2>=0 || amp1<=0 && amp2<=0)
                            amplitudeVal    = abs(amp1)-abs(amp2);  % Subtract both positive or negative values to get the amplitude
                        else
                            amplitudeVal    = abs(amp1)+abs(amp2);  % Take the absolute value of both and add them
                        end
                        
                        % Set number of compositions to 1
                        numCompositions=1;

                    %% 2 Primitives Composition: the amplitude difference is small, and it's okay to combine
                    else
                    
                        % Class
                        actnClass = actionLbl(increase);                               % Increase                    
                    
                        % Gradient labels
                        glabel1 = gradLbl2gradInt(gradLabels(CONST,:));
                        glabel2 = gradLbl2gradInt(gradLabels(MPOS,:));                           % Positive. Have not refined the exact dimension here.
                    end
                end


%%          CONSTANT WITH DECREASE
            elseif(strcmp(gradInt2gradLbl(statData(match,7)), gradLabels(BNEG,:)) || ...
                    strcmp(gradInt2gradLbl(statData(match,7)), gradLabels(MNEG,:)) || ...
                        strcmp(gradInt2gradLbl(statData(match,7)), gradLabels(SNEG,:)))        % CONSTANT + NEGATIVE

                %% Check Amplitude and Duration between compositions
                
                %% Get Duration of primitives inside compositions
                p1time = statData(index,TE)-statData(index,TS);   % Get duration of first primitive
                p2time = statData(match,TE)-statData(match,TS);   % Get duration of second primitive
                if(p1time == 0 || p1time==inf || p2time==inf)     % Throw away this primitive 
                    % Set number of compositions to 0
                    numCompositions=0;
                else
                    durationRatio=p2time/p1time;
                    % || durationRatio>lengthRatio || durationRatio < inv(lengthRatio)

                    %% Get Amplitude of primitives inside compositions amplitudeVal: maxp1,minp2Max and min values of first and second primitives
                    p1max = statData(index,2); p1min = statData(index,3);
                    p2max = statData(match,2); p2min = statData(match,3); 
                    p1 = [p1max p1min]; p2 = [p2max p2min];                
                    [amplitudeVal,amp1,amp2] = computedAmplitude('const','neg',p1,p2);

                    % Compute ratio of 2nd primitive vs 1st primitive. If ratio is bigger than "compositesAmplitudeRatio" don't combine. Otherwise do.
                    ampRatio = amp2/amp1;
                
                    %% 1 Primitive Composition
                    if(ampRatio==0 || ampRatio==inf || ampRatio > compositesAmplitudeRatio || ampRatio < inv(compositesAmplitudeRatio) || durationRatio>lengthRatio || durationRatio < inv(lengthRatio)) 

                        % Class: decrease
                        actnClass = actionLbl(constant);                             % constant                    

                        % Gradient labels
                        glabel1 = gradLbl2gradInt(gradLabels(CONST,:)); 
                        glabel2 = gradLbl2gradInt(gradLabels(CONST,:)); 
                        
                        % Check amplitude between compositions
                        amp1 = statData(index,2); amp2 = statData(index,3);                                       

                        % Amplitude: either both pos/neg or one pos the other neg.
                        if(amp1>=0 && amp2>=0 || amp1<=0 && amp2<=0)
                            amplitudeVal    = abs(amp1)-abs(amp2);  % Subtract both positive or negative values to get the amplitude
                        else
                            amplitudeVal    = abs(amp1)+abs(amp2);  % Take the absolute value of both and add them
                        end
                        
                        % Set number of compositions to 1
                        numCompositions=1;
                        
                    %% 2 Primitives Composition: the amplitude difference is small, and it's okay to combine
                    else                    
               
                        % Class: decrease
                        actnClass = actionLbl(decrease);                             % Decrease                    
                
                        % Gradient labels
                        glabel1 = gradLbl2gradInt(gradLabels(CONST,:));                    % Constant
                        glabel2 = gradLbl2gradInt(gradLabels(MNEG,:));                     % Negative. % Have not refined the exact dimension here
                    end
                end

                
%%          CONSTANT WITH CONSTANT
            elseif(strcmp(gradInt2gradLbl(statData(match,7)), gradLabels(CONST,:)))  % match is the index that looks ahead. 

                %% Check Amplitude and Duration between compositions
                
                %% Get Duration of primitives inside compositions
                p1time = statData(index,TE)-statData(index,TS);   % Get duration of first primitive
                p2time = statData(match,TE)-statData(match,TS);   % Get duration of second primitive
                if(p1time == 0 || p1time==inf || p2time==inf)     % Throw away this primitive        
                    % Set number of compositions to 0
                    numCompositions=0;
                else
                    durationRatio=p2time/p1time;
                    % || durationRatio>lengthRatio || durationRatio < inv(lengthRatio)

                    %% Get Amplitude of primitives inside compositions amplitudeVal: maxp1,minp2Max and min values of first and second primitives
                    p1max = statData(index,2); p1min = statData(index,3);
                    p2max = statData(match,2); p2min = statData(match,3); 
                    p1 = [p1max p1min]; p2 = [p2max p2min];                
                    [amplitudeVal,amp1,amp2] = computedAmplitude('const','const',p1,p2);

                    %  Compute ratio of 2nd primitive vs 1st primitive. If ratio is bigger than "compositesAmplitudeRatio" don't combine. Otherwise do.
                    ampRatio = amp2/amp1;
                
                    %% 1 Primitive Composition
                    if(ampRatio==0 || ampRatio==inf || ampRatio > compositesAmplitudeRatio || ampRatio < inv(compositesAmplitudeRatio) || durationRatio>lengthRatio || durationRatio < inv(lengthRatio) ) 
              
                        % Class
                        actnClass = actionLbl(constant);                   % CONSTANT

                        % Gradient labels
                        glabel1 = gradLbl2gradInt(gradLabels(CONST,:)); 
                        glabel2 = gradLbl2gradInt(gradLabels(CONST,:)); 
                        
                        % Check amplitude between compositions
                        amp1 = statData(index,2); amp2 = statData(index,3);                                       

                        % Amplitude: either both pos/neg or one pos the other neg.
                        if(amp1>=0 && amp2>=0 || amp1<=0 && amp2<=0)
                            amplitudeVal    = abs(amp1)-abs(amp2);  % Subtract both positive or negative values to get the amplitude
                        else
                            amplitudeVal    = abs(amp1)+abs(amp2);  % Take the absolute value of both and add them
                        end
                        
                                            
                        % Set number of compositions to 1
                        numCompositions=1;

                    %% 2 Primitives Composition The amplitude difference is small, and it's okay to combine
                    else              
                        % Class
                        actnClass = actionLbl(constant);                   % CONSTANT
                    
                        % Gradient labels
                        glabel1 = gradLbl2gradInt(gradLabels(CONST,:));                       % Constant
                        glabel2 = gradLbl2gradInt(gradLabels(CONST,:));                       % Constant. % Have not refined the exact dimension here
                    end
                end
               
                         

%%          CONSTANT LABEL followed by PIMP = POS_CONTACT
            %  Need a flag to see if we get constant repeat or a single
            %  case for the length of the window
            elseif(strcmp(gradInt2gradLbl(statData(match,7)), gradLabels(PIMP,:)))  % match is the index that looks ahead. 

                %% Get Amplitude of primitives inside compositions amplitudeVal: maxp1,minp2Max and min values of first and second primitives
                p1max = statData(index,2); p1min = statData(index,3);
                p2max = statData(match,2); p2min = statData(match,3); 
                p1 = [p1max p1min]; p2 = [p2max p2min];                
                [amplitudeVal,~,~] = computedAmplitude('const','impulse',p1,p2);                 
                
                % Contact
                actnClass = actionLbl(pos_contact);       % pos_contact                            

                % Gradient labels
                glabel1 = gradLbl2gradInt(gradLabels(lbl,:));             % Constant
                glabel2 = gradLbl2gradInt(gradLabels(PIMP,:));            % Pimp
                
                
%%          CONSTANT LABEL followed by NIMP = NEG_CONTACT
            %  Need a flag to see if we get constant repeat or a single
            %  case for the length of the window
            elseif(strcmp(gradInt2gradLbl(statData(match,7)), gradLabels(NIMP,:)))  % match is the index that looks ahead. 

                %% Get Amplitude of primitives inside compositions amplitudeVal: maxp1,minp2Max and min values of first and second primitives
                p1max = statData(index,2); p1min = statData(index,3);
                p2max = statData(match,2); p2min = statData(match,3); 
                p1 = [p1max p1min]; p2 = [p2max p2min];                
                [amplitudeVal,~,~] = computedAmplitude('const','impulse',p1,p2);                 
                
                % Contact
                actnClass = actionLbl(neg_contact);                % neg_contact               

                % Gradient labels
                glabel1 = gradLbl2gradInt(gradLabels(lbl,:));             % Constant
                glabel2 = gradLbl2gradInt(gradLabels(NIMP,:));            % Nimp               
                               
                
%%          PURE CONSTANT
            else
                actnClass       = actionLbl(constant);                           % constant
                %amplitudeVal    = 0;
                glabel1         = gradLbl2gradInt(gradLabels(lbl,:));             % constant
                glabel2         = gradLbl2gradInt(gradLabels(lbl,:));             % constant
                
                % Check amplitude between compositions
                amp1 = statData(index,2); amp2 = statData(index,3);                                                                   
                
                % Amplitude: either both pos/neg or one pos the other neg.
                if(amp1>=0 && amp2>=0 || amp1<=0 && amp2<=0)
                    amplitudeVal    = abs(amp1)-abs(amp2);  % Subtract both positive or negative values to get the amplitude
                else
                    amplitudeVal    = abs(amp1)+abs(amp2);  % Take the absolute value of both and add them
                end 
                
                % Set number of compositions to 1
                numCompositions=1;


            end % End combinations
            
            
%% IF PIMP: only looks at the next index
    elseif(strcmp(labelType,'pimp'))    
     
%%          Positive impulse with positive = POS_CONTACT
            if(strcmp(gradInt2gradLbl(statData(match,7)), gradLabels(BPOS,:)) || ...
                    strcmp(gradInt2gradLbl(statData(match,7)), gradLabels(MPOS,:)) || ...
                        strcmp(gradInt2gradLbl(statData(match,7)), gradLabels(SPOS,:)) || ...
                            strcmp(gradInt2gradLbl(statData(match,7)), gradLabels(PIMP,:)))  	% PIMP + POSITIVE
                                                                    
                % Class
                actnClass = actionLbl(pos_contact);                           % pos_contact

                % amplitudeVal: minp1,maxp2
                % Max and min values of first and second primitives
                p1max = statData(index,2); p1min = statData(index,3);
                p2max = statData(match,2); p2min = statData(match,3); 
                p1 = [p1max p1min]; p2 = [p2max p2min];                
                [amplitudeVal,~,~] = computedAmplitude('pos','pos',p1,p2);
                
                % Gradient labels
                glabel1 = gradLbl2gradInt(gradLabels(PIMP,:));                               % Impulse
                glabel2 = gradLbl2gradInt(gradLabels(MPOS,:));                               % Increase. Have not refined the exact dimension here.
                

%%          IF POSITIVE IMPULSE (PIMP) WITH NEG GRADIENT = POS_CONTACT
            elseif(strcmp(gradInt2gradLbl(statData(match,7)), gradLabels(BNEG,:)) || ...
                    strcmp(gradInt2gradLbl(statData(match,7)), gradLabels(MNEG,:)) || ...
                        strcmp(gradInt2gradLbl(statData(match,7)), gradLabels(SNEG,:)))        % IMPULSE + NEGATIVE
               
                % Class
                actnClass = actionLbl(contact);                       % contact
                
                % amplitudeVal: maxp1,minp2
                % Max and min values of first and second primitives
                p1max = statData(index,2); p1min = statData(index,3);
                p2max = statData(match,2); p2min = statData(match,3); 
                p1 = [p1max p1min]; p2 = [p2max p2min];                
                [amplitudeVal,~,~] = computedAmplitude('pos','neg',p1,p2);
                
                % Gradient labels
                glabel1 = gradLbl2gradInt(gradLabels(PIMP,:));                             % POSITIVE IMPULSE
                glabel2 = gradLbl2gradInt(gradLabels(MNEG,:));                             % Decrease. % Have not refined the exact dimension here
                
                
                
%%          POSITIVE IMPULSE (PIMP) WITH CONSTANT = POS_CONTACT
            elseif(strcmp(gradInt2gradLbl(statData(match,7)), gradLabels(CONST,:)))  % match is the index that looks ahead. 
              
                % Class
                actnClass = actionLbl(pos_contact);             % pos_contact

                % amplitudeVal: maxp1,minp2
                % Max and min values of first and second primitives
                p1max = statData(index,2); p1min = statData(index,3);
                p2max = statData(match,2); p2min = statData(match,3); 
                p1 = [p1max p1min]; p2 = [p2max p2min];                
                [amplitudeVal,~,~] = computedAmplitude('pos','const',p1,p2);
                
                % Gradient labels
                glabel1 = gradLbl2gradInt(gradLabels(PIMP,:));                 % Pimp
                glabel2 = gradLbl2gradInt(gradLabels(CONST,:));                % Constant
                
            
%%          POSITIVE IMPUSLE (PIMP) WITH PIMP = POS_CONTACT
            elseif(strcmp(gradInt2gradLbl(statData(match,7)), gradLabels(PIMP,:)))  % match is the index that looks ahead. 
                
                % Class
                actnClass = actionLbl(pos_contact);               % unstable

                % amplitudeVal: minp1,maxp2
                % Max and min values of first and second primitives
                p1max = statData(index,2); p1min = statData(index,3);
                p2max = statData(match,2); p2min = statData(match,3); 
                p1 = [p1max p1min]; p2 = [p2max p2min];                
                [amplitudeVal,~,~] = computedAmplitude('pos','pos',p1,p2);
                
                % Gradient labels
                glabel1 = gradLbl2gradInt(gradLabels(PIMP,:));     % impulse
                glabel2 = gradLbl2gradInt(gradLabels(PIMP,:));     % impulse
                
                
%%          PIMP WITH NIMP = CONTACT
            elseif(strcmp(gradInt2gradLbl(statData(match,7)), gradLabels(NIMP,:)))  % match is the index that looks ahead. 
                
                % Class
                actnClass = actionLbl(contact);                % contact

                % amplitudeVal: minp1,maxp2
                % Max and min values of first and second primitives
                p1max = statData(index,2); p1min = statData(index,3);
                p2max = statData(match,2); p2min = statData(match,3); 
                p1 = [p1max p1min]; p2 = [p2max p2min];                
                [amplitudeVal,~,~] = computedAmplitude('pos','neg',p1,p2);
                
                % Gradient labels
                glabel1 = gradLbl2gradInt(gradLabels(PIMP,:));     % impulse
                glabel2 = gradLbl2gradInt(gradLabels(NIMP,:));     % impulse
                

%%          NONE
            else
                actnClass       = actionLbl(pos_contact);                                  % pos_contact
                %amplitudeVal    = statData(index,2)-statData(index,3);  % max-min
                glabel1         = gradLbl2gradInt(gradLabels(lbl,:));                      % constant
                glabel2         = gradLbl2gradInt(gradLabels(lbl,:));                      % none

                % Check amplitude between compositions
                amp1 = statData(index,2); amp2 = statData(index,3);                                       
                
                % Amplitude: either both pos/neg or one pos the other neg.
                if(amp1>=0 && amp2>=0 || amp1<=0 && amp2<=0)
                    amplitudeVal    = abs(amp1)-abs(amp2);  % Subtract both positive or negative values to get the amplitude
                else
                    amplitudeVal    = abs(amp1)+abs(amp2);  % Take the absolute value of both and add them
                end

                % Set number of compositions to 1
                numCompositions=1;
                

            end % End combinations

        
        
%% IF NIMP: only looks at the next index
    elseif(strcmp(labelType,'nimp'))    
            
%%          NEGATIVE IMPULSE (NIMP) WITH POSITIVE = CONTACT
            if(strcmp(gradInt2gradLbl(statData(match,7)), gradLabels(BPOS,:)) || ...
                    strcmp(gradInt2gradLbl(statData(match,7)), gradLabels(MPOS,:)) || ...
                        strcmp(gradInt2gradLbl(statData(match,7)), gradLabels(SPOS,:)))  	% NIMP + pos
                                                                    
                % Class
                actnClass = actionLbl(contact);                         % contact

                % amplitudeVal: minp1,maxp2
                % Max and min values of first and second primitives
                p1max = statData(index,2); p1min = statData(index,3);
                p2max = statData(match,2); p2min = statData(match,3); 
                p1 = [p1max p1min]; p2 = [p2max p2min];                
                [amplitudeVal,~,~] = computedAmplitude('neg','pos',p1,p2);
                
                % Gradient labels
                glabel1 = gradLbl2gradInt(gradLabels(NIMP,:));                               % Neg. Impulse
                glabel2 = gradLbl2gradInt(gradLabels(MPOS,:));                               % Increase. Have not refined the exact dimension here.


%%          IF NEGATIVE IMPULSE (NIMP) WITH NEG GRADIENT = NEG_CONTACT
            elseif(strcmp(gradInt2gradLbl(statData(match,7)), gradLabels(BNEG,:)) || ...
                    strcmp(gradInt2gradLbl(statData(match,7)), gradLabels(MNEG,:)) || ...
                        strcmp(gradInt2gradLbl(statData(match,7)), gradLabels(SNEG,:)))        % NIMP + NEGATIVE
               
                % Class
                actnClass = actionLbl(neg_contact);                       % neg_contact
                
                % amplitudeVal: maxp1,minp2
                % Max and min values of first and second primitives
                p1max = statData(index,2); p1min = statData(index,3);
                p2max = statData(match,2); p2min = statData(match,3); 
                p1 = [p1max p1min]; p2 = [p2max p2min];                
                [amplitudeVal,~,~] = computedAmplitude('neg','neg',p1,p2);
                
                % Gradient labels
                glabel1 = gradLbl2gradInt(gradLabels(NIMP,:));                             % NEGATIVE IMPULSE
                glabel2 = gradLbl2gradInt(gradLabels(MNEG,:));                             % Decrease. % Have not refined the exact dimension here

                
                
%%          NEGATIVE IMPULSE (NIMP) WITH CONSTANT = NEG_CONTACT
            elseif(strcmp(gradInt2gradLbl(statData(match,7)), gradLabels(CONST,:)))  % match is the index that looks ahead. 
              
                % Class
                actnClass = actionLbl(neg_contact);             % neg_contact

                % amplitudeVal: maxp1,minp2
                % Max and min values of first and second primitives
                p1max = statData(index,2); p1min = statData(index,3);
                p2max = statData(match,2); p2min = statData(match,3); 
                p1 = [p1max p1min]; p2 = [p2max p2min];                
                [amplitudeVal,~,~] = computedAmplitude('neg','const',p1,p2);
                
                % Gradient labels
                glabel1 = gradLbl2gradInt(gradLabels(NIMP,:));                 % Pimp
                glabel2 = gradLbl2gradInt(gradLabels(CONST,:));                % Constant
                
            
%%          NEGATIVE IMPULSE (NIMP) WITH PIMP = CONTACT
            elseif(strcmp(gradInt2gradLbl(statData(match,7)), gradLabels(PIMP,:)))  % match is the index that looks ahead. 
                
                % Class
                actnClass = actionLbl(contact);                % CONTACT

                % amplitudeVal: minp1,maxp2
                % Max and min values of first and second primitives
                p1max = statData(index,2); p1min = statData(index,3);
                p2max = statData(match,2); p2min = statData(match,3); 
                p1 = [p1max p1min]; p2 = [p2max p2min];                
                [amplitudeVal,~,~] = computedAmplitude('neg','pos',p1,p2);
                
                % Gradient labels
                glabel1 = gradLbl2gradInt(gradLabels(NIMP,:));     % impulse
                glabel2 = gradLbl2gradInt(gradLabels(PIMP,:));     % impulse


%%          NEGATIVE IMPULSE (NIMP)  WITH NIMP = NEG_CONTACT
            elseif(strcmp(gradInt2gradLbl(statData(match,7)), gradLabels(NIMP,:)))  % match is the index that looks ahead. 
                
                % Class
                actnClass = actionLbl(neg_contact);                % neg_contact

                % amplitudeVal: minp1,maxp2
                % Max and min values of first and second primitives
                p1max = statData(index,2); p1min = statData(index,3);
                p2max = statData(match,2); p2min = statData(match,3); 
                p1 = [p1max p1min]; p2 = [p2max p2min];                
                [amplitudeVal,~,~] = computedAmplitude('neg','neg',p1,p2);
                
                % Gradient labels
                glabel1 = gradLbl2gradInt(gradLabels(NIMP,:));     % neg impulse
                glabel2 = gradLbl2gradInt(gradLabels(NIMP,:));     % neg impulse
                               
%%          NONE
            else
                actnClass = actionLbl(neg_contact);                   % neg_contact
                
                %amplitudeVal    = statData(index,2)-statData(index,3);  % max-min
                glabel1         = gradLbl2gradInt(gradLabels(lbl,:));                      % constant
                glabel2         = gradLbl2gradInt(gradLabels(lbl,:));                      % none

                % Check amplitude between compositions
                amp1 = statData(index,2); amp2 = statData(index,3);                                       
                
                % Amplitude: either both pos/neg or one pos the other neg.
                if(amp1>=0 && amp2>=0 || amp1<=0 && amp2<=0)
                    amplitudeVal    = abs(amp1)-abs(amp2);  % Subtract both positive or negative values to get the amplitude
                else
                    amplitudeVal    = abs(amp1)+abs(amp2);  % Take the absolute value of both and add them
                end
                
                % Set number of compositions to 1
                numCompositions=1;
                

            end % End combinations
      
    end         % IF positive/negative/constant/impulse

%% Compute values, time indeces, and return the motComps structure   

    %% Average values if 2 compositions
    if(numCompositions==2)
        % Average magnitude value 
        avgMagVal = (statData(index,1)+statData(match,1))/2;   

        % MavVal replaces RMS 2013Sept
        rmsVal = max(statData(index,2),statData(match,2));   
        % Root mean square
        %rmsVal = sqrt((statData(index,1)^2 + statData(match,1)^2)/2);

        % Compute time indeces
        t1Start = statData(index,4);              % Starting time for primitive 1
        t1End   = statData(index,5);%-0.001;      % Ending time for primitive 1
                    
        t2Start = statData(match,4);          % Starting time for primitive 2
        t2End   = statData(match,5);%-0.001;  % Ending time for primitive 2.  Previous code: statData(match+1,5)-0.001;


        tAvgIndex = (t1Start+t2End)/2;

        % Enter the following data into the motComps structure:
        motComps=[actnClass,...          % type of motion actnClass: "adjustment", "constant", or "impulse". 
                  avgMagVal,...          % Magnitude of data (average value). Needs to be averaged when second match is found
                  rmsVal,...             % Root mean square value
                  amplitudeVal,...       % Largest difference from one edge of p1 to the other edge of p2              
                  glabel1,...            % bpos...snet...impulse
                  glabel2,...            % type of label b/m/s/pos/neg/const/impulse
                  t1Start,...            % time at which first primitive starts
                  t1End,...              % time at which first primitive ends
                  t2Start,...            % time at which second primitive starts
                  t2End,...              % time at which second primitive ends
                  tAvgIndex              % Avg time
                  ];                     % [actnClass,avgMagVal,rmsVal,glabel1,glabel2,t1Start,t1End,t2Start,t2End,tAvgIndex]

        % Update index
        index       = match+1;     
        hasNew_cm   = 1;
    
    %% If one composition, then adjust values correspondingly
    elseif(numCompositions==1)
        
        % Average magnitude value 
        avgMagVal = statData(index,1);   

        % MavVal replaces RMS 2013Sept
        rmsVal = statData(index,2);
        % Root mean square
        %rmsVal = avgMagVal;

        % Compute time indeces
        t1Start = statData(index,4);            % Starting time for primitive 1
        t1End   = statData(index,4);            % Ending time for primitive 1                          
        t2Start = statData(index,5);            % Starting time for primitive 1
        t2End   = statData(index,5);            % Ending time for primitive 1. 
        tAvgIndex = (t1Start+t2End)/2;

        % Enter the following data into the motComps structure:
        motComps=[actnClass,...          % type of motion actnClass: "adjustment", "constant", or "impulse". 
                  avgMagVal,...          % Magnitude of data (average value). Needs to be averaged when second match is found
                  rmsVal,...             % Root mean square value
                  amplitudeVal,...       % Largest difference from one edge of p1 to the other edge of p2              
                  glabel1,...            % bpos...snet...impulse
                  glabel2,...            % type of label b/m/s/pos/neg/const/impulse
                  t1Start,...            % time at which first primitive starts
                  t1End,...              % time at which first primitive ends
                  t2Start,...            % time at which second primitive starts
                  t2End,...              % time at which second primitive ends
                  tAvgIndex              % Avg time
                  ];                     % [actnClass,avgMagVal,rmsVal,glabel1,glabel2,t1Start,t1End,t2Start,t2End,tAvgIndex]

        % Update index only by 1, since there are no two contiguous
        % primitives
        index       = index+1;     
        hasNew_cm   = 1;
    else
        motComps    = [0 0 0 0 0 0 0 0 0 0 0];
        index       = index+1;
        hasNew_cm   = 0;
    end
    
end