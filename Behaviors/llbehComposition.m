%% ********************** Documentation ***********************************
% The LLBeh structure holds low-level behaviors for each force dimension.
%
% This level combines motion composition actions to produce lower-level behaviors in states 
% in which there is contact between parts (State 2 through end). This level compares degrees 
% of change across two motion compositions per state. There are six low-level behaviors which 
% are defined as follows:
% �	Fixed
%	Def: Occurs when the part is no longer moving and remains fixed in place.
%	Conditions: No change and a total minimum of time ThreshTime
%	Sequence of mot. Comps: {k, kk}
% �	Contact
%	Def: Occurs when the male part hits the back wall of the female part 
%	Conditions: any action, except for an unstable one, followed by a
%               contact, or pc/nc,nc/pc followed by any action except an unstable action. 
%	Sequence of mot. Comps: { *c*}
% �	Pushing
%	Def: Occurs when a part moves along the negative direction of motion
%	Conditions: action is produced by one or more decrement actions and a total minimum 
%               of time ThreshTime. 
%	Sequence of mot. Comps: {i,ii}.
% �	Pulling
%	Def: Occurs when a part moves along the positive direction of motion
%	Conditions: action is produced by one or more increment actions and a total minimum 
%   of time ThreshTime. 
%	Sequence of mot. Comps: {d,dd}.
% �	Aligning
%	Def: Occurs when contiguous adjustments have smaller amplitudes and share an average 
%        value within 50% of the original one.. 
%	Conditions: action is produced by increasingly smaller adjustments. 
%	Sequence of mot. Comps: {aa}.
% �	Shift
%	Def: Occurs when contiguous adjustments increase in value indicated small alignments over time. 
%	Conditions: action is produced by increasingly smaller adjustments and a total minimum of time 
%               ThreshTime. 
%	Sequence of mot. Comps: {u,uu}.
% �	Noise
%	If any of the previous ones cannot be recognized.
% 
% For Reference:
% actionLbl  = {'a','i','d','k','pc','nc','c','u','n','z');     % String representation of each possibility in the actnClass set.                 
% llbehLbl   = {'FX' 'CT' 'PS' 'PL' 'AL' 'SH' 'U' 'N');         % {'fix' 'cont' 'push' 'pull' 'align' 'shift' 'unstable' 'noise');
% Primitives = [bpos,mpos,spos,bneg,mneg,sneg,cons,pimp,nimp,none]
% llbehStruc = [llBehClass,avgMagVal,avgMagVal,avgMagVal,rmsVal,rmsVal,rmsVal,amplitudeVal,amplitudeVal,amplitudeVal,mc1,mc2,t1Start,t1End,t2Start,t2End,tAvgIndex];    
%
% Input Parameters:
% StrategyType  - what kind of strategy: PA10 Pivot Approach or HIRO Side Approach.
% motCompsFM:   - a nx11 cell array that contains all 11 tags of
%                 information for each motion composition action that took place in each of
%                 the six force axes. 
% plotHanlde    - handle to current FxyzMxyz plot.
% TL            - top limit of axes object.
% BL            - low limit of axes object.
% 
% Output Parameters:
% llBehStruc:     - a nx17 cell array that contains all information about each of the 
%                   low-level behaviors found
% llbehLbl:       - label cell array struc containing strings of possible
%                   llbehaviors
% htext:          - handles to current axes
%**************************************************************************
function [llbehStruc,llbehLbl] = llbehComposition(StrategyType,motComps,curHandle,TL,BL,fPath,StratTypeFolder,FolderName,pType)

%% Debugging Flags and Global Variables
% Global variables defined in the main file for program: SnapVerification.m
%------------------------------------------------------------------------------------------
    global DB_PLOT;                                 % Declared in snapVerification. Enables plotting.
    global DB_WRITE;
%------------------------------------------------------------------------------------------   
    global LLB_REFINEMENT_CYCLES;  
%------------------------------------------------------------------------------------------    
    % Size
    r = size(motComps);
    
    % Threshold Time
    timeThresh=0.10;
    
    % Number of times to refine
    RefinementCycles = 5; % RefinementCycles = LLB_REFINEMENT_CYCLES; Beacuse global variable will be unuseful during parfor.
    
    % Time Flag
    timeCheckFlag = false;  % Used to indicate that a single primitive is longer than the time threshold
    
%%  Structure and indeces for low-level behavior structure

%%  Labels for low-level behaviors
% 	FIX     = 1;        % Fixed in place
%   CONTACT = 2;        % Contact
%   PUSH    = 3;        % Push
%   PULL    = 4;        % Pull
%   ALIGN   = 5;        % Alignment
%   SHIFT   = 6;        % Shift
%   UNSTABLE= 7;        % Unstable
%   NOISE   = 8;        % Noise
%   llbehLbl    = {'FX' 'CT' 'PS' 'PL' 'AL' 'SH' 'U' 'N');
    
%%  Low-Level Behavior Structure (1X17)
    % [llbehLbl	avgVal1 avgVal2 AVG_MAG_VAL rmsVal1 rmsVal2 AVG_RMS_VAL ampVal1 ampVal AVG_AMP_VAL mc1Lbl mc2Lbl t1Start t1End t2Start t2End tAvgIndex]
    llbehStruc = zeros(1,17); % Updated July 2012 %llbehStruc = {[] [] [] [] [] [] [] [] [] [] [] [] [] [] [] [] []);   
    
%%  Motion Composition Labels
    % Labels for actions in motion compositions
    adjustment      = 1;    % a
    increase        = 2;    % i
    decrease        = 3;    % d
    constant        = 4;    % k
    pos_contact     = 5;    % pc
    neg_contact     = 6;    % nc
    contact         = 7;    % c
%   unstable        = 8;    % u
    noise           = 9;    % n
%   none            = 10;   % z
    %actionLbl      = {'a','i','d','k','pc','nc','c','u','n','z'); % String representation of each possibility in the actnClass set.                 
    actionLbl       = [ 1,  2,  3,  4,  5,   6,   7,  8,  9,  10];  % Updated July 2012. Represent the actionLbl's directly by integers. Facilitates conversion into C++
%% Motion Composition Structure Indeces

    % Time
    T1S = 7;
%   T1E = 8;
%   T2S = 9;
    T2E = 10;
    
%% A) Iterate through compositions

    % Go through all motion compositions -1
    index           = 1;
    llbehStrucIndex = 1;
    
    % Examine all the rows. Index check is performed in motCompsMatchEval
    while (index<=r(1)) 
        
%%  	i. Determine if FIXED: check for constant
        if(intcmp(motComps(index,1),actionLbl(constant)))

            % Label type of motion composition
            labelType=actionLbl(constant);
            
            % a. If the first occurence is greater than the time limit
            p1time = motComps(index,T2E)-motComps(index,T1S);  % Get duration of first primitive. From T1START (note) TO T2END
            if(p1time>=timeThresh)
                timeCheckFlag = true;                              
            end

            % b.Is there a match?
            windowSZ=0; 
            [llbehStruc(llbehStrucIndex,:), index, llbehLbl] = motCompsMatchEval(index,labelType,motComps,timeCheckFlag,windowSZ);
            llbehStrucIndex = llbehStrucIndex + 1;
        
%%      Determine IF CONTACT: check for contact,pos_contact, neg_contact
        elseif(intcmp(motComps(index,1),actionLbl(contact)) || ...
                intcmp(motComps(index,1),actionLbl(pos_contact)) || ...
                    intcmp(motComps(index,1),actionLbl(neg_contact)) )
            
            % Label type of motion composition
            labelType=actionLbl(contact);

            % Is there a match?
            windowSZ=0;
            [llbehStruc(llbehStrucIndex,:), index, llbehLbl] = motCompsMatchEval(index,labelType,motComps,0,windowSZ);
            llbehStrucIndex = llbehStrucIndex + 1;
                
%% Determine if PUSH: check for decrease
        elseif(intcmp(motComps(index,1),actionLbl(increase)))

            % Label type of motion composition
            labelType=actionLbl(increase); 
            
            % a. If the first occurence is greater than the time limit
            p1time = motComps(index,T2E)-motComps(index,T1S);  % Get duration of first primitive
            if(p1time>=timeThresh)
                timeCheckFlag = true;                              
            end

            % b. Look for a possible match
            windowSZ = 0;
            [llbehStruc(llbehStrucIndex,:), index, llbehLbl] = motCompsMatchEval(index,labelType,motComps,timeCheckFlag,windowSZ);
            llbehStrucIndex = llbehStrucIndex + 1;

%% Determine if PULL: check for increase
        elseif(intcmp(motComps(index,1),actionLbl(decrease)))
            
            
            % Label type of motion composition
            labelType=actionLbl(decrease);
            
            % a) If the first occurence is greater than the time limit
            p1time = motComps(index,T2E)-motComps(index,T1S);  % Get duration of first primitive
            if(p1time>=timeThresh)
                timeCheckFlag = true;           
            end

            % Is there a match?
            windowSZ = 0;
            [llbehStruc(llbehStrucIndex,:), index, llbehLbl] = motCompsMatchEval(index,labelType,motComps,timeCheckFlag,windowSZ);
            llbehStrucIndex = llbehStrucIndex + 1;

%% Determine if ALIGN: check for adjustment
        elseif(intcmp(motComps(index,1),actionLbl(adjustment)))
           
            % a. If the first occurence is greater than the time limit
            p1time = motComps(index,T2E)-motComps(index,T1S);  % Get duration of first primitive
            if(p1time>=timeThresh)
                timeCheckFlag = true;                              
            end     
            
            % b. Label type of motion composition
            labelType=actionLbl(adjustment);           
            
            % C. Find Match
            windowSZ = 1;
            [llbehStruc(llbehStrucIndex,:), index, llbehLbl] = motCompsMatchEval(index,labelType,motComps,timeCheckFlag,windowSZ);
            llbehStrucIndex = llbehStrucIndex + 1;

%% Determine if SHIFT
%         elseif(intcmp(motComps(index,1),actionLbl(adjustment))))
% 
%             % Label type of motion composition
%             labelType=actionLbl(adjustment);           
% 
%             % Is there a match?
%             windowSZ = 0; 
%             [llbehStruc(llbehStrucIndex,:) index llbehLbl] = motCompsMatchEval(index,labelType,motComps,0,windowSZ);
%             llbehStrucIndex = llbehStrucIndex + 1;

%% Determine if NOISE
        else

            % Label type of motion composition
            labelType=actionLbl(noise);
            
            % Is there a match?
            windowSZ= 0;
            [llbehStruc(llbehStrucIndex,:), index, llbehLbl] = motCompsMatchEval(index,labelType,motComps,0,windowSZ);
            llbehStrucIndex = llbehStrucIndex + 1;

        end
    end            

%% Refine
    for i=1:RefinementCycles
        llbehStruc = Refinement(llbehStruc,actionLbl);
    end
%% Save Data

    % Print llbehStruc  to file
    % Select kind of data motComps = 0; llbehStruc = 1; hlbehStruc = 2;
    if(DB_WRITE)
        saveData = 1;
        dataFlag = 1;
        WriteCompositesToFile(fPath,StratTypeFolder,FolderName,pType,saveData,llbehStruc,dataFlag);
    end

%% Plot

    if(DB_PLOT)
        % Plot the low-level behavior labels. 
        htext = plotLowLevelBehCompositions(StrategyType,curHandle,TL,BL,llbehStruc);

        %  Save plot
        savePlot(fPath,StratTypeFolder,FolderName,curHandle,'llbehPlot'); 
    end
end