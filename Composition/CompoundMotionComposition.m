%%************************ Documentation **********************************
% The compound compositions function, looks for patterns in contiguous primitive
% motion data to create a higher level of abstraction called motion compositions.
% To do so it will call the function primMatchEval to find patterns. We
% want to match according to certain rules as long as the amplitude
% difference of the contiguous compositions is not too different.
% Otherwise, we do not want to create a composition to label that large
% change.
%
% There are seven types of actions that we are interested in:
% actionClass = {
%                adjustment: 'a' can happen under 6 combinations of primitives:   ['l/b/m/s/pos,l/b/m/s/neg']
%                                                                                 ['l/b/m/s/neg,l/b/m/s/neg']
%                increase:   'i' can happen by two positives or pos and const:    ['b/m/s/pos,pos, or pos/const]
%                decrease:   'd' can happen by two negatives or neg and const:    ['b/m/s/neg,neg, or neg/const]
%                constant:   'k' can happen under a single or repeated action of: ['const']
%                contact:    'c' can happen under a single or repeated action of: ['imp']
%                unstable:   'u' can happen with pimp+pos, nimp+neg.
%                none:       'n' no primitive combination match. should not occur.  
%
% This program is intimately connected with: GradientClassification.m in
% the way it labels the graidents of the data. Information in that m-file
% is saved in statData(n,:)=[dAvg dMax dMin dStart dFinish dGradient dLabel]
%
% Categories:                             [actionClass, szValue, gradLabel, tIndexP1, tIndexP2, avgtIndex]
%                       1) If bpos/bneg = [adjustment,   >b,      bpos,      1st=2,    2nd=3,    2.5]
%                       2) If mpos/mneg = [adjustment,   <m&>s,   mneg,      1st=4,    2nd=6,    5.0]
%                       3) If spos/sneg = [adjustment,   <s,      spos,      7,        8,        7.5]
%
% At the end of the routine there is a clean-up sessions that improves
% compositionality and reduces error.
%
% All information is saved to file and labels plotted on corresponding figures
% For reference:    - motComps labels:     ['a','i','d','k','pc','nc','c','u','n','z']
%                   - % motComps           [nameLabel,avgVal,rmsVal,amplitudeVal,p1lbl,p2lbl,t1Start,t1End,t2Start,t2End,tAvgIndex]
%                   - Primitives labels:   [bpos,mpos,spos,
%                                           bneg,mneg,sneg,
%                                           cons,pimp,nimp,none]
%***********************************************************************************************************
% Input Parameters:
% StrategyType          - What Strategy are we using? HiroSideApproach,PA10 PivotApproach?
% statData:             - CELL array containing 7 elems of info for a
%                         primitive motion segment [avg,max,min,start_time,finish_time,gradient,gradientlbl]. 
% saveData:             - If want to save .mat to file
% gradLabels:           - gradient classification structure. Defined in fitRegressionCurves.m
% rHandle:              - handle for the current axes out of the possible 8
%                         SJ1, SJ2, Fx, Fy, Fz, Mx, My, Mz. 
% TL:                   - the top_limit for the current axis. Used for plotting. 
% BL:                   - equivalent lower limit
% fPath,FolderName:
% StratTypeFolder:      - used to set path directory for plotting
% pType:                - type of plot 'Fx'...'Mz'
% stateData:            - contains vector elements that indicate the time
%                         when a state starts.
%*********************************************************************************************************
% Ouput Parameters:
% motComps:             - data structure that contains all compound motion composition information
%                         data struc is a cell array:
%                         [class,avgMagVal,rmsVal,AmplitudeVal,glabel1,glabel2,t1Index,t2Index,tAvg]
%***************************************************************************************************************
function motComps=CompoundMotionComposition(StrategyType,statData,saveData,gradLabels,rHandle,TL,BL,...
                                                        fPath,StratTypeFolder,FolderName,pType,stateData)
   
%% GLOBAL VARIABLES AND DEBUGGING FLAGS
% Global variables are set in the main program file: SnapVerification
%--------------------------------------------------------------------------
    global DB_PLOT;                                 % Declared in snapVerification. Enables plotting.
    global DB_WRITE;
%--------------------------------------------------------------------------    
    global MC_COMPS_CLEANUP_CYCLES;
%--------------------------------------------------------------------------
%% Initialization
    %----------------------------------------------------------------------
    motCompsIndex   = 1;                        % Index for motion compositions
    dataSz          = 11;                        % Composition Data: [class,avgMagVal,rmsVal,AmplitudeVal,glabel1,glabel2,t1Index,t2Index,tAvg]. See primMatchEval.
    
    % 1x9 Cell Data Structure for motion compositions.
    % [class,avgMagVal,rmsVal,AmplitudeVal,glabel1,glabel2,t1Index,t2Index,tAvg]
    motComps        = zeros(50,dataSz);                                   
    
    % gradLabels CELL string array. Indicates whether prim is
    % b/m/s/pos/net/const/imp. See GradientClassification.m                               
    
    % Get size of relevant data
    rStat   = size(statData);           % statData holds mx7 pieces of statistical data
    %rLbl    = length(szLabel);         % length of eight element gradient classification struc (can change in future)

%%  Analyze Motion Primitives

    % 1) Search for categories using a window of size w.
    % Classify as follows (size, class, value)
    % 2) If bpos+bneg = [big,  alignment,pos]
    % 3) If mpos+mneg = [med,  alignment,pos]
    % 4) If spos+sneg = [small,alignment,pos]
    % 5-9) Same for negative, const,pimp,nimp 
    
%% A. Search the whole space of primitives
    index = 1;
    while (index<=rStat(1)) % Iterate through the rows. primMatchEval updates the index.
        

%%      B. Search for positive, negative, const, impulse gradLabels
        for lbl=1:3            

%%          i) Check for match with bpos, mpos, spos. 
            if(strcmp(gradInt2gradLbl(statData(index,7)),gradLabels(lbl,:)))           %b/m/s/pos
                labelType = 'positive'; 

                % C. Find Match
                [motComps(motCompsIndex,:),index,actionLbl] = primMatchEval(index,labelType,lbl,statData,gradLabels);
                motCompsIndex = motCompsIndex + 1;
                break;      % break the for loop
%%          ii) Check for match with bneg, mneg, sneg                
            elseif(strcmp(gradInt2gradLbl(statData(index,7)),gradLabels(lbl+3,:)))           %b/m/s/neg
                labelType = 'negative'; match_lbl=lbl+3;

                % C. Find Match
                [motComps(motCompsIndex,:),index,actionLbl] = primMatchEval(index,labelType,match_lbl,statData,gradLabels);                   
                motCompsIndex = motCompsIndex + 1;                
                break;      % break the for loop
%%          iii) Check for match with constant                
            elseif(strcmp(gradInt2gradLbl(statData(index,7)),gradLabels(7,:)))
                labelType = 'constant'; match_lbl = 7;

                % C. Find Match
                [motComps(motCompsIndex,:),index,actionLbl] = primMatchEval(index,labelType,match_lbl,statData,gradLabels);                   
                motCompsIndex = motCompsIndex + 1;                
                break;      % break the for loop                
%%          iv) Check for match with positive impulse,pimp               
            elseif(strcmp( gradInt2gradLbl(statData(index,7)), gradLabels(8,:)))
                labelType = 'pimp'; match_lbl = 8;

                % C. Find Match
                [motComps(motCompsIndex,:),index,actionLbl] = primMatchEval(index,labelType,match_lbl,statData,gradLabels);                   
                motCompsIndex = motCompsIndex + 1;
                break;      % break the for loop                

%%          v) Check for match with negative impulse, nimp 
            elseif( strcmp(gradInt2gradLbl(statData(index,7)), gradLabels(9,:)) )
                labelType = 'nimp'; match_lbl = 9;

                % C. Find Match
                [motComps(motCompsIndex,:),index,actionLbl] = primMatchEval(index,labelType,match_lbl,statData,gradLabels);                   
                motCompsIndex = motCompsIndex + 1;
                break;      % break the for loop                
            end
        end     % End gradLabels iteration for labels                                               
    end             % End primitive space 
   
%% D. Resize motComps in case of empty cells
    motComps = resizeData(motComps);
    
%% E. CleanUp the motion compositions
    % PA10 Pivot Approach
    if(~strcmp(StrategyType,'HSA') && ~strcmp(StrategyType,'ErrorCharac'))
        CleanLoops = 4; %CleanLoops = MC_COMPS_CLEANUP_CYCLES; Beacuse global variable will be unuseful during parfor.
    % HIRO Side Approach
    else
        CleanLoops = 4; %CleanLoops = MC_COMPS_CLEANUP_CYCLES; Beacuse global variable will be unuseful during parfor.
    end
    
    % Issue cleaning
    for i=1:CleanLoops
        motComps = cleanUp(StrategyType,motComps,stateData,gradLabels,actionLbl);
    end
    
%% F. Save Data

    % Print motComps structure to file
    % Select kind of data motComps = 0; llbehStruc = 1; hlbehStruc = 2;
    if(DB_WRITE)
        dataFlag = 0;
        WriteCompositesToFile(fPath,StratTypeFolder,FolderName,pType,saveData,motComps,dataFlag);
    end
    
%% G. Plot Data
        
        % Plot the compound motion compositions, given the right axes and
        % top-limit boundary. 
        if(DB_PLOT)
            plotMotionCompositions(StrategyType,rHandle,TL,BL,motComps);
        end
        
end     % End function
    
