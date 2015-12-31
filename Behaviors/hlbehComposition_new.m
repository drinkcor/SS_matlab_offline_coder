%% ****************************** Documentation ***************************
% Latest Update: July 2013 (002). // Jan 2013 (001).
% 
% This code originally was developed for the PivotApproach/PA10 Simulation.
% We want to extend this to include SideApproach HIRO in Simulation/Physical Experiment.
% A new revision has come for it in July 2013 as we prepare the
% In July, 2013, we work to include the ErrorCharacterization and now this code will totally work without cells.
%
% The fifth layer of the taxonomy looks one state-at-a-time (context
% specific) 
% across all six force-moment low-level behaviors to produce high-level behaviors. 
% The high-level behaviors represent human-apropos behaviors associated with 
% each stage of the pivot assembly process. Here is a list across states:
%
%   �	State 1: Approach 	Y/N
%   �	State 2: Rotation	Y/N
%   �	State 3: Alignment 	Y/N << For PivotApproach/PA10 but not for SideApproach/HIRO
%   �	State 4: Snap 		Y/N
%   �	State 5: Mating 	Y/N
% 
% Each of the high-level behaviors requires a specific combination of low-level 
% behaviors across the different force-elements but not necessarily all of them. 
% The key is that if certain key low-level behaviors are present, the presence of 
% the high-level behavior can be ascertained. 
%
% Note: for State 1
%   State 1: Failure cases: most easily interpreted here. Study differnt conditions
%            to understand nature of failure if present.
%            If successful returns 1 for this state.
% 
% Here is the list of necessary state-sensitive low-level behavior requirements. 
%
% PivotApproach - PA10 - Simulation
% In other words, if the low-level behaviors are present (or a sequence of them are 
% present) then, we have a higher-level behavior. If not, we have the negative form 
% of the high-level behavior. 
%
%   �	Rotation: 
%       o	Fz-> FX (with value not equal to zero)
%       o	Fy -> PL 
%       o	Mx -> ALIGN
%   �	Alignment
%       o	AL to show up in all axes (in our present case Fxyz, Mxyz). However, 
%           the moment axis corresponding to the direction of motion in which the 
%           insertion is taking place (Mz) could have just a FX reference or ALIGN->FS instead. 
%   �	Snap
%       o	Fz � CT+AL
%       o	FxFyMxMyMz = ALIGN+FX || FX
%
%   �	Mating
%       o	Fx-Mz = ALIGN+FX || FX
%
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
% INPUTS
% SideApproach - Type of experiment/simulation
% keyLLB(axes) = KeyLLBLookUp(StrategyType,HLB(hlbTag,:),axes);
%
% This layer has a struc that lists the low-level behaviors contained in each state for each force axis
% 
% hlbehStruc = [ 
%               stateLbl1[ Fx[]; Fy[]; Fz[]; Mx[]; My[]; Mz[] ] : Approach
%               stateLbl2[ Fx[]; Fy[]; Fz[]; Mx[]; My[]; Mz[] ] : Rotation
%               stateLbl3[ Fx[]; Fy[]; Fz[]; Mx[]; My[]; Mz[] ] : Insertion
%               stateLbl4[ Fx[]; Fy[]; Fz[]; Mx[]; My[]; Mz[] ] : Mating
%              ]
%
% Input Parameters:
% motCompsFM    - Motion composition structure: mx11x6 (Num of MCs, 11 elements, 6 axis)    
% MCnumElems    - Number elements contained in the length of motCompsFM
% llbehFM:      - Low-level Beh structure: mx17x6 (Num of LLBs, 17 data elements, 6 axis)
% llbehNumElems - number of elements contained in the length of llbehFM
% llbehLbl:     - Array structure that contains a list of strings of 
%                 possible low-level behaviors.
% stateData:    - 4x1 vector that contains the times at which states for
%                 the pivot approach start. The first element refers to
%                 state 2. Will insert state 1 at the initialization stage.
% curHandle:    - handle to axes in figure.
% TL/BL:        - top and bottom limits of all axes in the subplot fig.
% fPath,StratTypeFolder,FolderName
%
% Output Parameters:
% hlbehStruc:   - a 1x5 numeric array that holds 1's or 0's, determing
%                 whether each state produced a successful:
%               - For PA10:
%                    - APPROACH
%                    - ROTATION
%                    - ALIGHMENT
%                    - INSERTION
%                    - MATING
%               - For HIRO:
%                    - APPROACH
%                    - ROTATION
%                    - INSERTION
%                    - MATING
%
% avgMyData                 - average value for all data structures computed in failureCharacterization
% snapVerificationSuccess   - true if all states succeeded; false otherwise
% bool_fcData               - boolean structure that contains data for each
%                             of the tests carried out in the xDir, yDir, xRollDir
%**************************************************************************
function [hlbehStruc,avgMyData,snapVerificationSuccess,bool_fcData] = hlbehComposition_new(motCompsFM,mcNumElems,llbehFM,LLBehNumElems,...
                                                                                           llbehLbl,stateData,...
                                                                                           curHandle,TL,BL,...
                                                                                           fPath,StratTypeFolder,FolderName)
   
%% Globals
    global DB_PLOT;     % This global variable determines if we print plots.
    global DB_WRITE;    % This global variable determines if we write data to file.

%%  Structure and indeces for low-level behavior structure
%%  Labels for low-level behaviors
 	FIX     = 1;        % Fixed in place
    CONTACT = 2;        % Contact
%   PUSH    = 3;        % Push
    PULL    = 4;        % Pull
    ALIGN   = 5;        % Alignment
    SHIFT   = 6;        % Shift
%   UNSTABLE= 7;        % Unstable
%   NOISE   = 8;        % Noise
%   llbehLbl    = {'FX' 'CT' 'PS' 'PL' 'AL' 'SH' 'U' 'N'}; % {'fix' 'cont' 'push' 'pull' 'align' 'shift' 'unstable' 'noise'};
    
%% Indeces for llbehStruc
%   behLbl          = 1;   % action class
%   averageVal1     = 2;   % averageVal1
%   averageVal2     = 3;
%   AVG_MAG_VAL     = 4;
%   rmsVal1         = 5;
%   rmsVal2         = 6;
%   AVG_RMS_VAL     = 7;
%   ampVal1         = 8;
%   ampVal2         = 9;
%   AVG_AMP_VAL     = 10;
%   mc1             = 11;
%   mc2             = 12;    
    T1S             = 13; 
%   T1E             = 14;
%   T2S             = 15; 
    T2E             = 16;    
%   TAVG_INDEX      = 17;
     
%% Initialization    
    
    % Compute the number of low-level behaviors per force/moment axis.
    [~,~,NumForceAxis] = size(llbehFM);       % Need to hardcode this as namefields returns a cell array. Currently expect 6 for FxyzMxyz
    
%%  Structure Size    
    % Create a matrix to keep the dimensions of each LLB struc. A (6,m)
    % matrix where m is the number of LLBs for a given axis. 
    %strucSize = zeros(NumForceAxis,1);  % How many LLBs in each axis?  // this is older comments... not applicable. 6axis x 2(rows=#LLBs, cols=17)   
    
    % Fill in strucSize with the size of each of the six llbehStruc's
    %for i=1:NumForceAxis
    strucSize = LLBehNumElems;   %numElems was computed in zeroFill and returns the last non-zero entry for each of the siz llbeh structures.
    %end


%%  State: 
    rState      = size(stateData);
    if(~strcmp(StratTypeFolder,'ForceControl/HIRO/') && ~strcmp(StratTypeFolder,'ForceControl/ErrorCharac/'))
        
        % Only when all states where accomplished and there is a terminating time, do we want to subtract 1 to enumerate the number of states
        if(rState==5)
            StateNum    = rState(1)-1;        % STATE VECTOR MUST INCLUDE TASK'S ENDING TIME. We subtract one b/c there is no upper boundary after 4
        else
            StateNum    = 0;                  % DQ adding, in case the following StateNum are used before assigned
        end
    % PA10 Experiments have one more state than the HIRO Side Approach, because they include Alignment
    else
        
        % Only when all states where accomplished and there is a terminating time, do we want to subtract 1 to enumerate the number of states
        if(rState==6)
            StateNum = rState(1)-1;
        % Failure case scenarios where there are less than the complete number of states
        else
            StateNum = rState(1)-1;        
        end
    end
    
    % Create an automata state array to hold LLB labels (now used integers
    % instead of strings) in the six FT dimensions for each automata state (except Approach state).
    % stateLbl = cell{StateNum,NumForceAxis};   % Used in PivotApproach/PA10 and is composed of a 5x6 structure.
    stateLbl = zeros(StateNum,1,NumForceAxis);  % And a 4,m,6 structure that will grow m over time for SideApproach/HIRO
                                                % Will save all the LLBeh names
    
%%  High-Level Behavior Structure
%   1xStateNum vector of 1's and 0's, dictating whether or not HL Behs were achieved.
	hlbehStruc = zeros(1,rState(1)-1);      % Currently 5 States for PivotApproach 
                                            % Currently 4 states for Side Approach

%% PivotApproach/PA10 Code  This case's codes are commented, because it's for old robot
    if(~strcmp(StratTypeFolder,'ForceControl/HIRO/') && ~strcmp(StratTypeFolder,'ForceControl/ErrorCharac/'))
        
%         
%         %% (1) Create a state x ForceElments Cell array structure
% 
%         % Keep a counter of which labels belong to a given state
%         llLabelVector = zeros(4,1);                  % Empirically determined size with int inputs.
% 
%         % Fill each state's dimension with llbeh sequence of labels. 
%         for state=1:StateNum
% 
%         %% Define TIME limits of prevprev/prev/current/next/nextnext states
%         % If only 1 state, a behavior can only be in: current state.
%         % 2 states: behavior can be in: previous state and current state, or current state and next state.
%         % 3 states: prevprev/prev/curr or prev/curr/next or curr/next/nextnext
%         % 4 states: ppp/pp/p/c or pp/p/c/n or p/c/n/nn or c/n/nn/nnn
%         % Visualization
%         % _____________________________________________
%         % |          |          |          |          |
%         % sApp      eApp
%         %           sRot       eRod
%         %                      sIns       eIns
%         %                                 sMat        eMat
%             currStateEndTime   = stateData(state+1);        % Add 1 to capture the ENDING time
% 
%             % PrevPrevState
%             if(state<3),                prevprevStateEndTime    = 0;
%             else                        prevprevStateEndTime    = stateData(state-2);
%             end
% 
%             % PrevState
%             if(state<2),                prevStateEndTime        = 0;
%             else                        prevStateEndTime        = stateData(state);     
%             end        
% 
%             % NextState (only applies if there are at laest 3 entries)
%             if(StateNum>=3)
%                 if(StateNum-state>=2),    nextStateEndTime      = stateData(state+2);  % Do this computation if the total number of states is at least 2 numbers bigger than our current state. Before we had the following line that now has been updated: if(state < StateNum)
%                 elseif(state == StateNum),nextStateEndTime      = currStateEndTime;
%                 end
%             end
% 
%             % NextNextState
%             if(StateNum>=4)
%                 if(StateNum-state>=3),    nextnextStateEndTime  = stateData(state+3);    % Do this if ther eare at least 3 more states than current state. Before we had the following line: if(state < StateNum-1)
%                 elseif(state == StateNum),nextnextStateEndTime  = nextStateEndTime;
%                 end
%             end
% 
% 
%             %% For each AXIS FxyzMxyz, extract the labels according to time
%             for axis=1:NumForceAxis
% 
%                 % 1. Extract the llbehStruc data for each of the six dimensions
%                 llbehStruc = llbehFM(:,:,axis);
% 
%                 % 2. For each label in llbehStruc. Traversing the structure. 
%                 for index=1:strucSize(axis,1)    
% 
%                     % 3. Extract a time vector
%                     timeVec = [llbehStruc(index,T1S:T2E)];
%                     minTime = min(timeVec);
%                     maxTime = max(timeVec);      
%                     if(maxTime>8.3)
%                         maxTime=8.3;
%                     end
% 
%                     % Get state limits
%                     for stateTime=1:length(stateData)-1;
%                         % Check to see in which state the min time starts.
%                         s1 = stateData(stateTime);
%                         s2 = stateData(stateTime+1);
%                         % Lowerbound 
%                         if(minTime>=s1 && minTime <= s2)
%                             lowerState = stateTime;
%                         end
%                         % Upperbound
%                         if(maxTime>=s1 && maxTime <= s2)
%                             upperState = stateTime;
%                         end                    
%                     end
% 
%                     %% Put in stateLbl
%                     for tt=lowerState:upperState
% 
%                         % Just make a copy of the current state we are in to
%                         % avoid repetion in future states
%                         if(tt==state)
%                             % Compute length of labels in desired state
%                             curLen = length(stateLbl(tt,axis));
% 
%                             % Copy current StateLbl to temp cell
%                             temp = stateLbl(tt,axis);
% 
%                             % Place the new label in temp
%                             temp(1,curLen+1) = llbehStruc(index,1);
%                             
%                             % Copy back to stateLbl
%                             stateLbl(tt,axis) = temp;
%                         end
%                     end
% 
% 
%                 end
%             end
%         end 
% 
%         %% (2) Look for patterned sequence of low-level behaviors to determine if hlbeh's are present
% 
%         Fx=1;Fy=2;Fz=3;Mx=4;My=5;Mz=6;
%         state2=2; snapState=3; matState=4; state5=5;
%         %%  Rotation (State 2). Conditions:
%         %       Fy -> PL   
%         %       Fz-> FX (with value not equal to zero)  
%         %       Mx -> ALIGN
% 
%         % Save the contents of Fy, Fz, Mx in cell arrays for PA10 Robot
%         tempFy=stateLbl{state2,Fy}; tempFz=stateLbl{state2,Fz}; tempMx=stateLbl{state2,Mx};
% 
%         % Look for conditions    
%         if(findStrings(tempFy,llbehLbl{PULL}))
%             if(findStrings(tempFz,llbehLbl{FIX}))
%                 if(findStrings(tempMx,llbehLbl{ALIGN}))
%                     % All three conditions have been met. Set hlbehStruc for state 1 and 2 to true
%                     hlbehStruc(1:2) = 1;
%                 end
%             end
%         end
% 
%         %%  ALIGNMENT
%         %   Conditions: AL || SH + FX to show up in all axes (in our present case Fxyz, Mxyz). 
%         %   However, the moment axis corresponding to the direction of motion in which 
%         %   the insertion is taking place (Mz) could have just a FX reference or ALIGN->FX instead. 
% 
%         % Save the contents of Fx, Fy, Mx
%         tempFx=stateLbl{snapState,Fx}; tempFy=stateLbl{snapState,Fy}; tempFz=stateLbl{snapState,Fz};
%         tempMx=stateLbl{snapState,Mx}; tempMy=stateLbl{snapState,My}; tempMz=stateLbl{snapState,Mz};
% 
%         if( findStrings(tempFx,llbehLbl{ALIGN})|| findStrings(tempFx,llbehLbl{SHIFT},llbehLbl{FIX})) 
%             if( findStrings(tempFy,llbehLbl{ALIGN})|| findStrings(tempFy,llbehLbl{SHIFT},llbehLbl{FIX})) 
%                 if( findStrings(tempFz,llbehLbl{ALIGN})|| findStrings(tempFz,llbehLbl{SHIFT},llbehLbl{FIX})) 
%                     if( findStrings(tempMx,llbehLbl{ALIGN})|| findStrings(tempMx,llbehLbl{SHIFT},llbehLbl{FIX})) 
%                         if( findStrings(tempMy,llbehLbl{ALIGN})|| findStrings(tempMy,llbehLbl{SHIFT},llbehLbl{FIX})) 
%                             if( findStrings(tempMz,llbehLbl{ALIGN},llbehLbl{FIX}) || findStrings(tempMz,llbehLbl{FIX})) 
% 
%                                 % All three conditions have been met. Set hlbehStruc for state 1 and 2 to true
%                                 hlbehStruc(3) = 1;
%                             end
%                         end
%                     end
%                 end
%             end
%         end
% 
%         %%  SNAP INSERTION    
%         %   Conditions: Fz = CT+AL and FxFyMxMyMz = ALIGN+FIX || SH + FIX || FX
% 
%         % Save the contents of Fx, Fy, Mx
%         tempFx=stateLbl{matState,Fx}; tempFy=stateLbl{matState,Fy}; tempFz=stateLbl{matState,Fz};
%         tempMx=stateLbl{matState,Mx}; tempMy=stateLbl{matState,My}; tempMz=stateLbl{matState,Mz};
% 
%         if( findStrings(tempFx,llbehLbl{ALIGN},llbehLbl{FIX}) || findStrings(tempFx,llbehLbl{FIX})) 
%             if( findStrings(tempFy,llbehLbl{ALIGN},llbehLbl{FIX}) || findStrings(tempFy,llbehLbl{FIX})) 
%                 if( findStrings(tempFz,llbehLbl{CONTACT},llbehLbl{ALIGN})) 
%                     if( findStrings(tempMx,llbehLbl{ALIGN},llbehLbl{FIX}) || findStrings(tempMx,llbehLbl{FIX})) 
%                         if( findStrings(tempMy,llbehLbl{ALIGN},llbehLbl{FIX}) || findStrings(tempMy,llbehLbl{FIX})) 
%                             if( findStrings(tempMz,llbehLbl{ALIGN},llbehLbl{FIX}) || findStrings(tempMz,llbehLbl{FIX})) 
% 
%                                 % All three conditions have been met. Set hlbehStruc for state 1 and 2 to true
%                                 hlbehStruc(4) = 1;
%                             end
%                         end
%                     end
%                 end
%             end
%         end
% 
%         %%  MATING    
%         %   Conditions: Fz = FX
% 
%         % Save the contents of Fx, Fy, Mx
%         tempFx=stateLbl{state5,Fx}; tempFy=stateLbl{state5,Fy}; tempFz=stateLbl{state5,Fz};
%         tempMx=stateLbl{state5,Mx}; tempMy=stateLbl{state5,My}; tempMz=stateLbl{state5,Mz};
% 
%         if( findStrings(tempFx,llbehLbl{FIX}) ) 
%             if( findStrings(tempFy,llbehLbl{FIX}) ) 
%                 if( findStrings(tempFz,llbehLbl{FIX}) ) 
%                     if( findStrings(tempMx,llbehLbl{FIX}) ) 
%                         if( findStrings(tempMy,llbehLbl{FIX}) ) 
%                             if( findStrings(tempMz,llbehLbl{FIX}) ) 
% 
%                                 % All three conditions have been met. Set hlbehStruc for state 1 and 2 to true
%                                 hlbehStruc(5) = 1;
%                             end
%                         end
%                     end
%                 end
%             end
%         end


    %% Code for HIRO Simulation
    else
        %% (1) Create a state x ForceElments array structure

        % Keep a counter of which labels belong to a given state
        %llLabelVector = zeros(StateNum,1);                  % Empirically determined size.

        % Fill each state's dimension with llbeh sequence of labels. 
        for state=1:StateNum

        %% Define TIME limits of prevprev/prev/current/next/nextnext states
            currStateEndTime   = stateData(state+1);        % Add 1 to capture the ENDING time

            % PrevPrevState
            if(state<3),            prevprevStateEndTime    = 0;
            else                    prevprevStateEndTime    = stateData(state-2);
            end

            % PrevState
            if(state<2),            prevStateEndTime        = 0;
            else                    prevStateEndTime        = stateData(state);     
            end        

            % NextState
            if(state < StateNum),     nextStateEndTime      = stateData(state+2);
            elseif(state == StateNum),nextStateEndTime      = currStateEndTime;
            end

            % NextNextState
            if(state < StateNum-1),   nextnextStateEndTime  = stateData(state+3);
            elseif(state == StateNum),nextnextStateEndTime  = nextStateEndTime;
            end

            
            %% Method:
            %% 1) Separate LLBeh labels from our large llbehFM structure, to llbehStruc. Do this in a loop for each axis.
            %% 2) For each axis, we then need to separate labels according to automata state. We use the labels start and end-times to understand in which states they are present.
            %% 3) Once we have segemented the labels, we look into each state for key LLBs to see if the assembly was successful.
            %% For each AXIS FxyzMxyz, extract the labels according to time
            for axis=1:NumForceAxis

                % 1. Extract the llbehStruc data for each of the six dimensions. It will contain the LLB label + all statistics.
                llbehStruc = llbehFM(1:LLBehNumElems(axis),:,axis); % Here we want to assign not the padded structure, but the structure with the real data, which is smaller in dimension. 

                % 2. Traverse the llbehStruc for all of its label itmes accross the 6 axes
                for index=1:strucSize(axis)    

                    % 3. Extract a time vector for the current automata state
                    timeVec = llbehStruc(index,T1S:T2E);  % Start and End time for LLB corresponding to index
                    minTime = min(timeVec);                 % Starting time
                    maxTime = max(timeVec);                 % Ending time (can be longer than one automata state)
                    if(maxTime>llbehStruc(end,T2E))
                        maxTime=llbehStruc(end,T2E);
                    end

                    % Get state limits for the current state We will loop through all state boundary times. 
                    % If the startingLLB time is within startState_i Time/endState_i Time then set the 'lowerState' boundary. Ie. llbeh start at 8.5 and say automata state 3 start at 8.4 and ends in 8.6, then that beh is assigned to start in state 3. 
                    % Similary, if the endingLLB time is within startState_i Time/endState_i then set that 'upperState' boundary. 
                    for stateTime=1:length(stateData)-1; % Assumes that the stateVector's last entry is equal to the end of the task. -1 is needed to compute s2.
                        % Check to see in which state the min time starts.
                        s1 = stateData(stateTime);      % startAutomataState
                        s2 = stateData(stateTime+1);    % endAutomataState
                        
                        % Set a lower and uperState bounds. First, Lowerbound for current state
                        if(minTime>=s1 && minTime <= s2)
                            lowerState = stateTime;
                        end
                        % Upperbound for current state
                        if(maxTime>=s1 && maxTime <= s2)
                            upperState = stateTime;
                        end                    
                    end

                    %% The next section was originally designed to work with a cell.
                    %% Fill in the stateLbl Matrix. Need to go through each (i) automata state, (ii) through each axis, (iii) through each LLB (in that order) and fill in this vector of ints. We will have a (4states,m LLB entries,6 force axis). If 0's, it means a null entry. This matrix will have many zeros because matlab has to keep matrix 2D size the same across the third dimension.
                    for tt=lowerState:upperState % Indicates that this LLB spans all these states
                        
                        % Copy the relevant LLB labels that belong to the state in turn. We have a big for loop going around all the states. 
                        if(tt==state)
                            
                            % 1. Compute length of labels in desired state
                            % Structure: 6 axis, each axis with 4 states, length=max num of labels. Start witha ll zero's, and we will iteratively fill it up. 
                            [~, stateLblEntry] = min(stateLbl(tt,:,axis)); % This vector will have one or more zero's. Find the first entry that contains a zero. That will be where our next entry will be.

                            % 2. Place the new LLB label from llbehStruc into temp for the relevant state and axis in turn. 
                            stateLbl(tt,stateLblEntry,axis) = llbehStruc(index,1);
                            stateLbl(tt,stateLblEntry+1,axis) = 0;
                            
                        end % End if(tt==state)
                    end     % End for tt    =lowerState:upperState
                end         % End for index =1:strucSize(axis,1) 
            end             % End for axis  =1:NumForceAxis
        end                 % End for state =1:StateNum

        %% (2) Look for patterned sequence of low-level behaviors to determine if hlbeh's are present
        approachState=1; rotState=2; snapState=3; matState=4; % Rotation, Insertion, and Mating.
        
        % Perform the following checks according to the size of the stateData vector. Check for failure in the Approach stage. If no failure, then
        %% Approach (State 1). Check to verify failure, if not assume success.
        
        if(rState(1)>1) % I.e. Do this if there is: [ApproachStart,ApproachEnd]
             [bool_fcData,avgMyData]=failureCharacterization(fPath,StratTypeFolder,stateData,motCompsFM,mcNumElems,llbehFM,LLBehNumElems,approachState);
             
             % Study Outcomes: if any of the following are true, there was failure. 
             if(sum(bool_fcData(:,1))) 
                 fcResult=1;            % If true, something failed.
             else
                 fcResult=0;            % First two zeros indicate no failure found, the other 5 indeces mean no condition to identify failure were found
             end
             
            %% Failure Specific Steps
            if(fcResult) % Indicates failure.
                % Do recovery steps and then...
                
            %% Indicate success
            else
                hlbehStruc(1,approachState)=1;
            end
            
%             % Fill the structure in order
%             stateLLBstruc.Fx=PULL;   stateLLBstruc.Fy=PUSH;    stateLLBstruc.Fz=PUSH;  stateLLBstruc.Mx=[];    stateLLBstruc.My=FIX;   stateLLBstruc.Mz=[];
% 
%             % Check for presence of labels in desired axis and states
%             llbIsInAxis = checkLLBExistance( stateLbl, approachState, llbehLbl, stateLLBstruc);
%             if(llbIsInAxis); hlbehStruc(1,approachState)=1; end
        else  % DQ adding in case that Output argument "avgMyData" (and maybe others) not assigned during call to hlbehComposition_new
            a_numSet      =4;
            a_numParams   =2;
            avgMyData = zeros(a_numSet,a_numParams);
            b_numSet      =3;
            b_numParams   =7;
            bool_fcData = zeros(b_numSet,b_numParams);

        end
        %%  ROTATION (State 2). Conditions:
        %       Fx-> FX (with value not equal to zero)    
        %       My-> Fx
        
        if(rState(1)>2 && fcResult==0) % Check if endRot exists [ApproachStart,ApproachEnd,RotationEnd]
            % Fill the structure in order
            stateLLBstruc.Fx=FIX;   stateLLBstruc.Fy=[];    stateLLBstruc.Fz=[]; 
            stateLLBstruc.Mx=[];    stateLLBstruc.My=FIX;   stateLLBstruc.Mz=[];

            % Check for presence of labels in desired axis and states
            llbIsInAxis = checkLLBExistance( stateLbl, rotState, llbehLbl, stateLLBstruc);
            if(llbIsInAxis); hlbehStruc(1,rotState)=1; end
        end
        %%  INSERTION    
        %   Conditions: Fx = CT and My = CT
        
        if(rState(1)>3 && fcResult==0) % Do it if endSnap exists [ApproachStart,ApproachEnd,RotationEnd,InsertionEnd]
            % Fill the structure in order
            stateLLBstruc.Fx=CONTACT;   stateLLBstruc.Fy=[];        stateLLBstruc.Fz=[]; 
            stateLLBstruc.Mx=[];        stateLLBstruc.My=CONTACT;   stateLLBstruc.Mz=[];

            % Check for presence of labels in desired axis and states
            llbIsInAxis = checkLLBExistance( stateLbl, snapState, llbehLbl, stateLLBstruc);
            if(llbIsInAxis); hlbehStruc(1,snapState)=1; end
        end
        %%  MATING    
        %   Conditions: Fx-Mz = FX or AL
        
        if(rState(1)>4 && fcResult==0) % Do it if endMat exists [ApproachStart,ApproachEnd,RotationEnd,InsertionEnd,MatingEnd]
            % Fill the structure in order
            stateLLBstruc.Fx=[FIX,ALIGN];   stateLLBstruc.Fy=[FIX,ALIGN];   stateLLBstruc.Fz=[FIX,ALIGN]; 
            stateLLBstruc.Mx=[FIX,ALIGN];   stateLLBstruc.My=[FIX,ALIGN];   stateLLBstruc.Mz=[FIX,ALIGN];
            % Check for presence of labels in desired axis and states
            llbIsInAxis = checkLLBExistance( stateLbl, matState, llbehLbl, stateLLBstruc);
            if(llbIsInAxis);hlbehStruc(1,matState)=1;end       
        end
    end % End if(strcmp(StratTypeFolder,'\\ForceControl\\HIRO\\'))
    
%% Compute successFlag
    snapVerificationSuccess=all(hlbehStruc); % Returns true if all states are successful
    
%% Plot
    if(DB_PLOT)
    	plotHighLevelBehCompositions(curHandle,TL,BL,hlbehStruc,stateData,fPath,StratTypeFolder,FolderName);
    end
%% Save to File
    if(DB_WRITE)
        pType=-1;   % Normally used to pass a stringed array of ['Fx'...'Mz']
        saveData=0; % Flag indicating whether this data should be saved as a .mat. Will already save to .txt.
        dataFlag = 2; % 2 represents that we want to save the higher-level behavior structure
        WriteCompositesToFile(fPath,StratTypeFolder,FolderName,pType,saveData,hlbehStruc,dataFlag); % This function can save data to file for motion compositions, llbehaviors, and hlbehaviors
    end
%% End of Function
end