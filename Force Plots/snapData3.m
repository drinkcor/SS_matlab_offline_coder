%%************************* Documentation *********************************
% snapData3:
% 1) Loads the force-torque data from file. 
% 2) Not in online implementation: Plots the data in a single plot if plotOptions = 0, or subplots if
% plotOptions = 1. 
% All plots are ajusted for best viewing peformance:
% - TOP & BOTTOM max limits
% - Duration of plots
%
% Input Parameters:
% StrategyType:     - Changes code according to strategy type, ie. 'HSA'
% FolderName:       - Folder where data is saved
% plotOptions       - tells whether plots should be graphed in subplots or
%                     not.
%**************************************************************************
function [fPath,StratTypeFolder,localForceData,stateData,handles,TOP_LIMIT,BOTTOM_LIMIT]...
                                        =snapData3(StrategyType,FolderName,plotOptions)

%% INTIALIZATION
    global DB_PLOT;         % Declared in snapVerification. Enables plotting.
    
    % Switch Flag
    SWITCH = 1;             % Used to determine whether to turn on margin's around plots for extra space when adjusting. 
    
%% Assing appropriate directoy based on Ctrl Strategy to read data files
    StratTypeFolder = AssignDir(StrategyType);
    
    if(ispc)
        fPath = 'C:\\Documents and Settings\\suarezjl\\My Documents\\School\\Research\\AIST\\Results';
    else
        fPath = '/home/grxuser/Documents/School/Research/AIST/Results/';
        % QNX    % '\\home\\hrpuser\forceSensorPlugin_Pivot\Results'; 
    end

%% Load the data
    % Angle data, cartesian data, local force data, world force data, joint spring data, state data
    % For one arm, default right:
    if ~strcmp(StrategyType,'SIM_SA_DualArm')
        [angleData,cartData,localForceData,worldForceData,jointsnapData,stateData] = loadData(fPath,StratTypeFolder,FolderName);
    
    % For two arms, left and right.
    else
         [angleData, cartData, localForceData, worldForceData, ...
          angleDataL,cartDataL,localForceDataL,worldForceDataL,...
          jointsnapData,stateData] = loadData(fPath,StratTypeFolder,FolderName);
    end
    %----------------------------
    
    %----------------------------------------------------------------------
    % The following code plots the data in the Torques.dat file. If running
    % an online system, do not print.
    %----------------------------------------------------------------------
    if(DB_PLOT)    
        %% Plots
            if(DB_PLOT)
                % Different plots may have different durations. The duration in seconds
                % is hard-coded in this function for specific trials. 
                % A percentage is returned to use in scaling the plot axis. 
                [TIME_LIMIT_PERC, SIGNAL_THRESHOLD] = CustomizePlotLength(StrategyType,FolderName,localForceData);   
            else
                TIME_LIMIT_PERC = -1; SIGNAL_THRESHOLD = -1;
            end

        %% For PA10 include spring joint angles
        if(~strcmp(StrategyType,'HSA') && ~strcmp(StrategyType,'ErrorCharac'))

        %% Plot Rotation Spring Joint Position
            if(plotOptions)
                pSJ1=subplot(4,2,1); plot(jointsnapData(:,1),jointsnapData(:,2:3) );
            else
                figure(1); plot(jointsnapData(:,1),jointsnapData(:,2:3) );
                pSJ1 = gca;
            end
            title('Snap Joint Position'); xlabel('Time (secs)'); ylabel('Joint Angle'); 

            % Adjust Axes
            MARGIN = SWITCH;    
            AVERAGE = 0;
            if(DB_PLOT)
                [TOP_LIMIT_S1, BOTTOM_LIMIT_S1] = adjustAxes('Rotation Spring',jointsnapData,TIME_LIMIT_PERC,SIGNAL_THRESHOLD,MARGIN,AVERAGE);         
            else
                TOP_LIMIT_S1 = -1; BOTTOM_LIMIT_S1 = -1;
            end

        %%  Repeat
            if(plotOptions)
                pSJ2=subplot(4,2,2); plot(jointsnapData(:,1),jointsnapData(:,2:3) );
            else
                figure(2); plot(jointsnapData(:,1),jointsnapData(:,2:3) );
                pSJ2=gca;
            end
            title('Snap Joint Position'); xlabel('Time (secs)'); ylabel('Joint Angle'); 

            % Adjust Axes
            MARGIN = SWITCH;
            AVERAGE = 0;
            [TOP_LIMIT_S2, BOTTOM_LIMIT_S2] = adjustAxes('Rotation Spring',jointsnapData,TIME_LIMIT_PERC,SIGNAL_THRESHOLD,MARGIN,AVERAGE);  
        end

%% For HIRO no spring joint angles in the camera
%% Plot Fx
            if(plotOptions==1)
                pFx=subplot(3,2,1); plot(localForceData(:,1),localForceData(:,2));
            else
                pFx=plot(localForceData(:,1),localForceData(:,2));
            end
            title('Fx Plot'); xlabel('Time (secs)'); ylabel('Force (N)');

            % Adjust Axes
            MARGIN = SWITCH;
            AVERAGE = 0;	% uses the average value of data to compute the axis limits. useful for data with big impulses
            [TOP_LIMIT_Fx, BOTTOM_LIMIT_Fx] = adjustAxes('Fx',localForceData,StrategyType,TIME_LIMIT_PERC,SIGNAL_THRESHOLD,MARGIN,AVERAGE);    

%% Plot Fy
            if(plotOptions==1)
                pFy=subplot(3,2,3); plot(localForceData(:,1),localForceData(:,3));
            else
                pFy=plot(localForceData(:,1),localForceData(:,3));
            end
            title('Fy Plot'); xlabel('Time (secs)'); ylabel('Force (N)');

            % Adjust Axes
            MARGIN = SWITCH;
            AVERAGE = 0;        
            [TOP_LIMIT_Fy, BOTTOM_LIMIT_Fy] = adjustAxes('Fy',localForceData,StrategyType,TIME_LIMIT_PERC,SIGNAL_THRESHOLD,MARGIN,AVERAGE);    

%% Plot Fz
            if(plotOptions==1)    
                pFz=subplot(3,2,5); plot(localForceData(:,1),localForceData(:,4));
            else
                pFz=plot(localForceData(:,1),localForceData(:,4));
            end
            title('Fz Plot'); xlabel('Time (secs)'); ylabel('Force (N)');

            % Adjust Axes
            MARGIN = SWITCH;
            AVERAGE = 0;    
            [TOP_LIMIT_Fz, BOTTOM_LIMIT_Fz] = adjustAxes('Fz',localForceData,StrategyType,TIME_LIMIT_PERC,SIGNAL_THRESHOLD,MARGIN,AVERAGE);

%% Plot Mx
            if(plotOptions==1)    
                pMx=subplot(3,2,2); plot(localForceData(:,1),localForceData(:,5));
            else
                pMx=plot(localForceData(:,1),localForceData(:,5));
            end
            title('Mx Plot'); xlabel('Time (secs)'); ylabel('Moment (N-m)');

            % Adjust Axes
            MARGIN = SWITCH;     % If you want to insert a margin into the plots, set true.
            AVERAGE = 0;    % If you want to average the signal value
            [TOP_LIMIT_Mx, BOTTOM_LIMIT_Mx] = adjustAxes('Mx',localForceData,StrategyType,TIME_LIMIT_PERC,SIGNAL_THRESHOLD,MARGIN,AVERAGE);


%% Plot My
            if(plotOptions==1)    
                pMy=subplot(3,2,4); plot(localForceData(:,1),localForceData(:,6));
            else
                pMy=plot(localForceData(:,1),localForceData(:,6));
            end
            title('My Plot'); xlabel('Time (secs)'); ylabel('Moment (N-m)');

            % Adjust Axes
            MARGIN = SWITCH;
            AVERAGE = 0;    
            [TOP_LIMIT_My, BOTTOM_LIMIT_My] = adjustAxes('My',localForceData,StrategyType,TIME_LIMIT_PERC,SIGNAL_THRESHOLD,MARGIN,AVERAGE);

%% Plot Mz
            if(plotOptions==1)    
                pMz=subplot(3,2,6); plot(localForceData(:,1),localForceData(:,7));
            else
                pMz=plot(localForceData(:,1),localForceData(:,7));
            end
            title('Mz Plot'); xlabel('Time (secs)'); ylabel('Moment (N-m)');

            % Adjust Axes
            MARGIN = SWITCH;
            AVERAGE = 0;
            [TOP_LIMIT_Mz, BOTTOM_LIMIT_Mz] = adjustAxes('Mz',localForceData,StrategyType,TIME_LIMIT_PERC,SIGNAL_THRESHOLD,MARGIN,AVERAGE);

%% Insert State Lines
            handles         = [pFx pFy pFz pMx pMy pMz];

            % 8 axes for PA10
            if(~strcmp(StrategyType,'HSA') && ~strcmp(StrategyType,'ErrorCharac')) % Include limits for the rotational spring
                TOP_LIMIT       = [TOP_LIMIT_S1 TOP_LIMIT_S2 TOP_LIMIT_Fx TOP_LIMIT_Fy TOP_LIMIT_Fz TOP_LIMIT_Mx TOP_LIMIT_My TOP_LIMIT_Mz];
                BOTTOM_LIMIT    = [BOTTOM_LIMIT_S1 BOTTOM_LIMIT_S2 BOTTOM_LIMIT_Fx BOTTOM_LIMIT_Fy BOTTOM_LIMIT_Fz BOTTOM_LIMIT_Mx BOTTOM_LIMIT_My BOTTOM_LIMIT_Mz];
            % 6 axes for HIRO
            else
                TOP_LIMIT       = [TOP_LIMIT_Fx TOP_LIMIT_Fy TOP_LIMIT_Fz TOP_LIMIT_Mx TOP_LIMIT_My TOP_LIMIT_Mz];
                BOTTOM_LIMIT    = [BOTTOM_LIMIT_Fx BOTTOM_LIMIT_Fy BOTTOM_LIMIT_Fz BOTTOM_LIMIT_Mx BOTTOM_LIMIT_My BOTTOM_LIMIT_Mz];
            end
            % Call insertStates
            EndTime = localForceData(length(localForceData),1);   % Pass the last time element of task as endtime.
            insertStates3(StrategyType,stateData,EndTime,handles,TOP_LIMIT,BOTTOM_LIMIT);    

%% Save plot to file
            savePlot(fPath,StratTypeFolder,FolderName,handles(1),mfilename);
     end % END DB_PRINT. 
end     % End the function