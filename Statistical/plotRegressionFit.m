%**************************** Documentation *******************************
% Plots the linear regression fit line. 
% Superimposes the plot on an already created plot for either Fx,Fy,Fz,Mx,My,Mz. 
%
% If a handle is given, it plots only data for that handle. The function
% that calls plotRegressionFit is running a for loop for each of the force
% types. 
% 
% If no handle is given, a new figure is opened, and the plot takes place
% there.
%
% Input parameters:
% x:                - the independent variable (i.e. time)
% yfit:             - fitted data (dependent variable)
% Type:             - string defining type of curve (FxFyFzMxMyMz)
% pHandle:          - handles used to plot the original plots for FxyzMxyz
% TL:               - the top axes limits of each of the eight subplots
%                     Fx,Fy,Fz,Mx,My,Mz,SJ1,SJ2
% BL:               - Same as TL but bottom limits. 
% FolderName:       - Used to set the ending parameter for the result
% Data:             - plot data used to determine y_max and y_min limits
% stateData:        - column vector containing when a new state starts
%
% Output Parameters:
% rHandle:          - handle to new graph
%**************************************************************************
function [rHandle]=plotRegressionFit(primiData,primiIndex,Type,pHandle)



%% Preprocessing and Plotting
    
    % Choose line color
    lineColor = 'r';
    lineWidth = 1.0; % Normally 1.5
    
    % If no handle is given, open a new figure
    if(pHandle==0)
        figure(2); grid on;
    
        % Plot Data.
        %% TODO: use drawnow limitrate to have dynamic plotting
        rHandle=plot(0,0);%(x,yfit,lineColor,'linewidth',lineWidth); 
        
%% Insert State Lines into Diagram
        % Compute the real end of the signal        
%         [TIME_LIMIT_PERC, SIGNAL_THRESHOLD] = CustomizePlotLength(FolderName,Data);                  
% 
%         % Adjust Axes
%         MARGIN = 0;         % Do you want a margin of white space surrounding the max,min values of the curve
%         AVERAGE = 0;        % Do you want to set axis data around average value of curve
%         [TOP_LIMIT, BOTTOM_LIMIT] = adjustAxes(Type,Data,TIME_LIMIT_PERC,SIGNAL_THRESHOLD,MARGIN,AVERAGE);
% 
%         % Insert the states
%         FillFlag = 1; % Fill with color
%         axes(gca);      % Activate the appropriate subplot/axis handle
%         insertStates(stateData,TOP_LIMIT,BOTTOM_LIMIT,FillFlag);                  
%%                
    else
        for i = 1:primiIndex
        % Set the handle of the corresponding already plotted force data.
            axes(pHandle); hold on; % When using parfor this throws an error saying that it is an invalid object handle
            x       = [primiData(i,4),primiData(i,5)];
            yfit    = [primiData(i,2),primiData(i,3)];
            rHandle=plot(x,yfit,lineColor,'linewidth',1.5);
            %rHandle=plot(0,0);  % Do this to keep the handle alive.                          
        end
        
        % Labels
        xlabel('Time (secs)');
        if(strcmp(Type,'Fx') || strcmp(Type,'Fy') || strcmp(Type,'Fz'))
            ylabel('Force (N)');
        elseif(strcmp(Type,'Mx') || strcmp(Type,'My') || strcmp(Type,'Mz'))
            ylabel('Moment (N-m)');
        end
        % Title
        title(['Regression Fits for',' ',Type]);
            
    end
end % End function