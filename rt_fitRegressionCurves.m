%%************************** Documentation *********************************
% Analyze a single force or moment element curve for snap assembly, and,
% using a linear regression with corrleation thresholds, segement the data,
% into segmentes of linear plots. 
%
% Each entry or segment is called a "Primitive" and contains elements with
% the following information about its segment: [dAvg dMax dMin dStart dFinish dGradient dLabel].
%
% Online analysis: 
% This algorithm should be run in parallel and iteratively as the force
% data grows throughout the time of the task. 
%
% Assumes:
% (1) that the correlation of fitted data will be high until there is
% significant change in data. It is at that time, that we want to segment. 
% (2) only a single plot info is output. In fact, this function is
% typically called from snapVerification, which is running a for loop for
% each of the six plots Fx,Fy,Fz,Mx,My,Mz
%
% Input Parameters:
% fPath             : path string to the "Results" directory
% StrategyType      : refers to PA10-PivotApproach, or HIRO SideApproach "HSA"
% StratTypeFolder   : path string to Position/ForceControl: //StraightLineApproach or Pivot Approach or Side Approach
% Type              : type of data to analyze: Fx,Fy,Fz,Mx,My,Mz
% forceData         : Contains an nx1 vector of the type of force data
%                     indicated
% wStart            : the time, in milliseconds, at which this segment
%                     clock starts
% pHandle           : handle to the corresponding FxyzMxyz plot, to
%                     superimpose lines. 
% TL                : the top axes limits of each of the eight subplots SJ1,SJ2,
%                     Fx,Fy,Fz,Mx,My,Mz
% BL                : Same as TL but bottom limits. 
% Output Parameters:
% statData          : contains 1 index, and 7 statistics of each segmented line fit:
%                     average value of segmen, max val, min val, start
%                     time, end time, gradient value, gradient label string.
% rHandle           : handle to the segmented plot. 
% gradLabels        : very important structure. Used by
%                     GradientClassification.m and by primMatchEval.m
%                     Consists of all possible gradient classifications. 
%                     It is important to define here, to keep coherence
%                     with any changes that may take place and avoid
%                     braking other parts of the code
% index             : the axis that we are analyzing

% real-time: delete some parameters about plotting, pHandle, TL, BL.
% 2016.7.7
%**************************************************************************
function [statData,rHandle,gradLabels] = rt_fitRegressionCurves(Wren_loc,localIndex,fPath,StrategyType,StratTypeFolder,FolderName,Type,forceData,stateData,wStart,index)


%% Initialize variables
    % Globals
    global DB_PLOT;                                 % Declared in snapVerification. Enables plotting.
    global DB_WRITE;                                % Write to file.
    global DB_DEBUG;                                % Save statData.mat 
    
    global segmentIndex;                            % Used to count how many segmentations we have in our plot
    segmentIndex = 1;                               
    CORREL = 0; RSQ = 1;                            % 0 used for correlation coefficient, 1 used for R^2 
    WHATCOEFF = RSQ;                                % Select which coefficient you want to use for thresholding. Adjust threshold value accordingly
    
    %global write2FileFlag; 
    write2FileFlag = true;                          % Used to set a date on file
    
    % Allocate
    FileName            = '';
    statData            = zeros(100,7);             % Growing array that will hold segmented block statistical data (int types).

    % Size
    window_length       = 5;                        % Length of window used to analyze the data
    [rows c]            = size(forceData);          % size elements of force data
    
    % Thresholds
    if(~strcmp(StrategyType,'HSA') && ~strcmp(StrategyType,'ErrorCharac'))
        GoodFitThreshold    = 0.70;                 % Correlation coefficient USed to determine when to start a new data fit
    else
        GoodFitThreshold    = 0.90;                 % Correlation coefficient USed to determine when to start a new data fit        
    end
                                                % So far 60% seems to give good results, compared to 70-90
    % Bools
    iterFlag            = true;                 % Flag used to indicate when to exit while loop    
    
    
    % in real-time mode, we can't know the top and bottom bound of the
    % data, and they are just useful for drawing, so we remove TL and BL
    
%    domain = abs(TL(index))+abs(BL(index));


%%  Gradient Classification Structure 

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
    
               
%% Retrieve force-moment data index based on the type of data
    if    (strcmp(Type,'Fx'));       forceIndex = 2;
    elseif(strcmp(Type,'Fy'));       forceIndex = 3;
    elseif(strcmp(Type,'Fz'));       forceIndex = 4;
    elseif(strcmp(Type,'Mx'));       forceIndex = 5;
    elseif(strcmp(Type,'My'));       forceIndex = 6;
    elseif(strcmp(Type,'Mz'));       forceIndex = 7;    
    end  
    
    % Initialize start and finish
    wFinish = rows;    
    i=wStart;
    
%% Data Analysis
    % Two while loops: (a) run until all data has been analyzed; (b) run until
    % a segment has been finished
    while(i<rows+window_length) % For all force data points including a window (allow the last iteration to pass), which will be greater than rows to pass through
        
        if(i<rows) % All iterations except the last one
            
            % Reset iterFlag to true
            iterFlag = true;
            while(iterFlag)

                % a) Perform a polyfit for our growing window of data
                %    Establish length of time window and data range
                windowIndex = i + window_length;           % Index value when added by window_length
                Range       = wStart:windowIndex;          % The window of data that we are studying
                    % Check limits at the end
                    if(windowIndex>rows) 
                        Range = wStart:rows;
                        iterFlag = false;
                    end
                Time        = forceData(Range,1);          % Time indeces that we are working with
                Data        = forceData(Range,forceIndex); % Corresponding force data for a given force element in a given window

%%              % b) Fit data with a linear polynomial. Retrieve coefficients. 
                polyCoeffs  = polyfit(Time,Data,1);            % First-order fit

                
                
                
                Range = localIndex-windows:localIndex;
                Time = Wren_loc(Range,1);
                Data = Wren_loc(Range,index);
                
                dataFit = polyval(polyCoeffs, TIme);
                
                
                
                
                
                
                
                % c) Compute the values of 'y' (dataFit) for a fitted line.
                dataFit = polyval(polyCoeffs, Time);

%%              % d) Perform a correlation test
                    if(WHATCOEFF==CORREL)
                        % i)Correlation Coefficient
                        correlCoeff=corrcoef(Data,dataFit);

                        % If perfrect correlation, size is 1x1
                        if(size(correlCoeff)~=[1 1])
                           correlCoeff = correlCoeff(1,2);
                        end

                        % Check for NaN condition
                        if(isnan(correlCoeff)) 
                            correlCoeff = 1;    % Set to 1, to continue to analyze data
                        end

                        % Copy for test
                        coeffThshld = correlCoeff;

                    else
                        % ii) Determination Coefficient, R^2
                        yresid = Data - dataFit;                % Compute residuals
                        SSresid = sum(yresid.^2);               % Sum of squares of residuals
                        SStotal = (length(Data)-1) * var(Data); % Sum of squares of "y". Implmented by multiplying the variance of y by the number of observations minus 1:

                        %% Floating Point Checks
                            % Check if SSresid or SStotal are almost zero
                            if(SSresid<0.0001); SSresid = 0; end
                            if(SStotal<0.0001); SStotal = 0; end

                        % Compute rsq
                        rsq = 1 - SSresid/SStotal;              % Variance in yfit over variance in y. 

                            % Check for NaN condition
                            if(isnan(rsq));      rsq = 1;    % Set to 1, to continue to analyze data
                            elseif(isinf(rsq));  rsq = 1;       
                            end 

                        % Copy for test
                        coeffThshld = rsq;
                    end
%%              % e) Thresholding for correlation data

                % ei) If good correlation, keep growing window
                if(coeffThshld > GoodFitThreshold)
                    i= i+window_length;

%%              % e2) If false, save data window, plot, & perform statistics. 
                else         

                    % i) Adjust window parameters except for first iteration
                    if(~(windowIndex-window_length==1))             % If not the beginning
                        wFinish     = windowIndex-window_length;
                        Range       = wStart:wFinish;               % Save from wStart to the immediately preceeding index that passed the threshold
                        Time        = forceData(Range,1);           % Time indeces that we are working with
                        Data        = forceData(Range,forceIndex);  % Corresponding force data for a given force element in a given window
                        dataFit     = dataFit(1:length(Range),1);   % Data fit - window components

                    % First iteration. Keep index the same. 
                    else
                        wFinish     = windowIndex;
                        Range       = wStart:wFinish;               % Save from wStart to the immediately preceeding index that passed the threshold
                        Time        = forceData(Range,1);           % Time indeces that we are working with
                        Data        = forceData(Range,forceIndex);  % Corresponding force data for a given force element in a given windowdataFit     = dataFit(Range);               % Corresponding force data for a given force element in a given window                    
                        dataFit     = dataFit(Range);               % Corresponding force data for a given force element in a given window                                        
                    end
%%                  ii) Retrieve the segment's statistical Data and write to file
                    [dAvg dMax dMin dStart dFinish dGradient dLabel]=statisticalData(Time(1),   Time(length(Range)),...
                                                                                     dataFit,   domain,      polyCoeffs,...
                                                                                     FolderName,StrategyType,index); % 1+windowlength

                    % iii) Keep history of statistical data 
                    % All data types are numerical in this version. // Prior versions: Given that the datatypes are mixed, we must use cells. See {http://www.mathworks.com/help/techdoc/matlab_prog/br04bw6-98.html}       
                    statData(segmentIndex,:) = [dAvg dMax dMin dStart dFinish dGradient dLabel];

                    % iv) Write to file
                    if(DB_WRITE)
                        [FileName,write2FileFlag]=WritePrimitivesToFile(fPath,StratTypeFolder,FolderName,...
                                                          Type,FileName,write2FileFlag, ...
                                                          segmentIndex,dAvg,dMax,dMin,dStart,dFinish,dGradient,dLabel);
                    end
%%                  % v) Plot data
                    if(DB_PLOT)
                        rHandle=plotRegressionFit(Time,dataFit,Type,pHandle,TL,BL,FolderName,forceData,stateData);
                    else
                        rHandle=0;
                    end

%%                  % Wrap Up 
                    % vi) Increase counter
                    segmentIndex = segmentIndex + 1;
                    i = i+window_length;

                    % vii) Reset the window start and the window finish markers
                    wStart = wFinish;       % Start with the last "out-of-threshold" window

                    % viii) Set iterflag to false. Exit inner loop and restart
                    % outside loop
                    iterflag = false;
                    wFinish  = rows;
                end % End coefficient threshold
            end     % End while(iterFlag)
        
%%      WRAP-UP: Last iteration        
        % This is the last iteration, wrap up. 
        else
            % Set the final variables
            wFinish     = rows;                            % Set to the last index of statData (the primitives space)
            Range       = wStart:wFinish;               % Save from wStart to the immediately preceeding index that passed the threshold
            Time        = forceData(Range,1);           % Time indeces that we are working with
            Data        = forceData(Range,forceIndex);  % Corresponding force data for a given force element in a given window
            dataFit     = dataFit(1:length(Range),1);   % Data fit - window components

%%          ii) Retrieve the segment's statistical Data and write to file
            [dAvg,dMax,dMin,dStart,dFinish,dGradient,dLabel]=statisticalData(Time(1),Time(length(Time)),Data,domain,polyCoeffs,FolderName,StrategyType,index); % 1+windowlength

            % iii) Keep history of statistical data 
            % All data types are numerical in this version. // Prior
            % versions: Given that the datatypes are mixed, we must use cells. See {http://www.mathworks.com/help/techdoc/matlab_prog/br04bw6-98.html}       
            statData(segmentIndex,:) = [dAvg dMax dMin dStart dFinish dGradient dLabel];
                        
%%          CleanUp the statistical data
            % For contiguous pairs of primitives, if one is 5 times longer
            % than the other, absorb it.
            statData = primitivesCleanUp(statData,gradLabels);

            % iv) Write to file
            if(DB_WRITE)
                [FileName,write2FileFlag]=WritePrimitivesToFile(fPath,StratTypeFolder,FolderName,...
                                                                Type,FileName,write2FileFlag, ...
                                                                segmentIndex,dAvg,dMax,dMin,dStart,dFinish,dGradient,dLabel);
            end

            % Get out of the while loop
            break;
        end         % End i<rows
    end             % End while(i<rows+window_length)
    
    
%% Resize statData in case not all of its rows were occupied
    statData = resizeData(statData);  

%% Save statData.mat to file
    if(DB_DEBUG)
        save(strcat(fPath,StratTypeFolder,FolderName,'/statData.mat'),'statData','-mat')        
    end
    
end     % End function