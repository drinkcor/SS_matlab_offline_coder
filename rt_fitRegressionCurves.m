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
function [statData,segmentIndex,dataFit_pre,wStart,marker] = rt_fitRegressionCurves(Wren_loc,localIndex,statData,segmentIndex,fPath,StrategyType,StratTypeFolder,FolderName,Type,dataFit_pre,wStart,marker,rate,index)


%% Initialize variables
    % Globals
    global DB_WRITE;                                % Write to file.
    global DB_DEBUG;                                % Save statData.mat 
                                 
    CORREL = 0; RSQ = 1;                            % 0 used for correlation coefficient, 1 used for R^2 
    WHATCOEFF = RSQ;                                % Select which coefficient you want to use for thresholding. Adjust threshold value accordingly
    
    %global write2FileFlag; 
    write2FileFlag = true;                          % Used to set a date on file
    
    % Allocate
    FileName            = '';

    % Size
    window_length       = 5;                        % Length of window used to analyze the data
     
    % Thresholds
    if(~strcmp(StrategyType,'HSA') && ~strcmp(StrategyType,'ErrorCharac'))
        GoodFitThreshold    = 0.70;                 % Correlation coefficient USed to determine when to start a new data fit
    else
        GoodFitThreshold    = 0.90;                 % Correlation coefficient USed to determine when to start a new data fit        
    end
                                                % So far 60% seems to give good results, compared to 70-90

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
    
    % the process won't be execute unless there is enough data for fitting.
    if (marker+window_length <= localIndex)
        
        windowIndex = marker+window_length;
        Range = wStart:windowIndex;
        Time  = (1/rate) * Wren_loc(Range,1);          % Time indeces that we are working with
        Data  = Wren_loc(Range,forceIndex); % Corresponding force data for a given force element in a given window

%%      % b) Fit data with a linear polynomial. Retrieve coefficients. 
        polyCoeffs  = polyfit(Time,Data,1);            % First-order fit
    
        % c) Compute the values of 'y' (dataFit) for a fitted line.
        dataFit = polyval(polyCoeffs, Time);
        
%%      % d) Perform a correlation test
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

%%        % e) Thresholding for correlation data

          % ei) If good correlation, keep growing window
          if(coeffThshld > GoodFitThreshold)
                marker= marker+ window_length;
                dataFit_pre = dataFit;

%%        % e2) If false, save data window, plot, & perform statistics. 
          else         

                % i) Adjust window parameters except for first iteration
                   if(~(windowIndex-window_length==1))             % If not the beginning
                        wFinish     = windowIndex-window_length;
                        Range       = wStart:wFinish;               % Save from wStart to the immediately preceeding index that passed the threshold
                        Time        = forceData(Range,1);           % Time indeces that we are working with
                        Data        = forceData(Range,forceIndex);  % Corresponding force data for a given force element in a given window
                        % if there isn't existing previous good fitting results, use current bad result.
                        if (isnan(dataFit_pre))
                            dataFit     = dataFit(1:length(Range),1);   % Data fit - window components
                        % if there is existing previous good fitting results, use good result.
                        else
                            dataFit     = dataFit_pre(1:length(Range),1);   % Data fit - window components
                            dataFit_pre = nan;
                        end

                    % First iteration. Keep index the same. 
                    else
                        wFinish     = windowIndex;
                        Range       = wStart:wFinish;               % Save from wStart to the immediately preceeding index that passed the threshold
                        Time        = forceData(Range,1);           % Time indeces that we are working with
                        Data        = forceData(Range,forceIndex);  % Corresponding force data for a given force element in a given windowdataFit     = dataFit(Range);               % Corresponding force data for a given force element in a given window                    
                        dataFit     = dataFit(Range);               % Corresponding force data for a given force element in a given window
                        dataFit_pre = nan;
                    end
%%                  ii) Retrieve the segment's statistical Data and write to file
                    [dAvg dMax dMin dStart dFinish dGradient dLabel]=rt_statisticalData(Time(1),   Time(length(Range)),...
                                                                                     dataFit,      polyCoeffs,...
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

%%                  % Wrap Up 
                    % vi) Increase counter
                    segmentIndex = segmentIndex + 1;
                    
                    % vii) Reset the window start and the window finish markers
                    wStart = wFinish;       % Start with the last "out-of-threshold" window
                    marker = marker+window_length;
            end % End coefficient threshold
        
    end
    
    
    
    