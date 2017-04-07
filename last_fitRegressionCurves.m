function [numNew,data_new,wStart,marker,uptillnow_p] = last_fitRegressionCurves(Wren_loc,localIndex,StrategyType,FolderName,wStart,marker,index)
%% Last iteration of primitive layer

%% Initialize variables
    numNew = 0;
    data_new = zeros(1,7);
    
    forceIndex = index+1;
    
    CORREL = 0; RSQ = 1;                            % 0 used for correlation coefficient, 1 used for R^2 
    WHATCOEFF = RSQ;                                % Select which coefficient you want to use for thresholding. Adjust threshold value accordingly
    
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
    last_wFinish = marker;
    
    windowIndex = localIndex;
    Range = wStart:windowIndex;
    Time  = Wren_loc(Range,1);          % Time indeces that we are working with
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
        if(isnan(rsq));      rsq = 1;           % Set to 1, to continue to analyze data
        elseif(isinf(rsq));  rsq = 1;
        end
        
        % Copy for test
        coeffThshld = rsq;
    end
    
    %%        % e) Thresholding for correlation data
    
    % In last iteration , if good correlation, will return just 1 item.
    if(coeffThshld > GoodFitThreshold)
        [dAvg,dMax,dMin,dStart,dFinish,dGradient,dLabel] = rt_statisticalData(Time(1),   Time(length(Range)),...
            dataFit,      polyCoeffs,...
            FolderName,StrategyType,index); % 1+windowlength
        numNew = numNew+1;
        data_new(numNew,1:7) = [dAvg,dMax,dMin,dStart,dFinish,dGradient,dLabel] ;
        
        % vii) Reset the window start and the window finish markers
        wFinish= windowIndex;
        wStart = wFinish;       % Start with the last "out-of-threshold" window
        marker = windowIndex;
        
        fprintf('----good,only one\n');
        fprintf('%f %f %f %f %f %f %f\n',data_new(numNew,1),data_new(numNew,2),data_new(numNew,3),data_new(numNew,4),data_new(numNew,5),data_new(numNew,6),data_new(numNew,7));
    % In last iteration , if bad correlation, will return 1 or 2 items.
    else
        
        % i) Adjust window parameters except for first iteration
        if(~(windowIndex-window_length==1))             % If not the beginning
            wFinish     = last_wFinish;
            Range       = wStart:wFinish;               % Save from wStart to the immediately preceeding index that passed the threshold
            Time        = Wren_loc(Range,1);    % Time indeces that we are working with
            Data        = Wren_loc(Range,forceIndex);  % Corresponding force data for a given force element in a given window
            
            % !!! If windowIndex-window_length == wStart,
            % we will do polyfit on a single point, not serious
            % points. It's Wrong. But we will throw away
            % this kind of result in Motion Composition
            % layer
            polyCoeffs  = polyfit(Time,Data,1);            % First-order fit
            
            % c) Compute the values of 'y' (dataFit) for a fitted line.
            dataFit = polyval(polyCoeffs, Time);
            
            %%                  ii) Retrieve the segment's statistical Data and write to file
            [dAvg,dMax,dMin,dStart,dFinish,dGradient,dLabel] = rt_statisticalData(Time(1),   Time(length(Range)),...
                dataFit,      polyCoeffs,...
                FolderName,StrategyType,index); % 1+windowlength
            numNew = numNew+1;
            data_new(numNew,1:7) = [dAvg,dMax,dMin,dStart,dFinish,dGradient,dLabel] ;
            
            % vii) Reset the window start and the window finish markers
            wStart = wFinish;       % Start with the last "out-of-threshold" window
            marker = wFinish;
            
            fprintf('----middle,one\n');
            fprintf('%f %f %f %f %f %f %f\n',data_new(numNew,1),data_new(numNew,2),data_new(numNew,3),data_new(numNew,4),data_new(numNew,5),data_new(numNew,6),data_new(numNew,7));
            
            wFinish     = windowIndex;
            Range       = wStart:wFinish;               % Save from wStart to the immediately preceeding index that passed the threshold
            Time        = Wren_loc(Range,1);    % Time indeces that we are working with
            Data        = Wren_loc(Range,forceIndex);  % Corresponding force data for a given force element in a given window
            
            % !!! If windowIndex-window_length == wStart,
            % we will do polyfit on a single point, not serious
            % points. It's Wrong. But we will throw away
            % this kind of result in Motion Composition
            % layer
            polyCoeffs  = polyfit(Time,Data,1);            % First-order fit
            
            % c) Compute the values of 'y' (dataFit) for a fitted line.
            dataFit = polyval(polyCoeffs, Time);
            
            %%                  ii) Retrieve the segment's statistical Data and write to file
            [dAvg,dMax,dMin,dStart,dFinish,dGradient,dLabel] = rt_statisticalData(Time(1),   Time(length(Range)),...
                dataFit,      polyCoeffs,...
                FolderName,StrategyType,index); % 1+windowlength
            numNew = numNew+1;
            data_new(numNew,1:7) = [dAvg,dMax,dMin,dStart,dFinish,dGradient,dLabel] ;
            
            % vii) Reset the window start and the window finish markers
            wStart = wFinish;       % Start with the last "out-of-threshold" window
            marker = wFinish;
            
            fprintf('----middle,two\n');
            fprintf('%f %f %f %f %f %f %f\n',data_new(numNew,1),data_new(numNew,2),data_new(numNew,3),data_new(numNew,4),data_new(numNew,5),data_new(numNew,6),data_new(numNew,7));
            
            
            % First iteration. Keep index the same.
        else
            wFinish     = windowIndex;
            Range       = wStart:wFinish;               % Save from wStart to the immediately preceeding index that passed the threshold
            Time        = Wren_loc(Range,1);   % Time indeces that we are working with
            Data        = Wren_loc(Range,forceIndex);  % Corresponding force data for a given force element in a given windowdataFit     = dataFit(Range);               % Corresponding force data for a given force element in a given window
            dataFit     = dataFit(Range);               % Corresponding force data for a given force element in a given window
            
            
            %%                  ii) Retrieve the segment's statistical Data and write to file
            [dAvg,dMax,dMin,dStart,dFinish,dGradient,dLabel] = rt_statisticalData(Time(1),   Time(length(Range)),...
                dataFit,      polyCoeffs,...
                FolderName,StrategyType,index); % 1+windowlength
            numNew = numNew+1;
            data_new(numNew,1:7) = [dAvg,dMax,dMin,dStart,dFinish,dGradient,dLabel] ;
            
            % vii) Reset the window start and the window finish markers
            wStart = wFinish;       % Start with the last "out-of-threshold" window
            marker = wFinish;
            
            fprintf('----first,only one\n');
            fprintf('%f %f %f %f %f %f %f\n',data_new(numNew,1),data_new(numNew,2),data_new(numNew,3),data_new(numNew,4),data_new(numNew,5),data_new(numNew,6),data_new(numNew,7));
        end
        
    end % End coefficient threshold
   
    uptillnow_p = false;
    
end
            