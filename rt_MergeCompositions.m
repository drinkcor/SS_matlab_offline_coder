%% **************************** Documentation *****************************
% Merges data between two continguous elements in a data composition data
% structure. 
%
% Code updated in 2013Aug. Currently this only merges two contiguous
% compositions. We want to update it such that it can merge multiple
% compositions. To keep compatibility, we just look at the variable
% indicator as an indicator of how many compositions to merge. 
% If this variable has value 0 or 1, keep the same code, otherwise merge as
% many compositions as indicated.
% 
% The data structure is a cell array composed of 11 elements: 
% motComps:(nameLabel,avgVal,rmsVal,amplitudeVal,p1lbl,p2lbl,t1Start,t1End,t2Start,t2End,tAvgIndex)
% 
% Input Parameters:
% index:            - first element of contiguous pair.
% data:             - an mx11 cell array data structure containing action compositions
% actionLbl         - cell array structure that holds strings for action compositions
% actionLblIndex    - index to select what label to use, i.e.: adjust, clean, etc.
% indicator         - if value = -1, indicates we should merge the 1st composition onto the second.
%                     if value = 0, indicates we should merge the 2nd composition onto the first.
%                     if value = 1, indicates the 1st and 2nd composition are equally important.
%                     if value = n, indicates we should merge the repeated compositions.
%**************************************************************************
function data_new = rt_MergeCompositions(index,data,actionLbl,actionLblIndex,indicator)

%%  Initialization

    % Define next contiguous element
    match = index+1;

    % mot Comps Structure Indeces
    ACTN_LBL         = 1;   % action class
    AVG_MAG_VAL      = 2;   % average value
    RMS_VAL          = 3;   % rms value
    AMPLITUDE_VAL    = 4;   % amplitude value 
    
    % Labels indeces for both primitives
    P1LBL = 5; 
    P2LBL = 6;    
    
    % Time Indeces
    T1S = 7; T1E = 8;
    T2S = 9; T2E = 10;    
    TAVG_INDEX   = 11;
    
    data_new    = data(index,:);
    
    
    data_new(ACTN_LBL) = actionLbl(actionLblIndex); % Set the name of the ACTN_LBL to the one passed by in actionLblIndex. Could be 'a' or 'c', etc.
    
    % Fake label(after clean up, we can't 100% retain the original sub labels, we can create some fake label to make up, but it's a waste of time)
    % Needn't keep track of P1LBL and P2LBL
    
    
    if(indicator<2)
    %%  Values                        
        % Average average magnitude value: use their time duration as weight
        Tindex                      = data(index,T2E)-data(index,T1S);
        Tmatch                      = data(match,T2E)-data(match,T1S);
        Twhole                      = data(match,T2E)-data(index,T1S);
        
        data_new(AVG_MAG_VAL)       = ( data(index,AVG_MAG_VAL) * Tindex + data(match,AVG_MAG_VAL) * Tmatch ) / Twhole;

        % MaxVal 2013Sept replaced: 
        data_new(RMS_VAL)           = max( data(index,RMS_VAL),data(match,RMS_VAL) ); 
        % RMS value: (index+match)/2
        % data(index,RMS_VAL)       = ( data(index,RMS_VAL)       + data(match,RMS_VAL) )/2; 

        index_min                   = data(index,RMS_VAL)-data(index,AMPLITUDE_VAL);
        match_min                   = data(match,RMS_VAL)-data(match,AMPLITUDE_VAL);
        
        data_new(AMPLITUDE_VAL)     = data_new(RMS_VAL) - min( index_min,match_min );

        % TAVG_INDEX
        data_new(TAVG_INDEX)        = ( data(index,T1S) + data(match,T2E) )/2 ;   
        data_new(T2E)               = data(match,T2E);
        
        % After clean up like repeated clean up, we can't retain the
        % original time separater 100%. Thus we use mid-time to make up.
        % Needn't keep track of P1LBL and P2LBL
        data_new(T1E)               = data_new(TAVG_INDEX);
        data_new(T2S)               = data_new(TAVG_INDEX);
        
        
    %% Multipe compositions
    else
    %%  Name Label: Do nothing, keep the label of the orig composition

    %%  Values                                                        
        % Average average magnitude value: use their time duration as weight
        sum_mag                 = 0;
        for  i = index:index+indicator
            sum_mag             = sum_mag + data(i,AVG_MAG_VAL) * ( data(i,T2E) - data(i,T1S) ); 
        end
        Twhole                  = data(index+indicator,T2E) - data(index,T1S);
        data_new(AVG_MAG_VAL)   = sum_mag / Twhole; 
        
        % MaxValue 2013Sept now replaces RMS
        data_new(RMS_VAL)       = max( data(index:index+indicator,RMS_VAL) ); 
        
        % RMS value: (index+match)/2
        %data(index,RMS_VAL)    = mean( data(index:index+indicator,RMS_VAL)    ); 

        % Amplitude value: take the maximum amplitude. This is a difficult number to compute as the data currently stands... Need max and min points.
        % Take the max value. These points are adjacent. The AMPLITUDE would not decrease, could only increase.T
        min_dataNew             = min( data(index:index+indicator,RMS_VAL)-data(index:index+indicator,AMPLITUDE_VAL) );
        data_new(AMPLITUDE_VAL) = data_new(RMS_VAL)-min_dataNew; 

    %%  Time
    %  Note!!! This merging requires us to skip some time boundaries of
    %  merged compositions. The best we can do is keep the boundaries. When
    %  one looks at the Composition file, times will not match for every corresponding 
    %  segment/composition. That is because we had to skip them here. It is not wrong,
    % it's just something we have to deal with. 
    
        % T2_END,index = T2_END,match
        data_new(T2E)        = data(index+indicator,T2E);       
   

        % TAVG_INDEX: (T1S+T2E)/2
        data_new(TAVG_INDEX) = ( data(index,T1S) + data(index+indicator,T2E) )/2;    
        
        data_new(T2S)        = data_new(TAVG_INDEX);

        data_new(T1E)        = data_new(TAVG_INDEX);
    end
end