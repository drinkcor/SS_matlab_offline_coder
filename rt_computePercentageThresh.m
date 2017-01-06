%% ********************************* Documentation ************************
% Function that computes the difference of the "average value" of a
% motion composition structure between index i and i+1, and compares it to
% see if it is less than a trehshold defined by AmplitudeRatio.
%
% The threshold defined up in the stack, is defined to be typically 10% of the
% value.
%
% The greatest average value is used to compute the percentage between
% contiguous elements. 
%
% Returns boolean result. 
% data:         - data to be analyzed
% dataIndex:    - selects what row in the data will be studies
% strucIndex:       - selects what element of the data to compare
% AmplitudeRation:  - sets the threshold
%**************************************************************************
function [boolPerc,biggerOne] = rt_computePercentageThresh(data,dataIndex,strucIndex,AmplitudeRatio)

%% Initialization
    [r,c]   = size(data);
    if(c==11)        
        % mot Comps Structure Indeces
        ACTN_LBL         = 1;   % action class
        AVG_MAG_VAL      = 2;   % average value
        RMS_VAL          = 3;   % rms value, which is max value too
        AMPLITUDE_VAL    = 4;   % amplitude value
        
        % Labels
        P1LBL = 5; 
        P2LBL = 6;   % label indeces for both primitives
        
        % Time Indeces
        T1S         = 7; 
        T1E         = 8;
        T2S         = 9; 
        T2E         = 10;
        TAVG_INDEX  = 11;
        
    elseif(c==17)
        % llbehStruc Indeces
        behLbl          = 1;   % action class
        averageVal1     = 2;   % averageVal1
        averageVal2     = 3;
        AVG_MAG_VAL     = 4;
        rmsVal1         = 5;
        rmsVal2         = 6;
        AVG_RMS_VAL     = 7;
        ampVal1         = 8;
        ampVal2         = 9;
        AMPLITUDE_VAL   = 10;  % AVG_AMP_VAL
        mc1             = 11;
        mc2             = 12;
        T1S             = 13;
        T1E             = 14;
        T2S             = 15;
        T2E             = 16;
        TAVG_INDEX      = 17;

    end
    % Time duration, just for comparing time duration, not actully exist
    TIME_DURATION = 0;
        
    % match is next contiguous element
    match = dataIndex + 1;
    
    % if strucIndex==0, means we should compare time duration
    if(strucIndex==TIME_DURATION)
        abs_Index = data(dataIndex,T2E) - data(dataIndex,T1S);
        abs_Match = data(match,    T2E) - data(match,    T1S);
    else
        abs_Index = abs(data(dataIndex,strucIndex));
        abs_Match = abs(data(match,    strucIndex));
    end
    
%%  Compute the percentage value that we will compare against
    percThresh = AmplitudeRatio*max([abs_Index,abs_Match]);
    
%%  If difference is smaller, return a true value, else false.
    if((strucIndex==AMPLITUDE_VAL)||(strucIndex==TIME_DURATION)) % Take the absolute value of each number
        c = abs(abs_Index-abs_Match);
        if(c <= percThresh)
            boolPerc = true;
        else
            boolPerc = false;
        end

    % Take the absolute value of the difference
    elseif(strucIndex==AVG_MAG_VAL) 
        if(abs(data(dataIndex,strucIndex) - data(match,strucIndex)) <= percThresh)
            boolPerc = true;
        else
            boolPerc = false;
        end        
    end
    
    % Determine which one is bigger
    if(abs_Index>abs_Match)
        biggerOne = -1;
    elseif(abs_Index<abs_Match)
        biggerOne = 1;
    else
        biggerOne = 0;
    end

end