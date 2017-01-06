%% **************************** Documentation *****************************
% Merges data between two continguous elements in a data composition data
% structure unto the first llb.
% 
% The llb data structure is an int array composed of 17 elements: 
% data:(nameLabel,avgVal1,avgVal2,Avg_avgVal,rmsVal1,rmsVal2,Avg_rmsVal,amplitudeVal1,amplitudeVal2,Avg_amplitudeVal,p1lbl,p2lbl,t1Start,t1End,t2Start,t2End,tAvgIndex)
% 
% Input Parameters:
% index:        - first element of contiguous pair.
% data:         - an mx11 cell array data structure.
% llbehLbl      - cell array structure that holds strings for action compositions
% llbehLblIndex - index to select what label to use.
% indicator     - Used to indicate how to merge
%**************************************************************************
function data_new = rt_MergeLowLevelBehaviors(index,data,llbehLbl,llbehLblIndex,indicator)

%%  Initialization

    % Define next contiguous element
    match = index+1;

    % CONSTANTS FOR gradLabels (defined in fitRegressionCurves.m)
% 	FIX     = 1;        % Fixed in place
%   CONTACT = 2;        % Contact
%   PUSH    = 3;        % Push
%   PULL    = 4;        % Pull
%   ALIGN   = 5;        % Alignment
%   SHIFT   = 6;        % Shift
%   UNSTABLE= 7;        % Unstable
%   NOISE   = 8;        % Noise
%  llbehLbl    = ('FX' 'CT' 'PS' 'PL' 'AL' 'SH' 'U' 'N'); % ('fix' 'cont' 'push' 'pull' 'align' 'shift' 'unstable' 'noise');

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
    AVG_AMP_VAL     = 10;
    mc1             = 11;
    mc2             = 12;    
    T1S             = 13; 
    T1E             = 14;
    T2S             = 15; 
    T2E             = 16;    
    TAVG_INDEX      = 17;    
    
        
    data_new        = data(index,:);
    
    data_new(behLbl) = llbehLbl(llbehLblIndex); % Set the name of the BEH_LBL to the one passed by in llbehLblIndex. Could be 'Fixed' or 'PUSH', etc.
    
    
    if(indicator<2)
        
        p1time      = data(index,T2E) - data(index,T1S);
        p2time      = data(match,T2E) - data(match,T1S);
        whole_time  = data(match,T2E) - data(index,T1S);
        
        %% Values
        %%  Average Values
        data_new(averageVal1) = data(index,AVG_MAG_VAL);
        data_new(averageVal2) = data(match,AVG_MAG_VAL);
        data_new(AVG_MAG_VAL) = ( p1time*data(index,AVG_MAG_VAL) + p2time*data(match,AVG_MAG_VAL) )/whole_time ;
        
        %% Max Values. 2013Sept replaced RMS.
        data_new(rmsVal1)     = data(index,AVG_RMS_VAL);
        data_new(rmsVal2)     = data(match,AVG_RMS_VAL);
        data_new(AVG_RMS_VAL) = max(data(index,AVG_RMS_VAL),data(match,AVG_RMS_VAL));
        
        %         %% RMS Values
        %         data(index,rmsVal2)     = data(match,rmsVal2);
        %         data(index,AVG_RMS_VAL) = ( data(index,AVG_RMS_VAL) + data(match,AVG_RMS_VAL) )/2;
        
        %% Amplitude value: Take the max value. These points are adjacent. The AMPLITUDE would not decrease, could only increase.T
        data_new(ampVal1)     = data(index,AVG_AMP_VAL);
        data_new(ampVal2)     = data(match,AVG_AMP_VAL);
        
        index_min             = data(index,AVG_RMS_VAL)-data(index,AVG_AMP_VAL);
        match_min             = data(match,AVG_RMS_VAL)-data(match,AVG_AMP_VAL);
        
        data_new(AVG_AMP_VAL) = data_new(AVG_RMS_VAL)-min( index_min,match_min );
        
        %%  LABELS
        % In this case assign low-level behavior labels for the
        % motion-composition labels. Although labels of low-level and
        % of motion compositions are inconsistent, it won't affact the subsequent
        % processing.
        data_new(mc1) = data(index,behLbl);
        data_new(mc2) = data(match,behLbl);
        
        %%  Time
        %      (index)             (match)
        %       T1S    T1E          T2S    T2E
        %   t1S,t1E  t2s,t2E    t1S,t1E  t2s,t2E
        % T1_END,index = T2_END,index
        data_new(T1E) = data(index,T2E);
        
        % T2_START,index = T1_START,match
        data_new(T2S) = data(match,T1S);
        
        % T2_END,index = T2_END,match
        data_new(T2E) = data(match,T2E);
        
        % Check limits - for strange cases
        
        %% index and match are in reverse order
        % if t2e,match = t1s,index then set (t1s,match) as the beginning.
        % The end won't need to be changed because  t2e,index is already
        % pointing to the end
        if(data(match,T2E) == data(index,T1S))
            data_new(T1S)=data(match,T1S);
        end
        
        % If the ending of T1E is greater than T2E
        if(data(index,T1E)>data(index,T2E))
            data_new(T2E)=data(index,T1E);
        end
        
        % If t1S > t2S in the same one
        if(data(index,T2S)<data(index,T1S))
            data_new(T1S)=data(index,T2S);
        end
        
        % If the end of index is less than the start of match
        if(data(index,T2E)<data(match,T1S))
            data_new(T1S)=data(index,T2S);
        end
        
        % TAVG_INDEX
        data_new(TAVG_INDEX) = ( data(index,T1S)+data(match,T2E) )/2 ;

    else % Which means dealing with repeated 
    
        %% Values
        %%  Average Values
        sum_mag                 = sum( data(index:index+indicator,AVG_MAG_VAL) .* (data(index:index+indicator,T2E) - data(index:index+indicator,T1S)) );       
        Twhole                  = data(index+indicator,T2E) - data(index,T1S);
        data_new(AVG_MAG_VAL)   = sum_mag / Twhole; 
        
        data_new(averageVal1) = data_new(AVG_MAG_VAL);
        data_new(averageVal2) = data_new(AVG_MAG_VAL);
        
        
        %% Max Values. 2013Sept replaced RMS.
        data_new(AVG_RMS_VAL) = max(data(index:index+indicator,AVG_RMS_VAL));
        data_new(rmsVal1)     = data_new(AVG_RMS_VAL);
        data_new(rmsVal2)     = data_new(AVG_RMS_VAL);

        
        %% Amplitude value: Take the max value. These points are adjacent. The AMPLITUDE would not decrease, could only increase.T  
        data_new(AVG_AMP_VAL) = data_new(AVG_RMS_VAL) - min( data(index:index+indicator,AVG_RMS_VAL)-data(index:index+indicator,AVG_AMP_VAL) );
        data_new(ampVal1)     = data_new(AVG_AMP_VAL);
        data_new(ampVal2)     = data_new(AVG_AMP_VAL);
        
        
        %%  LABELS
        % In this case assign low-level behavior labels for the
        % motion-composition labels. Although labels of low-level and
        % of motion compositions are inconsistent, it won't affact the subsequent
        % processing.
        data_new(mc1) = llbehLbl(llbehLblIndex);
        data_new(mc2) = llbehLbl(llbehLblIndex);
        
        %%  Time
        %      (index)             (match)
        %       T1S    T1E          T2S    T2E
        %   t1S,t1E  t2s,t2E    t1S,t1E  t2s,t2E
        % T1_END,index = T2_END,index
        
        % TAVG_INDEX
        data_new(TAVG_INDEX) = ( data(index,T1S)+data(index+indicator,T2E) )/2 ;
        
        data_new(T1E) = data_new(TAVG_INDEX);
        
        % T2_START,index = T1_START,match
        data_new(T2S) = data_new(TAVG_INDEX);
        
        % T2_END,index = T2_END,match
        data_new(T2E) = data(index+indicator,T2E);
        
        % Check limits - for strange cases
        
        %% index and match are in reverse order
        % if t2e,match = t1s,index then set (t1s,match) as the beginning.
        % The end won't need to be changed because  t2e,index is already
        % pointing to the end
%         if(data(index+indicator,T2E) == data(index,T1S))
%             data_new(T1S)=data(index+indicator,T1S);
%         end
%         
%         % If the ending of T1E is greater than T2E
%         if(data_new(T1E)>data_new(T2E))
%             data_new(T2E)=data_new(T1E);
%         end
%         
%         % If t1S > t2S in the same one
%         if(data_new(T2S)<data_new(T1S))
%             data_new(T1S)=data_new(T2S);
%         end
%         
%         % If the end of index is less than the start of match
%         if(data(index,T2E)<data(match,T1S))
%             data_new(T1S)=data(index,T2S);
%         end
%         
%         
        
    end
    
        

end