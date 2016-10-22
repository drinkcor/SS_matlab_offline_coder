%% **************************** Documentation *****************************
% Merges data between two continguous elements in a data composition data
% structure. 
%
% Code updated in 2013Aug. Currently this only merges two contiguous
% compositions. We want to update it such that it can merge multiple
% compositions. To keep compatibility, we just look at the variable
% whatComposition as an indicator of how many compositions to merge. 
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
% LABEL_FLAG        - if true, sets the value of the label
% AMPLITUDE_FLAG    - if true, computes the average amplitude. 
% whatComposition   - if value = -1, indicates the 1st and 2nd composition are equally important.
%                     if value = 0, indicates we should merge the 2nd composition onto the first.
%                     if value = 1, indicates we should merge the 1st composition onto the second.
%                     if value = n, indicates we should merge the repeated compositions.
%**************************************************************************
function data_new = rt_MergeCompositions(index,data,actionLbl,actionLblIndex,LABEL_FLAG,AMPLITUDE_FLAG,whatComposition)

%%  Initialization

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
    
    data_new     = data(index,:);
    
    if(whatComposition<2)
    %%  Name Label 
    
        data(index,ACTN_LBL) = actionLbl(actionLblIndex); % Set the name of the ACTN_LBL to the one passed by in actionLblIndex. Could be 'a' or 'c', etc.


    %%  Values                                                        
        % Average magnitude value: (index+match)/2
        data_new(index,AVG_MAG_VAL)   = ( data(index,AVG_MAG_VAL)   + data(match,AVG_MAG_VAL) )/2; 

        % MaxVal 2013Sept replaced: 
        data_new(index,RMS_VAL)       = max( data(index,RMS_VAL),data(match,RMS_VAL) ); 
        % RMS value: (index+match)/2
        % data(index,RMS_VAL)       = ( data(index,RMS_VAL)       + data(match,RMS_VAL) )/2; 

        % Amplitude value: (index+match)/2
        data(index,AMPLITUDE_VAL) = max(data(index,AMPLITUDE_VAL),data(match,AMPLITUDE_VAL));

    %%  LABELS
        % Arrange lables appropriately
        % If whatComposition==1,
        % Let the second label of the first composition to be the
        % second label of the second composition. Ignore the inbetween values. 
        
        if(indicator>2) %repeated
            data(index,AVG_MAG_VAL)   = mean( data(index:index+whatComposition,AVG_MAG_VAL) );
            maxx = max( data(index:index+whatComposition,RMS_VAL)); 
            minn = min( data(index:index+whatComposition,RMS_VAL)-data(index:index+whatComposition,AMPLITUDE_VAL) ); 
            data(index,AMPLITUDE_VAL) = maxx-minn;
            
            
            % T2_END,index = T2_END,match
            data(index,T2E) = data(index+whatComposition,T2E);     
            
            data(index,TAVG_INDEX) = sum( data(index,T1S) + data(index+whatComposition,T2E) )/2;
            % data(index,T1E) should be equal to data(index,T2S)
            data(index,T2S) = data(index,TAVG_INDEX);   
            data(index,T1E) = data(index,TAVG_INDEX);        
            
            % Set sub Labels according to MC model
            if(actionLblIndex=='a')
                data(index,P1LBL) = 'pos';
                data(index,P1LBL) = 'neg';
            elseif(actionLblIndex=='i')
                data(index,P1LBL) = 'pos';
                data(index,P1LBL) = 'pos';    
            elseif(actionLblIndex=='d')
                data(index,P1LBL) = 'neg';
                data(index,P1LBL) = 'neg'; 
            elseif(actionLblIndex=='k')
                data(index,P1LBL) = 'const';
                data(index,P1LBL) = 'const';
            elseif(actionLblIndex=='pc')
                data(index,P1LBL) = 'pimp';
                data(index,P1LBL) = 'pimp';
            elseif(actionLblIndex=='nc')
                data(index,P1LBL) = 'nimp';
                data(index,P1LBL) = 'nimp';
            elseif(actionLblIndex=='c')
                data(index,P1LBL) = 'pimp';
                data(index,P1LBL) = 'nimp';
            end
        else
            % not for n(n>2) repeated, just for merging two MC or 2 repeated
            % check merging 2 repeated
                    data(index,T2E)=data(match,T2E);
                else
                    data();
                end
            else
               
            end
           
            % combination merge
        end

            
            
            
            
        end
        if(whatComposition==1)
            data(index,P2LBL) = data(match,P2LBL);
        % Else set the first label of the second composition to be the label of
        % the first composition
        else
            data(index,P1LBL) = data(match,P1LBL);        
        end
    %%  Time
        if(whatComposition==1)
            % T2_END,index = T2_END,match
            data(index,T2E) = data(match,T2E);       

            % T2_START,index = T1_START,match
            data(index,T2S) = data(match,T1S);   

            % T1_END,index = T2_END,index
            data(index,T1E) = data(index,T2E);        

        % if(whatComposition==2)
        % Copy T2S first; otherwise you will copy a data that has been updated by a new value
        else
            % T2_START,index = T1_START,inedex
            data(index,T2S) = data(index,T1S);

            % T1_END,index = T2_END,match
            data(index,T1E) = data(match,T2E); 

            % T1_START,index = T1_START,match
            data(index,T1S) = data(match,T1S);        
        end

        % TAVG_INDEX
        data(index,TAVG_INDEX) = ( data(index,T1S)+data(index,T2E) )/2 ;

        
    %% Multipe compositions
    else
    %%  Name Label: Do nothing, keep the label of the orig composition

    %%  Values                                                        
        % Average magnitude value: (index+match)/2
        data(index,AVG_MAG_VAL)   = mean( data(index:index+whatComposition,AVG_MAG_VAL) ); 

        % MaxValue 2013Sept now replaces RMS
        data(index,RMS_VAL)       = max( data(index:index+whatComposition,RMS_VAL)      ); 
        
        % RMS value: (index+match)/2
        %data(index,RMS_VAL)       = mean( data(index:index+whatComposition,RMS_VAL)    ); 

        if(AMPLITUDE_FLAG)
            % Amplitude value: take the maximum amplitude. This is a difficult number to compute as the data currently stands... Need max and min points.
            % Take the max value. These points are adjacent. The AMPLITUDE would not decrease, could only increase.T
            data(index,AMPLITUDE_VAL) = max( data(index:index+whatComposition,AMPLITUDE_VAL) ); 
        end

    %%  LABELS
        % Here, we take the 2nd label of the last composition and place it
        % as the second label of index
        if(whatComposition==1)
            data(index,P2LBL) = data(index+whatComposition,P2LBL);
       
        end
    %%  Time
    %  Note!!! This merging requires us to skip some time boundaries of
    %  merged compositions. The best we can do is keep the boundaries. When
    %  one looks at the Composition file, times will not match for every corresponding 
    %  segment/composition. That is because we had to skip them here. It is not wrong,
    % it's just something we have to deal with. 
    
        % T2_END,index = T2_END,match
        data(index,T2E) = data(index+whatComposition,T2E);       

        % T2_START,index = T1_START,match
        data(index,T2S) = data(index+whatComposition,T1S);   

        % T1_END,index = T2_END,index
        data(index,T1E) = data(index,T2E);        

        % TAVG_INDEX: (T1S+T2E)/2
        data(index,TAVG_INDEX) = sum( data(index,T1S) + data(index+whatComposition,T2E) )/2;                                     
        
        end
    
        
        if(intcmp(motComps(i,ACTN_LBL),actionLbl(pos_contact)))
            if(abs(motComps(i,AMPLITUDE_VAL))<AMP_WINDOW_PC_NC*abs(maxAmplitude))
                motComps(i,ACTN_LBL)=actionLbl(increase);
            end

        % Find negative contacts
        elseif(intcmp(motComps(i,ACTN_LBL),actionLbl(neg_contact)))
            if(abs(motComps(i,AMPLITUDE_VAL))<AMP_WINDOW_PC_NC*abs(maxAmplitude))
                motComps(i,ACTN_LBL)=actionLbl(decrease);
            end            
        end
end