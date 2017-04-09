%********************* Documentation **************************************
% This function LABELS the plot of primitives with composition labels. 
% The set of compositions include: (alignment, increase, decrease, constant)
% and represendted by the strings: ('a','i','d','c').
% 
% Positioning of Labels:
% The dependent axis is time. The average time between two primitives that
% have been compounded together is used to set the x-position of the label.
%
% The y-position is set 
% 
% Text Labeling:
% The text labeling is performed by extracting the first category or
% element of the CELL ARRAY motComps of type string. 
%
% data:     - refers to the llbehStruc[llBehClass,avgVal,rmsVal,AmplitudeVal,mc1,mc2,t1Start,t1End,t2Start,t2End,tavgIndex]
%
% Output Parameters:
% htext         - handle to the text objects in case user wants to modify
%**************************************************************************
function htext = plotLowLevelBehCompositions(StrategyType,rHandle,TL,BL,data,dataIndex)

%%  Preprocessing
    len     = length(rHandle);          % Check how many hanlde entries we have
    htext   = zeros(dataIndex,1);               % This is a text handle and can be used if we want to modify/delete the text

    % Indeces
    LblIndex  = 1;                % type of composition: alignment, increase, decrease, constant
    AvgTime   = 17;                % Used as an index
    
   % Maximum height of plot
    fig_handle = gca;
    maxHeight = fig_handle.YLim(2);
    minHeight = fig_handle.YLim(1);
    
    % Set Text Upper Height Limit
    if(TL>maxHeight)
        TL=maxHeight;
    elseif(TL<minHeight)
        TL=minHeight;
    end
    
    % Set Text Lower Height Limit
    if(BL<minHeight)
        BL=minHeight;
    elseif(BL>maxHeight)
        BL=maxHeight;
    end     
    
    % Set Text Height Levels. 
    if(maxHeight>0) % 
        height_LLB =minHeight+((maxHeight-minHeight)*0.85);
    else 
        height_LLB =minHeight+((maxHeight-minHeight)*0.85);
    end   
    
%%  Labeling
    
    % For each of the handles
    for i=1:len                                 % getting 7 handles instead of six...        
        % For each of the compositions
        for index=1:dataIndex;                                           % rows
            % Pivot Approach, it has 5 states. 
            if(~strcmp(StrategyType,'HSA'))
                htext(i)=text(data(index,AvgTime),...               % x-position. Average time of composition.
                              height_LLB,...                        % y-position. No randomness here since there is no overcrowding... //Set it at 75% of the top boundary of the axis +/- randn w/ sigma = TL*0.04
                              data(index,LblIndex),...              % Composition string: alignment, increase, decrease, constant.
                              'Color',[1,0,0],...                   % Font color
                              'FontSize',8.5,...                  	% Size of font. Changed from 7.5 to 8.5
                              'FontWeight','light',...              % Font weight can be light, normal, demi, bold
                              'HorizontalAlignment','center');      % Alignment of font: left, center, right. 
            % Side Approach, it has 4 states. 
            else
                htext(i)=text   (data(index,AvgTime),...            % x-position. Average time of composition.
                                height_LLB,...                      % y-position. No randomness here since there is no overcrowding... //Set it at 75% of the top boundary of the axis +/- randn w/ sigma = TL*0.04
                                llbInt2llbLbl(...
                                data(index,LblIndex)),...           % Composition string: alignment, increase, decrease, constant.
                                'Color',              [1,0,0],...   % Font color
                                'FontSize',            8.5,...      % Size of font. Changed from 7.5 to 8.5
                                'FontWeight',         'light',...   % Font weight can be light, normal, demi, bold
                                'HorizontalAlignment','center');    % Alignment of font: left, center, right. 
            end
        end
    end       
end