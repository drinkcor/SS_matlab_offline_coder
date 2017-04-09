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
% motComps:     - [actionClass,avgVal,rmsVal,AmplitudeVal,p1label,p2label,t1Start,t1End,t2Start,t2End,tavgIndex]
%
% Output Parameters:
% htext         - handle to the text objects in case user wants to modify
%
% Modifications:
% July 2012 - to facilitate the conversion from Matlab to C++ all cells
% have been eliminated. All {} were converted to ()
%**************************************************************************
function htext = plotMotionCompositions(StrategyType,rHandle,TL,BL,motComps,motCompsIndex)

%%  Preprocessing
    len     = length(rHandle);          % Check how many hanlde entries we have
    htext   = zeros(motCompsIndex,1);               % This is a text handle and can be used if we want to modify/delete the text

    % Indeces
    compString  = 1;                    % type of composition: alignment, increase, decrease, constant
    AvgTime     = 11;               	% Used as an index. Always verify to make sure the index is not obsolete.
    
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
        height_LLB =minHeight+((maxHeight-minHeight)*0.95);
    else 
        height_LLB =minHeight+((maxHeight-minHeight)*0.95);
    end   

    
%%  Labeling
    
    % For each of the handles
    for i=1:len                                 % getting 7 handles instead of six...
        
        % For each of the compositions
        % PA10
        for index=1:motCompsIndex                                            % rows            
            htext(i)=text(motComps(index,AvgTime),...               % x-position. Average time of composition.
                          height_LLB,...                            % y-position. Set it at 75% of the top boundary of the axis +/- randn w/ sigma = TL*0.04
                          rt_actionInt2actionLbl(...
                          motComps(index,compString)),...           % Composition string: alignment, increase, decrease, constant.
                          'FontSize',8,...                          % Size of font. (Changed from 7.5 to 8).
                          'FontWeight','light',...                  % Font weight can be light, normal, demi, bold
                          'HorizontalAlignment','center');          % Alignment of font: left, center, right.             
        end
    end
end