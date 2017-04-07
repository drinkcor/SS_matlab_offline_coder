function [numNew,result,llbehLookForRepeat_cl,llbehNumRepeat_cl,llbehMarker_cl,uptillnow_llbc] = last_llbehCompositionCleanUp(llbehData,llbehIndex,llbehLookForRepeat_cl,llbehNumRepeat_cl,llbehMarker_cl)
    
    numNew = 0;
    result = zeros(1,17);
    
    % Initialization
    llbehLbl        = [ 1,   2,   3,   4,   5,   6,   7,  8];

    behLbl          = 1;    % low level behavior class (low level behavior layer)
    
    
    while (llbehMarker_cl+1 <= llbehIndex)
        [hasNew_llbc, data_new, llbehLookForRepeat_cl, llbehNumRepeat_cl, llbehMarker_cl] = rt_llbehCompositionCleanUp(llbehData,  llbehLookForRepeat_cl, llbehNumRepeat_cl, llbehMarker_cl);
        if (hasNew_llbc)
            % Increase counter
            numNew = numNew+1;
            % Keep history of MC layer clean up data
            result(numNew,:) = data_new;
        end
    end
    
    if (llbehMarker_cl == llbehIndex)
        if (llbehLookForRepeat_cl) % if looking for repeat
            nextPrimitive   = llbehNumRepeat_cl;
            startPrimitive  = llbehMarker_cl-llbehNumRepeat_cl;
            repeat_Lbl      = llbehData(startPrimitive,behLbl);
            data_new        = rt_MergeLowLevelBehaviors(startPrimitive,llbehData,llbehLbl,repeat_Lbl,nextPrimitive);
            numNew = numNew+1;
            result(numNew,:) = data_new;
            
            llbehMarker_cl = llbehMarker_cl+1;
            llbehLookForRepeat_cl = false;
            llbehNumRepeat_cl = 0;
        else
            data_new        = llbehData(llbehMarker_cl,:);
            numNew = numNew+1;
            result(numNew,:) = data_new;
            
            llbehMarker_cl = llbehMarker_cl+1;
        end
    end
    
    uptillnow_llbc = false;
end