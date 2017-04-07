function [numNew, result, lookForRepeat, numRepeat, maxAmplitude, marker_cmc, uptillnow_cmc] = last_MotCompsCleanUp(MCData,MCIndex,lookForRepeat, numRepeat, maxAmplitude, marker_cmc)
    
    numNew = 0;
    result = zeros(1,11);
    
    % Initialization
    actionLbl       = [1,2,3,4,5,6,7,8];                  % This array has been updated to be an int vector
   
    mc_ACTN_LBL     = 1;    % action class (motion compound layer)

    while (marker_cmc+1 <= MCIndex)
        [hasNew_cmc, data_new, lookForRepeat, numRepeat, maxAmplitude, marker_cmc] = rt_MotCompsCleanUp(MCData,  lookForRepeat, numRepeat, maxAmplitude, marker_cmc);
        if (hasNew_cmc)
            % Increase counter
            numNew = numNew+1;
            % Keep history of MC layer clean up data
            result(numNew,:) = data_new;
        end
    end
    
    if (marker_cmc == MCIndex)
        if (lookForRepeat) % if looking for repeat
            nextPrimitive   = numRepeat;
            startPrimitive  = marker_cmc-numRepeat;
            repeat_Lbl      = MCData(startPrimitive,mc_ACTN_LBL);
            data_new        = rt_MergeCompositions(startPrimitive,MCData,actionLbl,repeat_Lbl,nextPrimitive);
            numNew = numNew+1;
            result(numNew,:) = data_new;
            
            marker_cmc = marker_cmc+1;
            lookForRepeat = false;
            numRepeat   = 0;
        else
            data_new        = MCData(marker_cmc,:);
            numNew = numNew+1;
            result(numNew,:) = data_new;
            
            marker_cmc = marker_cmc+1;
        end
    end

    uptillnow_cmc = false;
end

