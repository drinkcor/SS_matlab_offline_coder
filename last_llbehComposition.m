function [numNew, result, llbehMarker, uptillnow_llbeh] = last_llbehComposition(MCData_cl, MCIndex_cl, llbehMarker)
    
    numNew = 0;
    result = zeros(1,17);
    
    while (llbehMarker+1 <= MCIndex_cl)
        [hasNew_llb, data_new, llbehMarker] = rt_llbehComposition(MCData_cl, llbehMarker, 0);
        if (hasNew_llb)
            % Increase counter
            numNew = numNew+1;
            result(numNew,:) = data_new;
        end
    end
    
    if (llbehMarker == MCIndex_cl)
        [hasNew_llb, data_new, llbehMarker] = rt_llbehComposition(MCData_cl, llbehMarker, 1);
        if (hasNew_llb)
            % Increase counter
            numNew = numNew+1;
            result(numNew,:) = data_new;
        end
    end
    
    uptillnow_llbeh = false;
end