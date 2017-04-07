function [numNew,result,primiLookForRepeat_cl,primiNumRepeat_cl,primiMarker_cl,uptillnow_pc] = last_primitivesCleanUp(primiData,primiIndex,primiLookForRepeat_cl,primiNumRepeat_cl,primiMarker_cl)

    numNew = 0;
    result = zeros(1,7);
    
    while (primiMarker_cl+1 <= primiIndex )
        [hasNew_pc,data_new_pc, primiLookForRepeat_cl, primiNumRepeat_cl, primiMarker_cl] = rt_primitivesCleanUp(primiData, primiLookForRepeat_cl, primiNumRepeat_cl, primiMarker_cl);
        if (hasNew_pc)
            % Increase counter
            numNew = numNew+1;
            % Keep history of primitive layer clean up data
            result(numNew,:) = data_new_pc;
            
        end
    end
    
    if (primiMarker_cl == primiIndex )
        if (primiLookForRepeat_cl) % if looking for repeat
            nextPrimitive   = primiNumRepeat_cl;
            startPrimitive  = primiMarker_cl-primiNumRepeat_cl;
            data_new_pc = rt_MergePrimitives(startPrimitive,primiData,nextPrimitive);
            numNew = numNew+1;
            result(numNew,:) = data_new_pc;
            
            primiMarker_cl          = primiMarker_cl+1;
            primiLookForRepeat_cl   = false;
            primiNumRepeat_cl       = 0;
            
        else
            data_new_pc = primiData(primiMarker_cl,:);
            numNew = numNew+1;
            result(numNew,:) = data_new_pc;
           
            primiMarker_cl          = primiMarker_cl+1;
        end
    end
    
    uptillnow_pc = false;
end