function [numNew,result,marker_cm,uptillnow_cm]=last_CompoundMotionComposition(primiData_cl,primiIndex_cl,marker_cm)

    numNew = 0;
    result = zeros(1,11);
    
    while (marker_cm+1 <= primiIndex_cl)
        [hasNew_cm, data_new, marker_cm] = rt_CompoundMotionComposition(primiData_cl, marker_cm, 0);
        if (hasNew_cm)
            % Increase counter
            numNew = numNew+1;
            result(numNew,:) = data_new;
        end
    end
    
    if (marker_cm == primiIndex_cl)
        [hasNew_cm, data_new, marker_cm] = rt_CompoundMotionComposition(primiData_cl, marker_cm, 1);
        if (hasNew_cm)
            % Increase counter
            numNew = numNew+1;
            result(numNew,:) = data_new;
        end
    end

    uptillnow_cm = false;

end