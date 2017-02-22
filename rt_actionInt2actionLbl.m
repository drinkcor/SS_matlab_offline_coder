%%-------------------------------------------------------------------------
% actionInt2actionLbl
% a i d k pc nc c u
% 1 2 3 4 5  6  7 8
%%-------------------------------------------------------------------------
function actionLbl = rt_actionInt2actionLbl(actionLbl)

    % Convert labels to ints
    if(actionLbl==1)
        actionLbl = 'a';    % alignment
    elseif(actionLbl==2)
        actionLbl = 'i';    % increase
    elseif(actionLbl==3)
        actionLbl = 'd';    % decrease
    elseif(actionLbl==4)
        actionLbl = 'k';    % constant
    elseif(actionLbl==5)
        actionLbl = 'pc';    % positive contactr
    elseif(actionLbl==6)
        actionLbl = 'nc';    % negative contact
    elseif(actionLbl==7)
        actionLbl = 'c';    % contact
    elseif(actionLbl==8)
        actionLbl = 'u';    % unstable
    elseif(actionLbl==9)
        actionLbl = 'n';    % noise
    elseif(actionLbl==10)
        actionLbl = 'z';    % none
    end
end


