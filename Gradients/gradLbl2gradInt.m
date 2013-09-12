%%-------------------------------------------------------------------------
% gradLbl2gradInt
% -Imp -Big -Med -Small Const Small Med Big Imp
%   -4   -3   -2     -1   0       1   2   3   4
% Primitives: bpos,mpos,spos,bneg,mneg,sneg,cons,pimp,nimp,none]
%%-------------------------------------------------------------------------
function dLabel = gradLbl2gradInt(dLabel)

    % Convert labels to ints
    if(strcmp(dLabel,'bpos'))
        dLabel = 1;
    elseif(strcmp(dLabel,'mpos'))
        dLabel = 2;
    elseif(strcmp(dLabel,'spos'))
        dLabel = 3;
    elseif(strcmp(dLabel,'bneg'))
        dLabel = 4;
    elseif(strcmp(dLabel,'mneg'))
        dLabel = 5;
    elseif(strcmp(dLabel,'sneg'))
        dLabel = 6;
    elseif(strcmp(dLabel,'cons'))
        dLabel = 7;
    elseif(strcmp(dLabel,'pimp'))
        dLabel = 8;
    elseif(strcmp(dLabel,'nimp'))
        dLabel = 9;
    elseif(strcmp(dLabel,'none'))
        dLabel = 10;
    else
        error('There is an incomplete conversion from string to int for gradLbls');
    end    
end