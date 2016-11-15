%% **************************** Documentation *****************************
% Merges data between two continguous elements in a data composition data
% structure. 
% 
% The data structure is a row numeric vector array composed of 7 elements: 
% statData = [dAvg dMax dMin dStart dFinish dGradient dLabel]. 
%
% Update: 
% Merge now can also look, not just at the next neighbor but at
% many neighbors.
%
% If the value of indicator is 0 or 1, it will run the former code,
% but if it is larger than 1, it functions as a counter and will average indicator number of primitives.
% 
% Input Parameters:
% index:            - first element of contiguous pair.
% data:             - an mx11 cell array data structure containing action compositions
% gradientLbl       - array structure that holds strings for primitives (not used after 2013Aug)
% indicator         - value of 0 or 1 used to select the first primitive or the second
%                     primitive. 
% 
% Output:
% Returns the whole data structure.
%**************************************************************************
function data_new = rt_MergePrimitives(index,data,indicator)
%function data = MergePrimitives(index,data,gradientLbl,indicator)

%%  Initialization

    % Define next contiguous element
    match = index+1;

%%  GRADIENT PRIMITIVES
    % primitives Structure Indeces
     AVG_MAG_VAL      = 1;   % average value of primitive
     MAX_VAL          = 2;   % maximum value of a primitive
     MIN_VAL          = 3;   % minimum value of a primitive   

     % Time Indeces
     T1S = 4; 
     T1E = 5;
    
    % Gradient Indeces
    GRAD_VAL    = 6;
    GRAD_LBL    = 7;
    data_new    = data(index,:);
    
%% Merge according the ratio (including the situation that numberRepeated==1)
    if(indicator<2)

    %%  Name Label 
        data_new(GRAD_LBL)          = data(index+indicator,GRAD_LBL); % Keep the label of the gradient that is longer

    %%  Values                                                        
        % Average average magnitude value: use their time duration as weight
        Tindex                      = data(index,T1E)-data(index,T1S);
        Tmatch                      = data(match,T1E)-data(match,T1S);
        Twhole                      = data(match,T1E)-data(index,T1S);
        
        data_new(AVG_MAG_VAL)       = ( data(index,AVG_MAG_VAL) * Tindex + data(match,AVG_MAG_VAL) * Tmatch ) / Twhole;

        % MAX_VAL value: keep the maximum value that comes from either one
        data_new(MAX_VAL)           = max( data(index,MAX_VAL),data(match,MAX_VAL) ); 

        % MIN_VAL value: (index+match)/2
        data_new(MIN_VAL)           = min( data(index,MIN_VAL),data(match,MIN_VAL) );     

    %%  Time
        % T1_END,index = T2_END,index
        data_new(T1E) = data(match,T1E);

    %% Gradient
        % Average gradient values
        data_new(GRAD_VAL)          = ( data(index,GRAD_VAL)*Tindex + data(match,GRAD_VAL)*Tmatch ) / Twhole; 
    
%%  Merge according to repeated primitives    
    else
        
        %% Label: Nothing needs to be done. We will keep the lable of the first primitive.
        
        %%  Values                                                        
            % Average average magnitude value: use their time duration as weight
            sum_mag = 0;
            for i=index:index+indicator
               sum_mag = sum_mag + data(i,AVG_MAG_VAL) * ( data(i,T1E) - data(i,T1S) ); 
            end
            Twhole = data(index+indicator,T1E) - data(index,T1S);
            data_new(AVG_MAG_VAL)   = sum_mag / Twhole; 

            % MAX_VAL value: (index+match)/2
            data_new(MAX_VAL)       = max( data(index:index+indicator,MAX_VAL) ); 

            % MIN_VAL value: (index+match)/2
            data_new(MIN_VAL)       = min( data(index:index+indicator,MIN_VAL) );    

        %%  Time
            % T1_END,index = T2_END,index
            data_new(T1E) = data(index+indicator,T1E);

        %% Gradient
            sum_grad = 0;
            for i=index:index+indicator
               sum_grad = sum_grad + data(i,GRAD_VAL) * ( data(i,T1E) - data(i,T1S) ); 
            end
            % Average gradient values
            data_new(GRAD_VAL)   = sum_grad / Twhole;    
      
    end
    
end