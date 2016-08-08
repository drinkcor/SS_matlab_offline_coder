% % Structure of ForcePicture: (Copy to a fixed structure(sliced varialbe in parfor) for post-processing)
% ForceCell= {-
%                 Fx = { statData, segmentIndex, motComs, compouIndex, llbehStruc, llbehIndex, dataFit_pre, wStart, marker, statData_pc, pc_index, lookForRepeat, numberRepeated, marker_pc}
%                 Fy = { statData, segmentIndex, motComs, compouIndex, llbehStruc, llbehIndex }
%                 Fz = { statData, segmentIndex, motComs, compouIndex, llbehStruc, llbehIndex }
%                 Mx = { statData, segmentIndex, motComs, compouIndex, llbehStruc, llbehIndex }
%                 My = { statData, segmentIndex, motComs, compouIndex, llbehStruc, llbehIndex }
%                 Mz = { statData, segmentIndex, motComs, compouIndex, llbehStruc, llbehIndex }
% if you want to get segmentIndex of Fx, you should use: ForceCell{1}{1}
% if you want to get compouIndex of My, you should use: ForceCell{5}{3}


function  rt_snapVerification(StrategyType,FolderName)
%UNTITLED real time RCBHT
%   Detailed explanation goes here

    %%  Gradient Classification Structure 

    % Create string array:
    gradLabels = [ 'bpos';   ... % big   pos grads
                   'mpos';   ... % med   pos grads
                   'spos';   ... % small pos grads
                   'bneg';   ... % big   neg grads
                   'mneg';   ... % med   neg grads
                   'sneg';   ... % small neg grads
                   'cons';  ... % constant  grads
                   'pimp';   ... % large pos grads
                   'nimp';   ... % large neg grads
                   'none'];
    
    %   % CONSTANTS FOR gradLabels (defined in fitRegressionCurves.m)
    BPOS            = 1;        % big   pos gradient
    MPOS            = 2;        % med   pos gradient
    SPOS            = 3;        % small pos gradient
    BNEG            = 4;        % big   neg gradient
    MNEG            = 5;        % med   neg gradient
    SNEG            = 6;        % small neg gradient
    CONST           = 7;        % constant  gradient
    PIMP            = 8;        % large pos gradient 
    NIMP            = 9;        % large neg gradient
  % NONE            = 10;       % none

    % Amplitude Indeces
    mxAmp           = 2;  
    minAmp          = 3;
    % Time Indeces
    T1S             = 4; 
    T1E             = 5; 
    % Grad label Indeces
    GRAD_LBL        = 7;
    
    lengthRatio     = 5;  % Empirically set
    amplitudeRatio  = 2;

    global globalIndex;
    global Wrench;
    global Wrench_new;
    
    globalIndex = 0;
    Wrench      = zeros(1000,7);
    Wrench_new  = zeros(1,6);
    
    plotType = ['Fx';'Fy';'Fz';'Mx';'My';'Mz'];
    llbehLbl    = [ 1,   2,   3,   4,   5,   6,   7,  8];

    localIndex  = 0;
    Wren_loc    = zeros(1000,7);

        
    rosshutdown;
    rosinit;
    
    % pub = rospublisher('/robot/limb/right/endpoint_state', 'baxter_core_msgs/EndpointState');
    
    wrenchHandle  = rossubscriber('/robot/limb/right/endpoint_state',@wrenchCallback, 'BufferSize', 1000);
    % wrench_node_sub = robotics.ros.Node('/wrench_node_subscriber');
    % wrenchHandle    = robotics.ros.Subscriber(wrench_node_sub,'/robot/limb/right/endpoint_state','baxter_core_msgs/EndpointState',@wrenchCallback, 'BufferSize', 1);
    
    % Delete any possible existing pools running previously
    delete(gcp);
    parpool(2);
    
    for axisIndex = 1:6
        % Variables for primitive level
        ForceCell{axisIndex}{1} = zeros(100,7);     % primitive level data
        ForceCell{axisIndex}{2} = 1;                % primitive level index
        ForceCell{axisIndex}{7} = nan;              % dataFit_pre
        ForceCell{axisIndex}{8} = 1;                % wStart
        ForceCell{axisIndex}{9} = 1;                % marker
        
        % Variables for primitive level CleanUp 
        ForceCell{axisIndex}{10} = zeros(100,7);    % primitive level CleanUp data
        ForceCell{axisIndex}{11} = 1;               % primitive level CleanUp index        
        ForceCell{axisIndex}{12} = 0;               % lookForRepeat, 0 means not looking for repeat, 1 means looking for repeat
        ForceCell{axisIndex}{13} = 0;               % numberRepeated
        ForceCell{axisIndex}{14} = 1;               % marker
        
    end
    
    
    rate = 40;
    window_length = 5;
    
    while(1)     
        while (localIndex<globalIndex)
            % Increase Counter for local index
            localIndex=localIndex+1;
            
            Wren_loc(localIndex,2:7)  = Wrench(localIndex,2:7);                      
            %Wren_loc_new = Wrench_new;
            
            % fprintf('\tGlobalIndex: %8f\tLocalIndex: %8f\n',globalIndex,localIndex);
            
            parfor axisIndex = 1:6
                
                pType  = plotType(axisIndex,:);  
                %% Primitive_layer: the process won't be execute unless there is enough data for fitting.
                if (ForceCell{axisIndex}{9}+window_length <= localIndex)
                    fprintf('marker: %1.0f  ',ForceCell{axisIndex}{9});
                    fprintf('wStart: %1.0f  ',ForceCell{axisIndex}{8});
                    [hasNew_p,dAvg,dMax,dMin,dStart,dFinish,dGradient,dLabel,ForceCell{axisIndex}{7},ForceCell{axisIndex}{8},ForceCell{axisIndex}{9}] = rt_fitRegressionCurves(Wren_loc,StrategyType,FolderName,pType,ForceCell{axisIndex}{7},ForceCell{axisIndex}{8},ForceCell{axisIndex}{9},rate,axisIndex);    % fit window
                    if(hasNew_p)
                        % Keep history of statistical data 
                        ForceCell{axisIndex}{1}(ForceCell{axisIndex}{2},:) = [dAvg dMax dMin dStart dFinish dGradient dLabel];
                        % Increase counter
                        ForceCell{axisIndex}{2} = ForceCell{axisIndex}{2}+1;
                    end
                end

                
                %% Primitive_layer clean up    
                if (ForceCell{axisIndex}{14}+2 <= ForceCell{axisIndex}{2})
                    [hasNew_pc,data_new, ForceCell{axisIndex}{12}, ForceCell{axisIndex}{13}, ForceCell{axisIndex}{14}] = rt_primitivesCleanUp(ForceCell{axisIndex}{1}, ForceCell{axisIndex}{12}, ForceCell{axisIndex}{13}, ForceCell{axisIndex}{14});
                    if (hasNew_pc)
                        % Keep history of statistical data 
                        ForceCell{axisIndex}{10}(ForceCell{axisIndex}{11},:) = data_new;
                        % Increase counter
                        ForceCell{axisIndex}{11} = ForceCell{axisIndex}{11}+1;
                    end
                     
                end
                                             
   %            [ForceCell{axisIndex}{3},ForceCell{axisIndex}{4}] = rt_CompoundMotionComposition(ForceCell{axisIndex}{1},ForceCell{axisIndex}{2},ForceCell{axisIndex}{3},ForceCell{axisIndex}{4} );
   %            [ForceCell{axisIndex}{5},ForceCell{axisIndex}{6}] = rt_llbehComposition(ForceCell{axisIndex}{3},ForceCell{axisIndex}{4},ForceCell{axisIndex}{5},ForceCell{axisIndex}{6});

                
            end
            drawnow;
        end 
        drawnow;
        a=input('please input 2 to run the last iteration: ');
        if(a==2)
        %% Last iteration to wrap up            
            parfor axisIndex = 1:6
                
                %% Last iteration of primitive layer           
                wFinish     = localIndex;                            % Set to the last index of statData (the primitives space)
                Range       = ForceCell{axisIndex}{8}:wFinish;               % Save from wStart to the immediately preceeding index that passed the threshold
                Time        = (Range)'; % Wren_loc(Range,1);          % Time indeces that we are working with
                Data        = Wren_loc(Range,axisIndex+1); % Corresponding force data for a given force element in a given window
                
                polyCoeffs  = polyfit(Time,Data,1);            % First-order fit
                dataFit     = polyval(polyCoeffs, Time);

                [dAvg,dMax,dMin,dStart,dFinish,dGradient,dLabel]=rt_statisticalData(Time(1)*(1/rate),Time(length(Time))*(1/rate),dataFit,polyCoeffs,FolderName,StrategyType,axisIndex); % 1+windowlength
    
                ForceCell{axisIndex}{1}(ForceCell{axisIndex}{2},:) = [dAvg dMax dMin dStart dFinish dGradient dLabel];
               
                %% Last iteration of primitive layer clean up             
                if (ForceCell{axisIndex}{12})
                    if ( intcmp(ForceCell{axisIndex}{1}(ForceCell{axisIndex}{14},GRAD_LBL),ForceCell{axisIndex}{1}(ForceCell{axisIndex}{14}+1,GRAD_LBL)) )
                        nextPrimitive   = ForceCell{axisIndex}{13}+1;
                        startPrimitive  = ForceCell{axisIndex}{14}-ForceCell{axisIndex}{13};
                        ForceCell{axisIndex}{10}(ForceCell{axisIndex}{11},:) = rt_MergePrimitives(startPrimitive,ForceCell{axisIndex}{1},nextPrimitive);
                    else
                        nextPrimitive   = ForceCell{axisIndex}{13};
                        startPrimitive  = ForceCell{axisIndex}{14}-ForceCell{axisIndex}{13};
                        ForceCell{axisIndex}{10}(ForceCell{axisIndex}{11},:) = rt_MergePrimitives(startPrimitive,ForceCell{axisIndex}{1},nextPrimitive);
                        ForceCell{axisIndex}{11} = ForceCell{axisIndex}{11}+1;
                        ForceCell{axisIndex}{10}(ForceCell{axisIndex}{11},:) = ForceCell{axisIndex}{1}(ForceCell{axisIndex}{14}+1,:);
                    end
                else
                    if  ( intcmp(ForceCell{axisIndex}{1}(ForceCell{axisIndex}{14},GRAD_LBL),ForceCell{axisIndex}{1}(ForceCell{axisIndex}{14}+1,GRAD_LBL)) )
                        nextPrimitive   = 1;
                        startPrimitive  = ForceCell{axisIndex}{14};
                        ForceCell{axisIndex}{10}(ForceCell{axisIndex}{11},:) = rt_MergePrimitives(startPrimitive,ForceCell{axisIndex}{1},nextPrimitive);
                    else
                        if ( ~intcmp(ForceCell{axisIndex}{1}(ForceCell{axisIndex}{14},GRAD_LBL),PIMP) && ~intcmp(ForceCell{axisIndex}{1}(ForceCell{axisIndex}{14},GRAD_LBL),NIMP) ) 
                            amp1 = abs(ForceCell{axisIndex}{1}(ForceCell{axisIndex}{14},mxAmp)-ForceCell{axisIndex}{1}(ForceCell{axisIndex}{14},minAmp));
                            amp2 = abs(ForceCell{axisIndex}{1}(ForceCell{axisIndex}{14}+1,mxAmp)-ForceCell{axisIndex}{1}(ForceCell{axisIndex}{14}+1,minAmp));
                            ampRatio = amp2/amp1;       
                            
                            if (ampRatio <= amplitudeRatio && ampRatio >= inv(amplitudeRatio) && ampRatio~=0 && ampRatio~=inf )
                                p1time = ForceCell{axisIndex}{1}(ForceCell{axisIndex}{14},T1E)-ForceCell{axisIndex}{1}(ForceCell{axisIndex}{14},T1S);       % Get duration of first primitive
                                p2time = ForceCell{axisIndex}{1}(ForceCell{axisIndex}{14}+1,T1E)-ForceCell{axisIndex}{1}(ForceCell{axisIndex}{14}+1,T1S);   % Get duration of second primitive    
                                ratio  = p1time/p2time;

                                if(ratio~=0 && ratio~=inf && ratio > lengthRatio)
                                    thisPrim = 0;            % First primitive is longer
                                    ForceCell{axisIndex}{10}(ForceCell{axisIndex}{11},:)  = rt_MergePrimitives(ForceCell{axisIndex}{14},ForceCell{axisIndex}{1},thisPrim);
                                elseif(ratio~=0 && ratio~=inf && ratio < inv(lengthRatio))
                                    nextPrim = 0;            % Second primitive is longer
                                    ForceCell{axisIndex}{10}(ForceCell{axisIndex}{11},:)  = rt_MergePrimitives(ForceCell{axisIndex}{14},ForceCell{axisIndex}{1},nextPrim);
                                else
                                    ForceCell{axisIndex}{10}(ForceCell{axisIndex}{11},:) = ForceCell{axisIndex}{1}(ForceCell{axisIndex}{14},:);
                                    ForceCell{axisIndex}{11} = ForceCell{axisIndex}{11}+1;
                                    ForceCell{axisIndex}{10}(ForceCell{axisIndex}{11},:) = ForceCell{axisIndex}{1}(ForceCell{axisIndex}{14}+1,:);
                                end
                            else
                                ForceCell{axisIndex}{10}(ForceCell{axisIndex}{11},:) = ForceCell{axisIndex}{1}(ForceCell{axisIndex}{14},:);
                                ForceCell{axisIndex}{11} = ForceCell{axisIndex}{11}+1;
                                ForceCell{axisIndex}{10}(ForceCell{axisIndex}{11},:) = ForceCell{axisIndex}{1}(ForceCell{axisIndex}{14}+1,:);
                            end
                        else
                            ForceCell{axisIndex}{10}(ForceCell{axisIndex}{11},:) = ForceCell{axisIndex}{1}(ForceCell{axisIndex}{14},:);
                            ForceCell{axisIndex}{11} = ForceCell{axisIndex}{11}+1;
                            ForceCell{axisIndex}{10}(ForceCell{axisIndex}{11},:) = ForceCell{axisIndex}{1}(ForceCell{axisIndex}{14}+1,:);
                        end
                    end
                end
                        
                        
                        
                
                
                
                
                %% Last iteration of compound motion layer  
                
                %% Last iteration of compound motion layer clean up  
                
                %% Last iteration of low level behavior layer  
                
                %% Last iteration of low level behavior layer clean up  
                
                %% Last iteration of high level behavior layer
                
                %% Last iteration of high level behavior layer clean up  
                
            end
            break;
        end
    end
     

    plot(1:localIndex,Wren_loc(:,2));
    hold on;
    plot(1:localIndex,Wren_loc(:,3));
    plot(1:localIndex,Wren_loc(:,4));
      
    save('Wren_loc.mat','Wren_loc');
    
       
       
       
end
