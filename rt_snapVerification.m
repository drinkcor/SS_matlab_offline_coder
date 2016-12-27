% % Structure of ForcePicture: (Copy to a fixed structure(sliced varialbe in parfor) for post-processing)
% ForceCell= {-
%                 Fx = { statData, segmentIndex, motComs, compouIndex, llbehStruc, llbehIndex, dataFit_pre, wStart, marker, statData_pc, pc_index, lookForRepeat, numberRepeated, marker_pc, marker_cm, }
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
    
    actionLbl       = [1,2,3,4,5,6,7,8];                  % This array has been updated to be an int vector
    
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

  
    % Primitives Structure Indeces
    p_mxAmp           = 2;  
    p_minAmp          = 3;
    % Time Indeces
    p_T1S             = 4; 
    p_T1E             = 5; 
    % Grad label Indeces
    p_GRAD_LBL        = 7;
    % Empirically set
    p_lengthRatio     = 5;  
    p_amplitudeRatio  = 2;

    
 
    % Mot Comps Structure Indeces
    mc_ACTN_LBL         = 1;   % action class
    mc_AVG_MAG_VAL      = 2;   % average value
    mc_RMS_VAL          = 3;   % rms value, which is max value too
    mc_AMPLITUDE_VAL    = 4;   % amplitude value 
    % Labels
    mc_P1LBL = 5; mc_P2LBL = 6;   % label indeces for both primitives
    % Time Indeces
    T1S = 7; T1E = 8;
    T2S = 9; T2E = 10;    
    TAVG_INDEX   = 11;
    
    
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
        % Variables for Primitive layer
        ForceCell{axisIndex}{1} = zeros(100,7);     % primitive layer data
        ForceCell{axisIndex}{2} = 0;                % primitive layer index
        ForceCell{axisIndex}{7} = nan;              % dataFit_pre
        ForceCell{axisIndex}{8} = 1;                % wStart
        ForceCell{axisIndex}{9} = 1;                % marker_p
        
        % Variables for Primitive layer CleanUp 
        ForceCell{axisIndex}{10} = zeros(100,7);    % primitive layer CleanUp data
        ForceCell{axisIndex}{11} = 0;               % primitive layer CleanUp index        
        ForceCell{axisIndex}{12} = 0;               % lookForRepeat, 0 means not looking for repeat, 1 means looking for repeat
        ForceCell{axisIndex}{13} = 0;               % numberRepeated
        ForceCell{axisIndex}{14} = 1;               % marker_pc
        
        % Variables for CompoundMotionComposition layer
        ForceCell{axisIndex}{3}  = zeros(100,11);   % CompoundMotionComposition layer data
        ForceCell{axisIndex}{4}  = 0;               % CompoundMotionComposition layer index
        ForceCell{axisIndex}{15} = 1;               % marker_cm
        
        % Variables for CompoundMotionComposition layer CleanUp
        ForceCell{axisIndex}{16} = zeros(100,11);   % CompoundMotionComposition layer CleanUp data
        ForceCell{axisIndex}{17} = 0;               % CompoundMotionComposition layer CleanUp index
        ForceCell{axisIndex}{18} = 0;               % lookForRepeat, 0 means not looking for repeat, 1 means looking for repeat
        ForceCell{axisIndex}{19} = 0;               % numberRepeated
        ForceCell{axisIndex}{20} = 0;               % maxAmplitude
        ForceCell{axisIndex}{21} = 1;               % marker_cmc
         
        % Variables for low level behaviour layer 
        ForceCell{axisIndex}{5} = zeros(100,17);   % CompoundMotionComposition layer CleanUp data
        ForceCell{axisIndex}{6} = 0;               % CompoundMotionComposition layer CleanUp index
        ForceCell{axisIndex}{22} = 1;               % marker_llb

        % Variables for low level behaviour layer CleanUp

        
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
                %% Primitive_layer
                %  The process won't be execute unless there is enough data for fitting.
                if (ForceCell{axisIndex}{9}+window_length <= localIndex)
                    fprintf('marker: %1.0f  ',ForceCell{axisIndex}{9});
                    fprintf('wStart: %1.0f  ',ForceCell{axisIndex}{8});
                    [hasNew_p,dAvg,dMax,dMin,dStart,dFinish,dGradient,dLabel,ForceCell{axisIndex}{7},ForceCell{axisIndex}{8},ForceCell{axisIndex}{9}] = rt_fitRegressionCurves(Wren_loc,StrategyType,FolderName,pType,ForceCell{axisIndex}{7},ForceCell{axisIndex}{8},ForceCell{axisIndex}{9},rate,axisIndex);    % fit window
                    if(hasNew_p)
                        % Increase counter
                        ForceCell{axisIndex}{2} = ForceCell{axisIndex}{2}+1;
                        % Keep history of primitive layer data 
                        ForceCell{axisIndex}{1}(ForceCell{axisIndex}{2},:) = [dAvg dMax dMin dStart dFinish dGradient dLabel];
                    end
                end

                
                %% Primitive_layer clean up    
                if (ForceCell{axisIndex}{14}+1 <= ForceCell{axisIndex}{2})
                    [hasNew_pc,data_new, ForceCell{axisIndex}{12}, ForceCell{axisIndex}{13}, ForceCell{axisIndex}{14}] = rt_primitivesCleanUp(ForceCell{axisIndex}{1}, ForceCell{axisIndex}{12}, ForceCell{axisIndex}{13}, ForceCell{axisIndex}{14});
                    if (hasNew_pc)
                        % Increase counter
                        ForceCell{axisIndex}{11} = ForceCell{axisIndex}{11}+1;
                        % Keep history of primitive layer clean up data 
                        ForceCell{axisIndex}{10}(ForceCell{axisIndex}{11},:) = data_new;
                    end
                     
                end
                
                
                %% CompoundMotionComposition_layer
                 if (ForceCell{axisIndex}{15}+1 <= ForceCell{axisIndex}{11})
                    [hasNew_cm, data_new, ForceCell{axisIndex}{15}] = rt_CompoundMotionComposition(ForceCell{axisIndex}{10}, ForceCell{axisIndex}{15}, 0);
                    if (hasNew_cm)
                        % Increase counter
                        ForceCell{axisIndex}{4} = ForceCell{axisIndex}{4}+1;
                        % Keep history of MC layer data
                        ForceCell{axisIndex}{3}(ForceCell{axisIndex}{4},:) = data_new;
                    end
                 end
                


                 %% CompoundMotionComposition_layer clean up
                 if (ForceCell{axisIndex}{21}+1 <= ForceCell{axisIndex}{4})
                    [hasNew_cmc, data_new, ForceCell{axisIndex}{18}, ForceCell{axisIndex}{19}, ForceCell{axisIndex}{20}, ForceCell{axisIndex}{21}] = rt_MotCompsCleanUp(ForceCell{axisIndex}{3},  ForceCell{axisIndex}{18}, ForceCell{axisIndex}{19}, ForceCell{axisIndex}{20}, ForceCell{axisIndex}{21});
                    if (hasNew_cmc)
                        % Increase counter
                        ForceCell{axisIndex}{17} = ForceCell{axisIndex}{17}+1;
                        % Keep history of MC layer clean up data
                        ForceCell{axisIndex}{16}(ForceCell{axisIndex}{17},:) = data_new;
                    end
                 end
                 
                 
                 
                %% Low layer behavior layer
                if (ForceCell{axisIndex}{22}+1 <= ForceCell{axisIndex}{17})
                    [hasNew_llb, data_new, ForceCell{axisIndex}{22}] = rt_llbehComposition(ForceCell{axisIndex}{16}, ForceCell{axisIndex}{22}, 0);
                    if (hasNew_llb)
                        % Increase counter
                        ForceCell{axisIndex}{6} = ForceCell{axisIndex}{6}+1;
                        % Keep history of MC layer data
                        ForceCell{axisIndex}{5}(ForceCell{axisIndex}{6},:) = data_new;
                    end
                 end
   
                %% Low layer behavior layer clean up

                
   
            end
            drawnow;
        end 
        drawnow;
        a=input('please input 2 to run the last iteration: ');
        if(a==2)
        %% Last iteration to wrap up            
            parfor axisIndex = 1:6
                
                
                %% Last iteration of primitive layer  
                %  In the last iteration of primitive layer, just 1 new item will certainly be appended to primitive layer data
                wFinish     = localIndex;                            % Set to the last index of statData (the primitives space)
                Range       = ForceCell{axisIndex}{8}:wFinish;               % Save from wStart to the immediately preceeding index that passed the threshold
                Time        = (Range)'; % Wren_loc(Range,1);          % Time indeces that we are working with
                Data        = Wren_loc(Range,axisIndex+1); % Corresponding force data for a given force element in a given window
                
                polyCoeffs  = polyfit(Time,Data,1);            % First-order fit
                dataFit     = polyval(polyCoeffs, Time);

                [dAvg,dMax,dMin,dStart,dFinish,dGradient,dLabel]=rt_statisticalData(Time(1)*(1/rate),Time(length(Time))*(1/rate),dataFit,polyCoeffs,FolderName,StrategyType,axisIndex); % 1+windowlength
    
                ForceCell{axisIndex}{2} = ForceCell{axisIndex}{2}+1;
                
                ForceCell{axisIndex}{1}(ForceCell{axisIndex}{2},:) = [dAvg dMax dMin dStart dFinish dGradient dLabel];
                
                
                
                
                %% Last iteration of primitive layer clean up       
                %  In the last iteration of primitive layer clean up, 1 or 2 new items will certainly be appended to primitive layer clean up data
                while (ForceCell{axisIndex}{14}+1 <= ForceCell{axisIndex}{2})
                    [hasNew_pc,data_new, ForceCell{axisIndex}{12}, ForceCell{axisIndex}{13}, ForceCell{axisIndex}{14}] = rt_primitivesCleanUp(ForceCell{axisIndex}{1}, ForceCell{axisIndex}{12}, ForceCell{axisIndex}{13}, ForceCell{axisIndex}{14});
                    if (hasNew_pc)
                        % Increase counter
                        ForceCell{axisIndex}{11} = ForceCell{axisIndex}{11}+1;
                        % Keep history of primitive layer clean up data 
                        ForceCell{axisIndex}{10}(ForceCell{axisIndex}{11},:) = data_new;
                    end
                end
                   
                if (ForceCell{axisIndex}{14} == ForceCell{axisIndex}{2})
                    if (ForceCell{axisIndex}{12}) % if looking for repeat
                        nextPrimitive   = ForceCell{axisIndex}{13};
                        startPrimitive  = ForceCell{axisIndex}{14}-ForceCell{axisIndex}{13};
                        ForceCell{axisIndex}{11} = ForceCell{axisIndex}{11}+1;
                        ForceCell{axisIndex}{10}(ForceCell{axisIndex}{11},:) = rt_MergePrimitives(startPrimitive,ForceCell{axisIndex}{1},nextPrimitive);
                    else
                        ForceCell{axisIndex}{11} = ForceCell{axisIndex}{11}+1;
                        ForceCell{axisIndex}{10}(ForceCell{axisIndex}{11},:) = ForceCell{axisIndex}{1}(ForceCell{axisIndex}{14},:);
                    end
                end                                
                
                
                
                
                
                %% Last iteration of compound motion layer  
                while (ForceCell{axisIndex}{15}+1 <= ForceCell{axisIndex}{11})
                    [hasNew_cm, data_new, ForceCell{axisIndex}{15}] = rt_CompoundMotionComposition(ForceCell{axisIndex}{10}, ForceCell{axisIndex}{15}, 0);
                    if (hasNew_cm)
                        % Increase counter
                        ForceCell{axisIndex}{4} = ForceCell{axisIndex}{4}+1;
                        ForceCell{axisIndex}{3}(ForceCell{axisIndex}{4},:) = data_new;
                    end
                end
                
                if (ForceCell{axisIndex}{15} == ForceCell{axisIndex}{11})
                    [hasNew_cm, data_new, ForceCell{axisIndex}{15}] = rt_CompoundMotionComposition(ForceCell{axisIndex}{10}, ForceCell{axisIndex}{15}, 1);
                    if (hasNew_cm)
                        % Increase counter
                        ForceCell{axisIndex}{4} = ForceCell{axisIndex}{4}+1;
                        ForceCell{axisIndex}{3}(ForceCell{axisIndex}{4},:) = data_new;
                    end                    
                end
                
                
                
                
                %% Last iteration of compound motion layer clean up  
                while (ForceCell{axisIndex}{21}+1 <= ForceCell{axisIndex}{4})
                    [hasNew_cmc, data_new, ForceCell{axisIndex}{18}, ForceCell{axisIndex}{19}, ForceCell{axisIndex}{20}, ForceCell{axisIndex}{21}] = rt_MotCompsCleanUp(ForceCell{axisIndex}{3},  ForceCell{axisIndex}{18}, ForceCell{axisIndex}{19}, ForceCell{axisIndex}{20}, ForceCell{axisIndex}{21});
                    if (hasNew_cmc)
                        % Increase counter
                        ForceCell{axisIndex}{17} = ForceCell{axisIndex}{17}+1;
                        % Keep history of MC layer clean up data
                        ForceCell{axisIndex}{16}(ForceCell{axisIndex}{17},:) = data_new;
                    end
                end
                   
                if (ForceCell{axisIndex}{21} == ForceCell{axisIndex}{4})
                    if (ForceCell{axisIndex}{18}) % if looking for repeat
                        nextPrimitive   = ForceCell{axisIndex}{19};
                        startPrimitive  = ForceCell{axisIndex}{21}-ForceCell{axisIndex}{19};
                        repeat_Lbl      = ForceCell{axisIndex}{3}(startPrimitive,mc_ACTN_LBL);
                        ForceCell{axisIndex}{17} = ForceCell{axisIndex}{17}+1;
                        ForceCell{axisIndex}{16}(ForceCell{axisIndex}{17},:) = rt_MergeCompositions(startPrimitive,ForceCell{axisIndex}{3},actionLbl,repeat_Lbl,nextPrimitive);
                    else
                        ForceCell{axisIndex}{17} = ForceCell{axisIndex}{17}+1;
                        ForceCell{axisIndex}{16}(ForceCell{axisIndex}{17},:) = ForceCell{axisIndex}{3}(ForceCell{axisIndex}{21},:);
                    end
                end 
                
                
                
                
                %% Last iteration of low layer behavior layer  
                while (ForceCell{axisIndex}{22}+1 <= ForceCell{axisIndex}{17})
                    [hasNew_llb, data_new, ForceCell{axisIndex}{22}] = rt_llbehComposition(ForceCell{axisIndex}{16}, ForceCell{axisIndex}{22}, 0);
                    if (hasNew_llb)
                        % Increase counter
                        ForceCell{axisIndex}{6} = ForceCell{axisIndex}{6}+1;
                        ForceCell{axisIndex}{5}(ForceCell{axisIndex}{6},:) = data_new;
                    end
                end
                
                if (ForceCell{axisIndex}{22} == ForceCell{axisIndex}{17})
                    [hasNew_llb, data_new, ForceCell{axisIndex}{22}] = rt_llbehComposition(ForceCell{axisIndex}{16}, ForceCell{axisIndex}{22}, 1);
                    if (hasNew_llb)
                        % Increase counter
                        ForceCell{axisIndex}{6} = ForceCell{axisIndex}{6}+1;
                        ForceCell{axisIndex}{5}(ForceCell{axisIndex}{6},:) = data_new;
                    end                    
                end
                
                
                %% Last iteration of low layer behavior layer clean up  
                
                
                
                
                
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
