
function [Result,Middle_llbeh] = SharedM_subSnapVerification(axisIndex, cl_cycle_primi, cl_cycle_mc, cl_cycle_llbeh, StrategyType, FolderName)
    
    % Initialization
   
    actionLbl       = [1,2,3,4,5,6,7,8];                  % This array has been updated to be an int vector
    
    plotType        = ['Fx';'Fy';'Fz';'Mx';'My';'Mz'];
    pType           = plotType(axisIndex,:); 
    
    llbehLbl        = [ 1,   2,   3,   4,   5,   6,   7,  8];

    p_GRAD_LBL      = 7;    % segment class (primitive layer)
    mc_ACTN_LBL     = 1;    % action class (motion compound layer)
    behLbl          = 1;    % low level behavior class (low level behavior layer)
    
    
    %% Variables
    % Variables for Torque data
    localIndex  = 0;
    Result.Wren_loc    = zeros(1000,7);
    
    % Variables for Primitive layer
    Result.primiData                = zeros(100,7);     % primitive layer data
    Result.primiIndex               = 0;                % primitive layer index
    Result.primiDataFitPre          = nan;              % dataFit_pre
    Result.primiWStart              = 1;                % wStart
    Result.primiMarker              = 1;                % marker_p
    
    % Variables for Primitive layer CleanUp
    Result.primiData_cl             = zeros(100,7);    % primitive layer CleanUp data
    Result.primiIndex_cl            = 0;               % primitive layer CleanUp index
    Result.primiLookForRepeat_cl    = 0;               % lookForRepeat, 0 means not looking for repeat, 1 means looking for repeat
    Result.primiNumRepeat_cl        = 0;               % numberRepeated
    Result.primiMarker_cl           = 1;               % marker_pc
    
    % Variables for CompoundMotionComposition layer
    Result.MCData                   = zeros(100,11);   % CompoundMotionComposition layer data
    Result.MCIndex                  = 0;               % CompoundMotionComposition layer index
    Result.MCMarker                 = 1;               % marker_cm
    
    % Variables for CompoundMotionComposition layer CleanUp
    Result.MCData_cl                = zeros(100,11);   % CompoundMotionComposition layer CleanUp data
    Result.MCIndex_cl               = 0;               % CompoundMotionComposition layer CleanUp index
    Result.MCLookForRepeat_cl       = 0;               % lookForRepeat, 0 means not looking for repeat, 1 means looking for repeat
    Result.MCNumRepeat_cl           = 0;               % numberRepeated
    Result.MCMaxAmp_cl              = 0;               % maxAmplitude
    Result.MCMarker_cl              = 1;               % marker_cmc
    
    % Variables for low level behaviour layer
    Result.llbehData                = zeros(100,17);    % low level behaviour layer data
    Result.llbehIndex               = 0;                % low level behaviour layer index
    Result.llbehMarker              = 1;                % marker_llb
    
    % Variables for low level behaviour layer CleanUp
    Result.llbehData_cl             = zeros(100,17);    % low level behaviour layer CleanUp data
    Result.llbehIndex_cl            = 0;                % low level behaviour layer CleanUp index
    Result.llbehLookForRepeat_cl    = 0;                % lookForRepeat, 0 means not looking for repeat, 1 means looking for repeat
    Result.llbehNumRepeat_cl        = 0;                % numberRepeated
    Result.llbehMarker_cl           = 1;                % marker_llbc
    
    
    %% cycle of clean up
    
    for i=1:cl_cycle_primi
        Middle_primi(i).primiData_cl             = zeros(100,7);    % primitive layer CleanUp data
        Middle_primi(i).primiIndex_cl            = 0;               % primitive layer CleanUp index
        Middle_primi(i).primiLookForRepeat_cl    = 0;               % lookForRepeat, 0 means not looking for repeat, 1 means looking for repeat
        Middle_primi(i).primiNumRepeat_cl        = 0;               % numberRepeated
        Middle_primi(i).primiMarker_cl           = 1;               % marker_pc
    end
    
    for i=1:cl_cycle_mc
        Middle_mc(i).MCData_cl                = zeros(100,11);   % CompoundMotionComposition layer CleanUp data
        Middle_mc(i).MCIndex_cl               = 0;               % CompoundMotionComposition layer CleanUp index
        Middle_mc(i).MCLookForRepeat_cl       = 0;               % lookForRepeat, 0 means not looking for repeat, 1 means looking for repeat
        Middle_mc(i).MCNumRepeat_cl           = 0;               % numberRepeated
        Middle_mc(i).MCMaxAmp_cl              = 0;               % maxAmplitude
        Middle_mc(i).MCMarker_cl              = 1;               % marker_cmc
    end
    
    for i=1:cl_cycle_llbeh
        Middle_llbeh(i).llbehData_cl             = zeros(100,17);    % low level behaviour layer CleanUp data
        Middle_llbeh(i).llbehIndex_cl            = 0;                % low level behaviour layer CleanUp index
        Middle_llbeh(i).llbehLookForRepeat_cl    = 0;                % lookForRepeat, 0 means not looking for repeat, 1 means looking for repeat
        Middle_llbeh(i).llbehNumRepeat_cl        = 0;                % numberRepeated
        Middle_llbeh(i).llbehMarker_cl           = 1;                % marker_llbc
    end
    
    
    
    %% ROS component that used to publish result labels
    rosshutdown;
    rosinit;
    
    segmentMsg = rosmessage('publish_files/Segments');
    compositeMsg = rosmessage('publish_files/Composites');
    llbehaviorMsg = rosmessage('publish_files/llBehaviors');

    segmentPub      = rospublisher(strcat('/topic_segments_',pType), 'publish_files/Segments');
    compositePub    = rospublisher(strcat('/topic_composites_',pType), 'publish_files/Composites');
    llbehaviorPub   = rospublisher(strcat('/topic_llbehaviors_',pType), 'publish_files/llBehaviors');
    
    
    
    window_length   = 5;
    
    breakSignal = 0;

    
    % Open updating shared file to read data
    fileR = fopen('WrenchData.bin');
    fseek(fileR,0,-1);
    
    while(1)
        
        cur = fread(fileR,[1,7],'double');
        [r,c] = size(cur);
        
        
        while(r~=0)
            
            if(c~=7)
                fseek(fileR,-c*8,0);
            else
                if(cur(1)==-1)
                    %% last iteration
                    breakSignal = 1;
                else
                    try
                        localIndex  = localIndex+1;
                        Result.Wren_loc(localIndex,:) = cur;
                    catch
                        error('Column is not equal to 7:  C is %d\n',c);
                    end
                end
            end
            cur = fread(fileR,[1,7],'double');
            [r,c] = size(cur);
            
            if(mod(localIndex,5)==1)
                
                %% Primitive_layer
                %  The process won't be execute unless there is enough data for fitting.
                while (Result.primiMarker+window_length <= localIndex)
                    %                     fprintf('marker: %1.0f  ',Result.primiMarker);
                    %                     fprintf('wStart: %1.0f  ',Result.primiWStart);
                    [hasNew_p,dAvg,dMax,dMin,dStart,dFinish,dGradient,dLabel,Result.primiDataFitPre,Result.primiWStart,Result.primiMarker] = rt_fitRegressionCurves(Result.Wren_loc,StrategyType,FolderName,pType,Result.primiDataFitPre,Result.primiWStart,Result.primiMarker,axisIndex);    % fit window
                    if(hasNew_p)
                        % Increase counter
                        Result.primiIndex  = Result.primiIndex +1;
                        % Keep history of primitive layer data
                        Result.primiData(Result.primiIndex ,:) = [dAvg dMax dMin dStart dFinish dGradient dLabel];
                    end
                end
                
                
                %% Cycles of Primitive_layer clean up, determine by parameter: cl_cycle_primi
                if(cl_cycle_primi>1)
                    %% Multiple cycles of primitive_layer clean up
                    
                    % The first cycle of Primitive_layer clean up
                    while (Middle_primi(1).primiMarker_cl+1 <= Result.primiIndex )
                        [hasNew_pc,data_new, Middle_primi(1).primiLookForRepeat_cl, Middle_primi(1).primiNumRepeat_cl, Middle_primi(1).primiMarker_cl] = rt_primitivesCleanUp(Result.primiData, Middle_primi(1).primiLookForRepeat_cl, Middle_primi(1).primiNumRepeat_cl, Middle_primi(1).primiMarker_cl);
                        if (hasNew_pc)       
                            % Increase counter
                            Middle_primi(1).primiIndex_cl = Middle_primi(1).primiIndex_cl+1;
                            % Keep history of primitive layer clean up data
                            Middle_primi(1).primiData_cl(Middle_primi(1).primiIndex_cl,:) = data_new;                          
                        end
                    end
                    
                    % The middle cycles of Primitive_layer clean up
                    if(cl_cycle_primi>2)
                        for i=2:cl_cycle_primi-1
                            while (Middle_primi(i).primiMarker_cl+1 <= Middle_primi(i-1).primiIndex_cl )
                                [hasNew_pc,data_new, Middle_primi(i).primiLookForRepeat_cl, Middle_primi(i).primiNumRepeat_cl, Middle_primi(i).primiMarker_cl] = rt_primitivesCleanUp(Middle_primi(i-1).primiData_cl, Middle_primi(i).primiLookForRepeat_cl, Middle_primi(i).primiNumRepeat_cl, Middle_primi(i).primiMarker_cl);
                                if (hasNew_pc)
                                    % Increase counter
                                    Middle_primi(i).primiIndex_cl = Middle_primi(i).primiIndex_cl+1;
                                    % Keep history of primitive layer clean up data
                                    Middle_primi(i).primiData_cl(Middle_primi(i).primiIndex_cl,:) = data_new;
                                end
                            end
                        end       
                    end
                    
                    % The last cycle of Primitive_layer clean up        
                    while (Result.primiMarker_cl+1 <= Middle_primi(cl_cycle_primi-1).primiIndex_cl )
                        [hasNew_pc,data_new, Result.primiLookForRepeat_cl, Result.primiNumRepeat_cl, Result.primiMarker_cl] = rt_primitivesCleanUp(Middle_primi(cl_cycle_primi-1).primiData_cl, Result.primiLookForRepeat_cl, Result.primiNumRepeat_cl, Result.primiMarker_cl);
                        if (hasNew_pc)          
                            % Increase counter
                            Result.primiIndex_cl = Result.primiIndex_cl+1;
                            % Keep history of primitive layer clean up data
                            Result.primiData_cl(Result.primiIndex_cl,:) = data_new;
                            
                            % Publish the new item to post processing
                            %        fprintf('Reach here segment\n');
                            segmentMsg.Average   = data_new(1);
                            segmentMsg.MaxVal    = data_new(2);
                            segmentMsg.MinVal    = data_new(3);
                            segmentMsg.TStart    = data_new(4);
                            segmentMsg.TEnd      = data_new(5);
                            segmentMsg.Gradient  = data_new(6);
                            segmentMsg.GradLabel = gradInt2gradLbl(data_new(7));
                            send(segmentPub,segmentMsg);
                            %                         send(segmentPubConstant.Value,segmentMsg);
                        end
                        
                    end
                else
                    %% The only cycle of Primitive_layer clean up
                    while (Result.primiMarker_cl+1 <= Result.primiIndex )
                        [hasNew_pc,data_new, Result.primiLookForRepeat_cl, Result.primiNumRepeat_cl, Result.primiMarker_cl] = rt_primitivesCleanUp(Result.primiData, Result.primiLookForRepeat_cl, Result.primiNumRepeat_cl, Result.primiMarker_cl);
                        if (hasNew_pc)
                            
                            % Increase counter
                            Result.primiIndex_cl = Result.primiIndex_cl+1;
                            % Keep history of primitive layer clean up data
                            Result.primiData_cl(Result.primiIndex_cl,:) = data_new;
                            
                            % Publish the new item to post processing
                            %        fprintf('Reach here segment\n');
                            segmentMsg.Average   = data_new(1);
                            segmentMsg.MaxVal    = data_new(2);
                            segmentMsg.MinVal    = data_new(3);
                            segmentMsg.TStart    = data_new(4);
                            segmentMsg.TEnd      = data_new(5);
                            segmentMsg.Gradient  = data_new(6);
                            segmentMsg.GradLabel = gradInt2gradLbl(data_new(7));
                            send(segmentPub,segmentMsg);
                            %                         send(segmentPubConstant.Value,segmentMsg);
                        end
                        
                    end
                end
                
                
                %% CompoundMotionComposition_layer
                while (Result.MCMarker+1 <= Result.primiIndex_cl)
                    [hasNew_cm, data_new, Result.MCMarker] = rt_CompoundMotionComposition(Result.primiData_cl, Result.MCMarker, 0);
                    if (hasNew_cm)
                        % Increase counter
                        Result.MCIndex = Result.MCIndex+1;
                        % Keep history of MC layer data
                        Result.MCData(Result.MCIndex,:) = data_new;
                    end
                end
                
                
                
                %% CompoundMotionComposition_layer clean up 
                
                if(cl_cycle_mc>1)
                    % Multiple cycles oc composite layer clean up
                    
                    % The first cycle
                    while (Middle_mc(1).MCMarker_cl+1 <= Result.MCIndex)
                        [hasNew_cmc, data_new, Middle_mc(1).MCLookForRepeat_cl, Middle_mc(1).MCNumRepeat_cl, Middle_mc(1).MCMaxAmp_cl, Middle_mc(1).MCMarker_cl] = rt_MotCompsCleanUp(Result.MCData,  Middle_mc(1).MCLookForRepeat_cl, Middle_mc(1).MCNumRepeat_cl, Middle_mc(1).MCMaxAmp_cl, Middle_mc(1).MCMarker_cl);
                        if (hasNew_cmc)       
                            % Increase counter
                            Middle_mc(1).MCIndex_cl = Middle_mc(1).MCIndex_cl+1;
                            % Keep history of MC layer clean up data
                            Middle_mc(1).MCData_cl(Middle_mc(1).MCIndex_cl,:) = data_new;
                        end
                    end
                    
                    % The middle cycle
                    if(cl_cycle_mc>2)
                        for i=2:cl_cycle_mc-1
                            while (Middle_mc(i).MCMarker_cl+1 <= Middle_mc(i-1).MCIndex_cl)
                                [hasNew_cmc, data_new, Middle_mc(i).MCLookForRepeat_cl, Middle_mc(i).MCNumRepeat_cl, Middle_mc(i).MCMaxAmp_cl, Middle_mc(i).MCMarker_cl] = rt_MotCompsCleanUp(Middle_mc(i-1).MCData_cl,  Middle_mc(i).MCLookForRepeat_cl, Middle_mc(i).MCNumRepeat_cl, Middle_mc(i).MCMaxAmp_cl, Middle_mc(i).MCMarker_cl);
                                if (hasNew_cmc)
                                    % Increase counter
                                    Middle_mc(i).MCIndex_cl = Middle_mc(i).MCIndex_cl+1;
                                    % Keep history of MC layer clean up data
                                    Middle_mc(i).MCData_cl(Middle_mc(i).MCIndex_cl,:) = data_new;
                                end
                            end
                        end
                    end
                    
                    % The last cycle
                    while (Result.MCMarker_cl+1 <= Middle_mc(cl_cycle_mc-1).MCIndex_cl)
                        [hasNew_cmc, data_new, Result.MCLookForRepeat_cl, Result.MCNumRepeat_cl, Result.MCMaxAmp_cl, Result.MCMarker_cl] = rt_MotCompsCleanUp(Middle_mc(cl_cycle_mc-1).MCData_cl,  Result.MCLookForRepeat_cl, Result.MCNumRepeat_cl, Result.MCMaxAmp_cl, Result.MCMarker_cl);
                        if (hasNew_cmc)       
                            % Increase counter
                            Result.MCIndex_cl = Result.MCIndex_cl+1;
                            % Keep history of MC layer clean up data
                            Result.MCData_cl(Result.MCIndex_cl,:) = data_new;    
                            
                            % Publish the new item to post processing
                            compositeMsg.McLabel     = rt_actionInt2actionLbl(data_new(1));
                            compositeMsg.AverageVal  = data_new(2);
                            compositeMsg.RmsVal      = data_new(3);
                            compositeMsg.AmpVal      = data_new(4);
                            compositeMsg.GradLabel1  = gradInt2gradLbl(data_new(5));
                            compositeMsg.GradLabel2  = gradInt2gradLbl(data_new(6));
                            compositeMsg.T1Start     = data_new(7);
                            compositeMsg.T1End       = data_new(8);
                            compositeMsg.T2Start     = data_new(9);
                            compositeMsg.T2End       = data_new(10);
                            compositeMsg.TAverage    = data_new(11);
                            send(compositePub,compositeMsg);
                            %                         send(compositePubConstant.Value,compositeMsg);
                        end
                    end
 
                else
                    % The only cycle of composite layer clean up
                    while (Result.MCMarker_cl+1 <= Result.MCIndex)
                        [hasNew_cmc, data_new, Result.MCLookForRepeat_cl, Result.MCNumRepeat_cl, Result.MCMaxAmp_cl, Result.MCMarker_cl] = rt_MotCompsCleanUp(Result.MCData,  Result.MCLookForRepeat_cl, Result.MCNumRepeat_cl, Result.MCMaxAmp_cl, Result.MCMarker_cl);
                        if (hasNew_cmc)
                            
                            % Increase counter
                            Result.MCIndex_cl = Result.MCIndex_cl+1;
                            % Keep history of MC layer clean up data
                            Result.MCData_cl(Result.MCIndex_cl,:) = data_new;
                            
                            % Publish the new item to post processing
                            compositeMsg.McLabel     = rt_actionInt2actionLbl(data_new(1));
                            compositeMsg.AverageVal  = data_new(2);
                            compositeMsg.RmsVal      = data_new(3);
                            compositeMsg.AmpVal      = data_new(4);
                            compositeMsg.GradLabel1  = gradInt2gradLbl(data_new(5));
                            compositeMsg.GradLabel2  = gradInt2gradLbl(data_new(6));
                            compositeMsg.T1Start     = data_new(7);
                            compositeMsg.T1End       = data_new(8);
                            compositeMsg.T2Start     = data_new(9);
                            compositeMsg.T2End       = data_new(10);
                            compositeMsg.TAverage    = data_new(11);
                            send(compositePub,compositeMsg);
                            %                         send(compositePubConstant.Value,compositeMsg);
                        end
                    end
                end
                
                
                %% Low layer behavior layer
                while (Result.llbehMarker+1 <= Result.MCIndex_cl)
                    [hasNew_llb, data_new, Result.llbehMarker] = rt_llbehComposition(Result.MCData_cl, Result.llbehMarker, 0);
                    if (hasNew_llb)
                        % Increase counter
                        Result.llbehIndex = Result.llbehIndex+1;
                        % Keep history of MC layer data
                        Result.llbehData(Result.llbehIndex,:) = data_new;
                    end
                end
                
                
                %% Low layer behavior layer clean up
                if(cl_cycle_llbeh>1)
                    % Multiple cycles
                    
                    % The first cycle
                    while (Middle_llbeh(1).llbehMarker_cl+1 <= Result.llbehIndex)
                        [hasNew_llbc, data_new, Middle_llbeh(1).llbehLookForRepeat_cl, Middle_llbeh(1).llbehNumRepeat_cl, Middle_llbeh(1).llbehMarker_cl] = rt_llbehCompositionCleanUp(Result.llbehData,  Middle_llbeh(1).llbehLookForRepeat_cl, Middle_llbeh(1).llbehNumRepeat_cl, Middle_llbeh(1).llbehMarker_cl);
                        if (hasNew_llbc)
                            
                            % Increase counter
                            Middle_llbeh(1).llbehIndex_cl = Middle_llbeh(1).llbehIndex_cl+1;
                            % Keep history of MC layer clean up data
                            Middle_llbeh(1).llbehData_cl(Middle_llbeh(1).llbehIndex_cl,:) = data_new;
                        end
                    end
                    
                    % The middle cycles
                    if(cl_cycle_llbeh>2)
                       for i=2:cl_cycle_llbeh-1
                           while (Middle_llbeh(i).llbehMarker_cl+1 <= Middle_llbeh(i-1).llbehIndex_cl)
                               [hasNew_llbc, data_new, Middle_llbeh(i).llbehLookForRepeat_cl, Middle_llbeh(i).llbehNumRepeat_cl, Middle_llbeh(i).llbehMarker_cl] = rt_llbehCompositionCleanUp(Middle_llbeh(i-1).llbehData_cl,  Middle_llbeh(i).llbehLookForRepeat_cl, Middle_llbeh(i).llbehNumRepeat_cl, Middle_llbeh(i).llbehMarker_cl);
                               if (hasNew_llbc)
                                   
                                   % Increase counter
                                   Middle_llbeh(i).llbehIndex_cl = Middle_llbeh(i).llbehIndex_cl+1;
                                   % Keep history of MC layer clean up data
                                   Middle_llbeh(i).llbehData_cl(Middle_llbeh(i).llbehIndex_cl,:) = data_new;
                               end
                           end      
                       end
                    end
                    
                    % The last cycle
                    while (Result.llbehMarker_cl+1 <= Middle_llbeh(cl_cycle_llbeh-1).llbehIndex_cl)
                        [hasNew_llbc, data_new, Result.llbehLookForRepeat_cl, Result.llbehNumRepeat_cl, Result.llbehMarker_cl] = rt_llbehCompositionCleanUp(Middle_llbeh(cl_cycle_llbeh-1).llbehData_cl,  Result.llbehLookForRepeat_cl, Result.llbehNumRepeat_cl, Result.llbehMarker_cl);
                        if (hasNew_llbc)
                            
                            % Increase counter
                            Result.llbehIndex_cl = Result.llbehIndex_cl+1;
                            % Keep history of MC layer clean up data
                            Result.llbehData_cl(Result.llbehIndex_cl,:) = data_new;
                            
                            % Publish the new item to post processing
                            llbehaviorMsg.LlbLabel        = llbInt2llbLbl(data_new(1));
                            llbehaviorMsg.AverageVal1     = data_new(2);
                            llbehaviorMsg.AverageVal2     = data_new(3);
                            llbehaviorMsg.AvgMagVal       = data_new(4);
                            llbehaviorMsg.RmsVal1         = data_new(5);
                            llbehaviorMsg.RmsVal2         = data_new(6);
                            llbehaviorMsg.AvgRmsVal       = data_new(7);
                            llbehaviorMsg.AmplitudeVal1   = data_new(8);
                            llbehaviorMsg.AmplitudeVal2   = data_new(9);
                            llbehaviorMsg.AvgAmpVal       = data_new(10);
                            llbehaviorMsg.McLabel1        = rt_actionInt2actionLbl(data_new(11));
                            llbehaviorMsg.McLabel2        = rt_actionInt2actionLbl(data_new(12));
                            llbehaviorMsg.T1Start         = data_new(13);
                            llbehaviorMsg.T1End           = data_new(14);
                            llbehaviorMsg.T2Start         = data_new(15);
                            llbehaviorMsg.T2End           = data_new(16);
                            llbehaviorMsg.TAverage        = data_new(17);
                            send(llbehaviorPub,llbehaviorMsg);
                            %                         send(llbehaviorPubConstant.Value,llbehaviorMsg);
                        end
                    end
                else
                    % The only cycle
                    while (Result.llbehMarker_cl+1 <= Result.llbehIndex)
                        [hasNew_llbc, data_new, Result.llbehLookForRepeat_cl, Result.llbehNumRepeat_cl, Result.llbehMarker_cl] = rt_llbehCompositionCleanUp(Result.llbehData,  Result.llbehLookForRepeat_cl, Result.llbehNumRepeat_cl, Result.llbehMarker_cl);
                        if (hasNew_llbc)
                            
                            % Increase counter
                            Result.llbehIndex_cl = Result.llbehIndex_cl+1;
                            % Keep history of MC layer clean up data
                            Result.llbehData_cl(Result.llbehIndex_cl,:) = data_new;
                            
                            % Publish the new item to post processing
                            llbehaviorMsg.LlbLabel        = llbInt2llbLbl(data_new(1));
                            llbehaviorMsg.AverageVal1     = data_new(2);
                            llbehaviorMsg.AverageVal2     = data_new(3);
                            llbehaviorMsg.AvgMagVal       = data_new(4);
                            llbehaviorMsg.RmsVal1         = data_new(5);
                            llbehaviorMsg.RmsVal2         = data_new(6);
                            llbehaviorMsg.AvgRmsVal       = data_new(7);
                            llbehaviorMsg.AmplitudeVal1   = data_new(8);
                            llbehaviorMsg.AmplitudeVal2   = data_new(9);
                            llbehaviorMsg.AvgAmpVal       = data_new(10);
                            llbehaviorMsg.McLabel1        = rt_actionInt2actionLbl(data_new(11));
                            llbehaviorMsg.McLabel2        = rt_actionInt2actionLbl(data_new(12));
                            llbehaviorMsg.T1Start         = data_new(13);
                            llbehaviorMsg.T1End           = data_new(14);
                            llbehaviorMsg.T2Start         = data_new(15);
                            llbehaviorMsg.T2End           = data_new(16);
                            llbehaviorMsg.TAverage        = data_new(17);
                            send(llbehaviorPub,llbehaviorMsg);
                            %                         send(llbehaviorPubConstant.Value,llbehaviorMsg);
                        end
                    end
                    
                end
                
            end
        end
        
        
        
        if(breakSignal)
            
            %% Last iteration of primitive layer
            %  In the last iteration of primitive layer, just 1 new item will certainly be appended to primitive layer data
            wFinish     = localIndex;                            % Set to the last index of statData (the primitives space)
            Range       = Result.primiWStart:wFinish;               % Save from wStart to the immediately preceeding index that passed the threshold
            Time        = Result.Wren_loc(Range,1);          % Time indeces that we are working with
            Data        = Result.Wren_loc(Range,axisIndex+1); % Corresponding force data for a given force element in a given window
            
            polyCoeffs  = polyfit(Time,Data,1);            % First-order fit
            dataFit     = polyval(polyCoeffs, Time);
            
            [dAvg,dMax,dMin,dStart,dFinish,dGradient,dLabel]=rt_statisticalData(Time(1),Time(length(Time)),dataFit,polyCoeffs,FolderName,StrategyType,axisIndex); % 1+windowlength
            
            Result.primiIndex  = Result.primiIndex +1;
            
            Result.primiData(Result.primiIndex ,:) = [dAvg dMax dMin dStart dFinish dGradient dLabel];
            
            
            %% Last iteration of primitive layer clean up
            %  In the last iteration of primitive layer clean up, 1 or 2 new items will certainly be appended to primitive layer clean up data
            if(cl_cycle_primi>1)
                
                % The first cycle
                while (Middle_primi(1).primiMarker_cl+1 <= Result.primiIndex )
                    [hasNew_pc,data_new, Middle_primi(1).primiLookForRepeat_cl, Middle_primi(1).primiNumRepeat_cl, Middle_primi(1).primiMarker_cl] = rt_primitivesCleanUp(Result.primiData, Middle_primi(1).primiLookForRepeat_cl, Middle_primi(1).primiNumRepeat_cl, Middle_primi(1).primiMarker_cl);
                    if (hasNew_pc)
                        % Increase counter
                        Middle_primi(1).primiIndex_cl = Middle_primi(1).primiIndex_cl+1;
                        % Keep history of primitive layer clean up data
                        Middle_primi(1).primiData_cl(Middle_primi(1).primiIndex_cl,:) = data_new;
                    end
                end
                
                if (Middle_primi(1).primiMarker_cl == Result.primiIndex )
                    if (Middle_primi(1).primiLookForRepeat_cl) % if looking for repeat
                        nextPrimitive   = Middle_primi(1).primiNumRepeat_cl;
                        startPrimitive  = Middle_primi(1).primiMarker_cl-Middle_primi(1).primiNumRepeat_cl;
                        data_new = rt_MergePrimitives(startPrimitive,Result.primiData,nextPrimitive);
                        Middle_primi(1).primiIndex_cl = Middle_primi(1).primiIndex_cl+1;
                        Middle_primi(1).primiData_cl(Middle_primi(1).primiIndex_cl,:) = data_new;
                    else
                        data_new = Result.primiData(Middle_primi(1).primiMarker_cl,:);
                        Middle_primi(1).primiIndex_cl = Middle_primi(1).primiIndex_cl+1;
                        Middle_primi(1).primiData_cl(Middle_primi(1).primiIndex_cl,:) = data_new;
                    end
                end
                
                % The middle cycles
                if(cl_cycle_primi>2)
                    for i=2:cl_cycle_primi-1
                        
                        while (Middle_primi(i).primiMarker_cl+1 <= Middle_primi(i-1).primiIndex_cl )
                            [hasNew_pc,data_new, Middle_primi(i).primiLookForRepeat_cl, Middle_primi(i).primiNumRepeat_cl, Middle_primi(i).primiMarker_cl] = rt_primitivesCleanUp(Middle_primi(i-1).primiData_cl, Middle_primi(i).primiLookForRepeat_cl, Middle_primi(i).primiNumRepeat_cl, Middle_primi(i).primiMarker_cl);
                            if (hasNew_pc)
                                % Increase counter
                                Middle_primi(i).primiIndex_cl = Middle_primi(i).primiIndex_cl+1;
                                % Keep history of primitive layer clean up data
                                Middle_primi(i).primiData_cl(Middle_primi(i).primiIndex_cl,:) = data_new;
                            end
                        end
                        
                        if (Middle_primi(i).primiMarker_cl == Middle_primi(i-1).primiIndex_cl )
                            if (Middle_primi(i).primiLookForRepeat_cl) % if looking for repeat
                                nextPrimitive   = Middle_primi(i).primiNumRepeat_cl;
                                startPrimitive  = Middle_primi(i).primiMarker_cl-Middle_primi(i).primiNumRepeat_cl;
                                data_new = rt_MergePrimitives(startPrimitive,Middle_primi(i-1).primiData_cl,nextPrimitive);
                                Middle_primi(i).primiIndex_cl = Middle_primi(i).primiIndex_cl+1;
                                Middle_primi(i).primiData_cl(Middle_primi(i).primiIndex_cl,:) = data_new;
                            else
                                data_new = Middle_primi(i-1).primiData_cl(Middle_primi(i).primiMarker_cl,:);
                                Middle_primi(i).primiIndex_cl = Middle_primi(i).primiIndex_cl+1;
                                Middle_primi(i).primiData_cl(Middle_primi(i).primiIndex_cl,:) = data_new;
                            end
                        end
                        
                    end
                end
                
                % The last cycle
                while (Result.primiMarker_cl+1 <= Middle_primi(cl_cycle_primi-1).primiIndex_cl )
                    [hasNew_pc,data_new, Result.primiLookForRepeat_cl, Result.primiNumRepeat_cl, Result.primiMarker_cl] = rt_primitivesCleanUp(Middle_primi(cl_cycle_primi-1).primiData_cl, Result.primiLookForRepeat_cl, Result.primiNumRepeat_cl, Result.primiMarker_cl);
                    if (hasNew_pc)
                        % Increase counter
                        Result.primiIndex_cl = Result.primiIndex_cl+1;
                        % Keep history of primitive layer clean up data
                        Result.primiData_cl(Result.primiIndex_cl,:) = data_new;
                        
                        % Publish the new item to post processing
                        %        fprintf('Reach here segment\n');
                        segmentMsg.Average   = data_new(1);
                        segmentMsg.MaxVal    = data_new(2);
                        segmentMsg.MinVal    = data_new(3);
                        segmentMsg.TStart    = data_new(4);
                        segmentMsg.TEnd      = data_new(5);
                        segmentMsg.Gradient  = data_new(6);
                        segmentMsg.GradLabel = gradInt2gradLbl(data_new(7));
                        send(segmentPub,segmentMsg);
                        %                         send(segmentPubConstant.Value,segmentMsg);
                    end
                end
                
                if (Result.primiMarker_cl == Middle_primi(cl_cycle_primi-1).primiIndex_cl )
                    if (Result.primiLookForRepeat_cl) % if looking for repeat
                        nextPrimitive   = Result.primiNumRepeat_cl;
                        startPrimitive  = Result.primiMarker_cl-Result.primiNumRepeat_cl;
                        data_new = rt_MergePrimitives(startPrimitive,Middle_primi(cl_cycle_primi-1).primiData_cl,nextPrimitive);
                        Result.primiIndex_cl = Result.primiIndex_cl+1;
                        Result.primiData_cl(Result.primiIndex_cl,:) = data_new;
                        
                        % Publish the new item to post processing
                        %        fprintf('Reach here segment\n');
                        segmentMsg.Average   = data_new(1);
                        segmentMsg.MaxVal    = data_new(2);
                        segmentMsg.MinVal    = data_new(3);
                        segmentMsg.TStart    = data_new(4);
                        segmentMsg.TEnd      = data_new(5);
                        segmentMsg.Gradient  = data_new(6);
                        segmentMsg.GradLabel = gradInt2gradLbl(data_new(7));
                        send(segmentPub,segmentMsg);
                        %                         send(segmentPubConstant.Value,segmentMsg);
                    else
                        data_new = Middle_primi(cl_cycle_primi-1).primiData_cl(Result.primiMarker_cl,:);
                        Result.primiIndex_cl = Result.primiIndex_cl+1;
                        Result.primiData_cl(Result.primiIndex_cl,:) = data_new;
                        
                        % Publish the new item to post processing
                        %        fprintf('Reach here segment\n');
                        segmentMsg.Average   = data_new(1);
                        segmentMsg.MaxVal    = data_new(2);
                        segmentMsg.MinVal    = data_new(3);
                        segmentMsg.TStart    = data_new(4);
                        segmentMsg.TEnd      = data_new(5);
                        segmentMsg.Gradient  = data_new(6);
                        segmentMsg.GradLabel = gradInt2gradLbl(data_new(7));
                        send(segmentPub,segmentMsg);
                        %                         send(segmentPubConstant.Value,segmentMsg);
                    end
                end
            else
                while (Result.primiMarker_cl+1 <= Result.primiIndex )
                    [hasNew_pc,data_new, Result.primiLookForRepeat_cl, Result.primiNumRepeat_cl, Result.primiMarker_cl] = rt_primitivesCleanUp(Result.primiData, Result.primiLookForRepeat_cl, Result.primiNumRepeat_cl, Result.primiMarker_cl);
                    if (hasNew_pc)
                        % Increase counter
                        Result.primiIndex_cl = Result.primiIndex_cl+1;
                        % Keep history of primitive layer clean up data
                        Result.primiData_cl(Result.primiIndex_cl,:) = data_new;
                        
                        % Publish the new item to post processing
                        %        fprintf('Reach here segment\n');
                        segmentMsg.Average   = data_new(1);
                        segmentMsg.MaxVal    = data_new(2);
                        segmentMsg.MinVal    = data_new(3);
                        segmentMsg.TStart    = data_new(4);
                        segmentMsg.TEnd      = data_new(5);
                        segmentMsg.Gradient  = data_new(6);
                        segmentMsg.GradLabel = gradInt2gradLbl(data_new(7));
                        send(segmentPub,segmentMsg);
                        %                         send(segmentPubConstant.Value,segmentMsg);
                    end
                end
                
                if (Result.primiMarker_cl == Result.primiIndex )
                    if (Result.primiLookForRepeat_cl) % if looking for repeat
                        nextPrimitive   = Result.primiNumRepeat_cl;
                        startPrimitive  = Result.primiMarker_cl-Result.primiNumRepeat_cl;
                        data_new = rt_MergePrimitives(startPrimitive,Result.primiData,nextPrimitive);
                        Result.primiIndex_cl = Result.primiIndex_cl+1;
                        Result.primiData_cl(Result.primiIndex_cl,:) = data_new;
                        
                        % Publish the new item to post processing
                        %        fprintf('Reach here segment\n');
                        segmentMsg.Average   = data_new(1);
                        segmentMsg.MaxVal    = data_new(2);
                        segmentMsg.MinVal    = data_new(3);
                        segmentMsg.TStart    = data_new(4);
                        segmentMsg.TEnd      = data_new(5);
                        segmentMsg.Gradient  = data_new(6);
                        segmentMsg.GradLabel = gradInt2gradLbl(data_new(7));
                        send(segmentPub,segmentMsg);
                        %                         send(segmentPubConstant.Value,segmentMsg);
                    else
                        data_new = Result.primiData(Result.primiMarker_cl,:);
                        Result.primiIndex_cl = Result.primiIndex_cl+1;
                        Result.primiData_cl(Result.primiIndex_cl,:) = data_new;
                        
                        % Publish the new item to post processing
                        %        fprintf('Reach here segment\n');
                        segmentMsg.Average   = data_new(1);
                        segmentMsg.MaxVal    = data_new(2);
                        segmentMsg.MinVal    = data_new(3);
                        segmentMsg.TStart    = data_new(4);
                        segmentMsg.TEnd      = data_new(5);
                        segmentMsg.Gradient  = data_new(6);
                        segmentMsg.GradLabel = gradInt2gradLbl(data_new(7));
                        send(segmentPub,segmentMsg);
                        %                         send(segmentPubConstant.Value,segmentMsg);
                    end
                end
            end
            
            
            
            
            %% Last iteration of compound motion layer
            while (Result.MCMarker+1 <= Result.primiIndex_cl)
                [hasNew_cm, data_new, Result.MCMarker] = rt_CompoundMotionComposition(Result.primiData_cl, Result.MCMarker, 0);
                if (hasNew_cm)
                    % Increase counter
                    Result.MCIndex = Result.MCIndex+1;
                    Result.MCData(Result.MCIndex,:) = data_new;
                end
            end
            
            if (Result.MCMarker == Result.primiIndex_cl)
                [hasNew_cm, data_new, Result.MCMarker] = rt_CompoundMotionComposition(Result.primiData_cl, Result.MCMarker, 1);
                if (hasNew_cm)
                    % Increase counter
                    Result.MCIndex = Result.MCIndex+1;
                    Result.MCData(Result.MCIndex,:) = data_new;
                end
            end
            
            
            %% Last iteration of compound motion layer clean up
            
            if(cl_cycle_mc>1)
                % Multiple cycles 
               
                % The first cycle
                while (Middle_mc(1).MCMarker_cl+1 <= Result.MCIndex)
                    [hasNew_cmc, data_new, Middle_mc(1).MCLookForRepeat_cl, Middle_mc(1).MCNumRepeat_cl, Middle_mc(1).MCMaxAmp_cl, Middle_mc(1).MCMarker_cl] = rt_MotCompsCleanUp(Result.MCData,  Middle_mc(1).MCLookForRepeat_cl, Middle_mc(1).MCNumRepeat_cl, Middle_mc(1).MCMaxAmp_cl, Middle_mc(1).MCMarker_cl);
                    if (hasNew_cmc)
                        % Increase counter
                        Middle_mc(1).MCIndex_cl = Middle_mc(1).MCIndex_cl+1;
                        % Keep history of MC layer clean up data
                        Middle_mc(1).MCData_cl(Middle_mc(1).MCIndex_cl,:) = data_new;
                    end
                end
                
                if (Middle_mc(1).MCMarker_cl == Result.MCIndex)
                    if (Middle_mc(1).MCLookForRepeat_cl) % if looking for repeat
                        nextPrimitive   = Middle_mc(1).MCNumRepeat_cl;
                        startPrimitive  = Middle_mc(1).MCMarker_cl-Middle_mc(1).MCNumRepeat_cl;
                        repeat_Lbl      = Result.MCData(startPrimitive,mc_ACTN_LBL);
                        data_new        = rt_MergeCompositions(startPrimitive,Result.MCData,actionLbl,repeat_Lbl,nextPrimitive);
                        Middle_mc(1).MCIndex_cl = Middle_mc(1).MCIndex_cl+1;
                        Middle_mc(1).MCData_cl(Middle_mc(1).MCIndex_cl,:) = data_new;
                    else
                        data_new        = Result.MCData(Middle_mc(1).MCMarker_cl,:);
                        Middle_mc(1).MCIndex_cl = Middle_mc(1).MCIndex_cl+1;
                        Middle_mc(1).MCData_cl(Middle_mc(1).MCIndex_cl,:) = data_new;
                    end
                end
                
                % The middle cycle
                if(cl_cycle_mc>2)
                    for i=2:cl_cycle_mc-1
                        while (Middle_mc(i).MCMarker_cl+1 <= Middle_mc(i-1).MCIndex_cl)
                            [hasNew_cmc, data_new, Middle_mc(i).MCLookForRepeat_cl, Middle_mc(i).MCNumRepeat_cl, Middle_mc(i).MCMaxAmp_cl, Middle_mc(i).MCMarker_cl] = rt_MotCompsCleanUp(Middle_mc(i-1).MCData_cl,  Middle_mc(i).MCLookForRepeat_cl, Middle_mc(i).MCNumRepeat_cl, Middle_mc(i).MCMaxAmp_cl, Middle_mc(i).MCMarker_cl);
                            if (hasNew_cmc)
                                % Increase counter
                                Middle_mc(i).MCIndex_cl = Middle_mc(i).MCIndex_cl+1;
                                % Keep history of MC layer clean up data
                                Middle_mc(i).MCData_cl(Middle_mc(i).MCIndex_cl,:) = data_new;
                            end
                        end
                        
                        if (Middle_mc(i).MCMarker_cl == Middle_mc(i-1).MCIndex_cl)
                            if (Middle_mc(i).MCLookForRepeat_cl) % if looking for repeat
                                nextPrimitive   = Middle_mc(i).MCNumRepeat_cl;
                                startPrimitive  = Middle_mc(i).MCMarker_cl-Middle_mc(i).MCNumRepeat_cl;
                                repeat_Lbl      = Middle_mc(i-1).MCData_cl(startPrimitive,mc_ACTN_LBL);
                                data_new        = rt_MergeCompositions(startPrimitive,Middle_mc(i-1).MCData_cl,actionLbl,repeat_Lbl,nextPrimitive);
                                Middle_mc(i).MCIndex_cl = Middle_mc(i).MCIndex_cl+1;
                                Middle_mc(i).MCData_cl(Middle_mc(i).MCIndex_cl,:) = data_new;
                            else
                                data_new        = Middle_mc(i-1).MCData_cl(Middle_mc(i).MCMarker_cl,:);
                                Middle_mc(i).MCIndex_cl = Middle_mc(i).MCIndex_cl+1;
                                Middle_mc(i).MCData_cl(Middle_mc(i).MCIndex_cl,:) = data_new;
                            end
                        end
                    end
                end
                
                
                % The last cycle
                while (Result.MCMarker_cl+1 <= Middle_mc(cl_cycle_mc-1).MCIndex_cl)
                    [hasNew_cmc, data_new, Result.MCLookForRepeat_cl, Result.MCNumRepeat_cl, Result.MCMaxAmp_cl, Result.MCMarker_cl] = rt_MotCompsCleanUp(Middle_mc(cl_cycle_mc-1).MCData_cl,  Result.MCLookForRepeat_cl, Result.MCNumRepeat_cl, Result.MCMaxAmp_cl, Result.MCMarker_cl);
                    if (hasNew_cmc)
                        % Increase counter
                        Result.MCIndex_cl = Result.MCIndex_cl+1;
                        % Keep history of MC layer clean up data
                        Result.MCData_cl(Result.MCIndex_cl,:) = data_new;
                        
                        % Publish the new item to post processing
                        compositeMsg.McLabel     = rt_actionInt2actionLbl(data_new(1));
                        compositeMsg.AverageVal  = data_new(2);
                        compositeMsg.RmsVal      = data_new(3);
                        compositeMsg.AmpVal      = data_new(4);
                        compositeMsg.GradLabel1  = gradInt2gradLbl(data_new(5));
                        compositeMsg.GradLabel2  = gradInt2gradLbl(data_new(6));
                        compositeMsg.T1Start     = data_new(7);
                        compositeMsg.T1End       = data_new(8);
                        compositeMsg.T2Start     = data_new(9);
                        compositeMsg.T2End       = data_new(10);
                        compositeMsg.TAverage    = data_new(11);
                        send(compositePub,compositeMsg);
                        %                         send(compositePubConstant.Value,compositeMsg);
                    end
                end
                
                if (Result.MCMarker_cl == Middle_mc(cl_cycle_mc-1).MCIndex_cl)
                    if (Result.MCLookForRepeat_cl) % if looking for repeat
                        nextPrimitive   = Result.MCNumRepeat_cl;
                        startPrimitive  = Result.MCMarker_cl-Result.MCNumRepeat_cl;
                        repeat_Lbl      = Middle_mc(cl_cycle_mc-1).MCData_cl(startPrimitive,mc_ACTN_LBL);
                        data_new        = rt_MergeCompositions(startPrimitive,Middle_mc(cl_cycle_mc-1).MCData_cl,actionLbl,repeat_Lbl,nextPrimitive);
                        Result.MCIndex_cl = Result.MCIndex_cl+1;
                        Result.MCData_cl(Result.MCIndex_cl,:) = data_new;
                        
                        % Publish the new item to post processing
                        compositeMsg.McLabel     = rt_actionInt2actionLbl(data_new(1));
                        compositeMsg.AverageVal  = data_new(2);
                        compositeMsg.RmsVal      = data_new(3);
                        compositeMsg.AmpVal      = data_new(4);
                        compositeMsg.GradLabel1  = gradInt2gradLbl(data_new(5));
                        compositeMsg.GradLabel2  = gradInt2gradLbl(data_new(6));
                        compositeMsg.T1Start     = data_new(7);
                        compositeMsg.T1End       = data_new(8);
                        compositeMsg.T2Start     = data_new(9);
                        compositeMsg.T2End       = data_new(10);
                        compositeMsg.TAverage    = data_new(11);
                        send(compositePub,compositeMsg);
                        %                         send(compositePubConstant.Value,compositeMsg);
                    else
                        data_new        = Middle_mc(cl_cycle_mc-1).MCData_cl(Result.MCMarker_cl,:);
                        Result.MCIndex_cl = Result.MCIndex_cl+1;
                        Result.MCData_cl(Result.MCIndex_cl,:) = data_new;
                        
                        % Publish the new item to post processing
                        compositeMsg.McLabel     = rt_actionInt2actionLbl(data_new(1));
                        compositeMsg.AverageVal  = data_new(2);
                        compositeMsg.RmsVal      = data_new(3);
                        compositeMsg.AmpVal      = data_new(4);
                        compositeMsg.GradLabel1  = gradInt2gradLbl(data_new(5));
                        compositeMsg.GradLabel2  = gradInt2gradLbl(data_new(6));
                        compositeMsg.T1Start     = data_new(7);
                        compositeMsg.T1End       = data_new(8);
                        compositeMsg.T2Start     = data_new(9);
                        compositeMsg.T2End       = data_new(10);
                        compositeMsg.TAverage    = data_new(11);
                        send(compositePub,compositeMsg);
                        %                         send(compositePubConstant.Value,compositeMsg);
                    end
                end
                
                
            else
                % The only cycle
                while (Result.MCMarker_cl+1 <= Result.MCIndex)
                    [hasNew_cmc, data_new, Result.MCLookForRepeat_cl, Result.MCNumRepeat_cl, Result.MCMaxAmp_cl, Result.MCMarker_cl] = rt_MotCompsCleanUp(Result.MCData,  Result.MCLookForRepeat_cl, Result.MCNumRepeat_cl, Result.MCMaxAmp_cl, Result.MCMarker_cl);
                    if (hasNew_cmc)
                        % Increase counter
                        Result.MCIndex_cl = Result.MCIndex_cl+1;
                        % Keep history of MC layer clean up data
                        Result.MCData_cl(Result.MCIndex_cl,:) = data_new;
                        
                        % Publish the new item to post processing
                        compositeMsg.McLabel     = rt_actionInt2actionLbl(data_new(1));
                        compositeMsg.AverageVal  = data_new(2);
                        compositeMsg.RmsVal      = data_new(3);
                        compositeMsg.AmpVal      = data_new(4);
                        compositeMsg.GradLabel1  = gradInt2gradLbl(data_new(5));
                        compositeMsg.GradLabel2  = gradInt2gradLbl(data_new(6));
                        compositeMsg.T1Start     = data_new(7);
                        compositeMsg.T1End       = data_new(8);
                        compositeMsg.T2Start     = data_new(9);
                        compositeMsg.T2End       = data_new(10);
                        compositeMsg.TAverage    = data_new(11);
                        send(compositePub,compositeMsg);
                        %                         send(compositePubConstant.Value,compositeMsg);
                    end
                end
                
                if (Result.MCMarker_cl == Result.MCIndex)
                    if (Result.MCLookForRepeat_cl) % if looking for repeat
                        nextPrimitive   = Result.MCNumRepeat_cl;
                        startPrimitive  = Result.MCMarker_cl-Result.MCNumRepeat_cl;
                        repeat_Lbl      = Result.MCData(startPrimitive,mc_ACTN_LBL);
                        data_new        = rt_MergeCompositions(startPrimitive,Result.MCData,actionLbl,repeat_Lbl,nextPrimitive);
                        Result.MCIndex_cl = Result.MCIndex_cl+1;
                        Result.MCData_cl(Result.MCIndex_cl,:) = data_new;
                        
                        % Publish the new item to post processing
                        compositeMsg.McLabel     = rt_actionInt2actionLbl(data_new(1));
                        compositeMsg.AverageVal  = data_new(2);
                        compositeMsg.RmsVal      = data_new(3);
                        compositeMsg.AmpVal      = data_new(4);
                        compositeMsg.GradLabel1  = gradInt2gradLbl(data_new(5));
                        compositeMsg.GradLabel2  = gradInt2gradLbl(data_new(6));
                        compositeMsg.T1Start     = data_new(7);
                        compositeMsg.T1End       = data_new(8);
                        compositeMsg.T2Start     = data_new(9);
                        compositeMsg.T2End       = data_new(10);
                        compositeMsg.TAverage    = data_new(11);
                        send(compositePub,compositeMsg);
                        %                         send(compositePubConstant.Value,compositeMsg);
                    else
                        data_new        = Result.MCData(Result.MCMarker_cl,:);
                        Result.MCIndex_cl = Result.MCIndex_cl+1;
                        Result.MCData_cl(Result.MCIndex_cl,:) = data_new;
                        
                        % Publish the new item to post processing
                        compositeMsg.McLabel     = rt_actionInt2actionLbl(data_new(1));
                        compositeMsg.AverageVal  = data_new(2);
                        compositeMsg.RmsVal      = data_new(3);
                        compositeMsg.AmpVal      = data_new(4);
                        compositeMsg.GradLabel1  = gradInt2gradLbl(data_new(5));
                        compositeMsg.GradLabel2  = gradInt2gradLbl(data_new(6));
                        compositeMsg.T1Start     = data_new(7);
                        compositeMsg.T1End       = data_new(8);
                        compositeMsg.T2Start     = data_new(9);
                        compositeMsg.T2End       = data_new(10);
                        compositeMsg.TAverage    = data_new(11);
                        send(compositePub,compositeMsg);
                        %                         send(compositePubConstant.Value,compositeMsg);
                    end
                end
            end
            
            
            
            %% Last iteration of low layer behavior layer
            while (Result.llbehMarker+1 <= Result.MCIndex_cl)
                [hasNew_llb, data_new, Result.llbehMarker] = rt_llbehComposition(Result.MCData_cl, Result.llbehMarker, 0);
                if (hasNew_llb)
                    % Increase counter
                    Result.llbehIndex = Result.llbehIndex+1;
                    Result.llbehData(Result.llbehIndex,:) = data_new;
                end
            end
            
            if (Result.llbehMarker == Result.MCIndex_cl)
                [hasNew_llb, data_new, Result.llbehMarker] = rt_llbehComposition(Result.MCData_cl, Result.llbehMarker, 1);
                if (hasNew_llb)
                    % Increase counter
                    Result.llbehIndex = Result.llbehIndex+1;
                    Result.llbehData(Result.llbehIndex,:) = data_new;
                end
            end
            
            
            %% Last iteration of low layer behavior layer clean up
            if(cl_cycle_llbeh>1)
                % Multiple cycles
                
                % The first cycle
                while (Middle_llbeh(1).llbehMarker_cl+1 <= Result.llbehIndex)
                    [hasNew_llbc, data_new, Middle_llbeh(1).llbehLookForRepeat_cl, Middle_llbeh(1).llbehNumRepeat_cl, Middle_llbeh(1).llbehMarker_cl] = rt_llbehCompositionCleanUp(Result.llbehData,  Middle_llbeh(1).llbehLookForRepeat_cl, Middle_llbeh(1).llbehNumRepeat_cl, Middle_llbeh(1).llbehMarker_cl);
                    if (hasNew_llbc)
                        % Increase counter
                        Middle_llbeh(1).llbehIndex_cl = Middle_llbeh(1).llbehIndex_cl+1;
                        % Keep history of MC layer clean up data
                        Middle_llbeh(1).llbehData_cl(Middle_llbeh(1).llbehIndex_cl,:) = data_new;
                    end
                end
                
                if (Middle_llbeh(1).llbehMarker_cl == Result.llbehIndex)
                    if (Middle_llbeh(1).llbehLookForRepeat_cl) % if looking for repeat
                        nextPrimitive   = Middle_llbeh(1).llbehNumRepeat_cl;
                        startPrimitive  = Middle_llbeh(1).llbehMarker_cl-Middle_llbeh(1).llbehNumRepeat_cl;
                        repeat_Lbl      = Result.llbehData(startPrimitive,behLbl);
                        data_new        = rt_MergeLowLevelBehaviors(startPrimitive,Result.llbehData,llbehLbl,repeat_Lbl,nextPrimitive);
                        Middle_llbeh(1).llbehIndex_cl = Middle_llbeh(1).llbehIndex_cl+1;
                        Middle_llbeh(1).llbehData_cl(Middle_llbeh(1).llbehIndex_cl,:) = data_new;
                    else
                        data_new        = Result.llbehData(Middle_llbeh(1).llbehMarker_cl,:);
                        Middle_llbeh(1).llbehIndex_cl = Middle_llbeh(1).llbehIndex_cl+1;
                        Middle_llbeh(1).llbehData_cl(Middle_llbeh(1).llbehIndex_cl,:) = data_new;
                    end
                end
                
                % The middle cycles
                if(cl_cycle_llbeh>2)
                    for i=2:cl_cycle_llbeh-1
                        while (Middle_llbeh(i).llbehMarker_cl+1 <= Middle_llbeh(i-1).llbehIndex_cl)
                            [hasNew_llbc, data_new, Middle_llbeh(i).llbehLookForRepeat_cl, Middle_llbeh(i).llbehNumRepeat_cl, Middle_llbeh(i).llbehMarker_cl] = rt_llbehCompositionCleanUp(Middle_llbeh(i-1).llbehData_cl,  Middle_llbeh(i).llbehLookForRepeat_cl, Middle_llbeh(i).llbehNumRepeat_cl, Middle_llbeh(i).llbehMarker_cl);
                            if (hasNew_llbc)
                                % Increase counter
                                Middle_llbeh(i).llbehIndex_cl = Middle_llbeh(i).llbehIndex_cl+1;
                                % Keep history of MC layer clean up data
                                Middle_llbeh(i).llbehData_cl(Middle_llbeh(i).llbehIndex_cl,:) = data_new;
                            end
                        end
                        
                        if (Middle_llbeh(i).llbehMarker_cl == Middle_llbeh(i-1).llbehIndex_cl)
                            if (Middle_llbeh(i).llbehLookForRepeat_cl) % if looking for repeat
                                nextPrimitive   = Middle_llbeh(i).llbehNumRepeat_cl;
                                startPrimitive  = Middle_llbeh(i).llbehMarker_cl-Middle_llbeh(i).llbehNumRepeat_cl;
                                repeat_Lbl      = Middle_llbeh(i-1).llbehData_cl(startPrimitive,behLbl);
                                data_new        = rt_MergeLowLevelBehaviors(startPrimitive,Middle_llbeh(i-1).llbehData_cl,llbehLbl,repeat_Lbl,nextPrimitive);
                                Middle_llbeh(i).llbehIndex_cl = Middle_llbeh(i).llbehIndex_cl+1;
                                Middle_llbeh(i).llbehData_cl(Middle_llbeh(i).llbehIndex_cl,:) = data_new;
                            else
                                data_new        = Middle_llbeh(i-1).llbehData_cl(Middle_llbeh(i).llbehMarker_cl,:);
                                Middle_llbeh(i).llbehIndex_cl = Middle_llbeh(i).llbehIndex_cl+1;
                                Middle_llbeh(i).llbehData_cl(Middle_llbeh(i).llbehIndex_cl,:) = data_new;
                            end
                        end
                        
                    end
                end    
%                     
                    
                % The last cycle
                while (Result.llbehMarker_cl+1 <= Middle_llbeh(cl_cycle_llbeh-1).llbehIndex_cl)
                    [hasNew_llbc, data_new, Result.llbehLookForRepeat_cl, Result.llbehNumRepeat_cl, Result.llbehMarker_cl] = rt_llbehCompositionCleanUp(Middle_llbeh(cl_cycle_llbeh-1).llbehData_cl,  Result.llbehLookForRepeat_cl, Result.llbehNumRepeat_cl, Result.llbehMarker_cl);
                    if (hasNew_llbc)
                        % Increase counter
                        Result.llbehIndex_cl = Result.llbehIndex_cl+1;
                        % Keep history of MC layer clean up data
                        Result.llbehData_cl(Result.llbehIndex_cl,:) = data_new;
                        
                        % Publish the new item to post processing
                        llbehaviorMsg.LlbLabel        = llbInt2llbLbl(data_new(1));
                        llbehaviorMsg.AverageVal1     = data_new(2);
                        llbehaviorMsg.AverageVal2     = data_new(3);
                        llbehaviorMsg.AvgMagVal       = data_new(4);
                        llbehaviorMsg.RmsVal1         = data_new(5);
                        llbehaviorMsg.RmsVal2         = data_new(6);
                        llbehaviorMsg.AvgRmsVal       = data_new(7);
                        llbehaviorMsg.AmplitudeVal1   = data_new(8);
                        llbehaviorMsg.AmplitudeVal2   = data_new(9);
                        llbehaviorMsg.AvgAmpVal       = data_new(10);
                        llbehaviorMsg.McLabel1        = rt_actionInt2actionLbl(data_new(11));
                        llbehaviorMsg.McLabel2        = rt_actionInt2actionLbl(data_new(12));
                        llbehaviorMsg.T1Start         = data_new(13);
                        llbehaviorMsg.T1End           = data_new(14);
                        llbehaviorMsg.T2Start         = data_new(15);
                        llbehaviorMsg.T2End           = data_new(16);
                        llbehaviorMsg.TAverage        = data_new(17);
                        send(llbehaviorPub,llbehaviorMsg);
                        %                         send(llbehaviorPubConstant.Value,llbehaviorMsg);
                    end
                end
                
                if (Result.llbehMarker_cl == Middle_llbeh(cl_cycle_llbeh-1).llbehIndex_cl)
                    if (Result.llbehLookForRepeat_cl) % if looking for repeat
                        nextPrimitive   = Result.llbehNumRepeat_cl;
                        startPrimitive  = Result.llbehMarker_cl-Result.llbehNumRepeat_cl;
                        repeat_Lbl      = Middle_llbeh(cl_cycle_llbeh-1).llbehData_cl(startPrimitive,behLbl);
                        data_new        = rt_MergeLowLevelBehaviors(startPrimitive,Middle_llbeh(cl_cycle_llbeh-1).llbehData_cl,llbehLbl,repeat_Lbl,nextPrimitive);
                        Result.llbehIndex_cl = Result.llbehIndex_cl+1;
                        Result.llbehData_cl(Result.llbehIndex_cl,:) = data_new;
                        
                        % Publish the new item to post processing
                        llbehaviorMsg.LlbLabel        = llbInt2llbLbl(data_new(1));
                        llbehaviorMsg.AverageVal1     = data_new(2);
                        llbehaviorMsg.AverageVal2     = data_new(3);
                        llbehaviorMsg.AvgMagVal       = data_new(4);
                        llbehaviorMsg.RmsVal1         = data_new(5);
                        llbehaviorMsg.RmsVal2         = data_new(6);
                        llbehaviorMsg.AvgRmsVal       = data_new(7);
                        llbehaviorMsg.AmplitudeVal1   = data_new(8);
                        llbehaviorMsg.AmplitudeVal2   = data_new(9);
                        llbehaviorMsg.AvgAmpVal       = data_new(10);
                        llbehaviorMsg.McLabel1        = rt_actionInt2actionLbl(data_new(11));
                        llbehaviorMsg.McLabel2        = rt_actionInt2actionLbl(data_new(12));
                        llbehaviorMsg.T1Start         = data_new(13);
                        llbehaviorMsg.T1End           = data_new(14);
                        llbehaviorMsg.T2Start         = data_new(15);
                        llbehaviorMsg.T2End           = data_new(16);
                        llbehaviorMsg.TAverage        = data_new(17);
                        send(llbehaviorPub,llbehaviorMsg);
                        %                         send(llbehaviorPubConstant.Value,llbehaviorMsg);
                    else
                        data_new        = Middle_llbeh(cl_cycle_llbeh-1).llbehData_cl(Result.llbehMarker_cl,:);
                        Result.llbehIndex_cl = Result.llbehIndex_cl+1;
                        Result.llbehData_cl(Result.llbehIndex_cl,:) = data_new;
                        
                        % Publish the new item to post processing
                        llbehaviorMsg.LlbLabel        = llbInt2llbLbl(data_new(1));
                        llbehaviorMsg.AverageVal1     = data_new(2);
                        llbehaviorMsg.AverageVal2     = data_new(3);
                        llbehaviorMsg.AvgMagVal       = data_new(4);
                        llbehaviorMsg.RmsVal1         = data_new(5);
                        llbehaviorMsg.RmsVal2         = data_new(6);
                        llbehaviorMsg.AvgRmsVal       = data_new(7);
                        llbehaviorMsg.AmplitudeVal1   = data_new(8);
                        llbehaviorMsg.AmplitudeVal2   = data_new(9);
                        llbehaviorMsg.AvgAmpVal       = data_new(10);
                        llbehaviorMsg.McLabel1        = rt_actionInt2actionLbl(data_new(11));
                        llbehaviorMsg.McLabel2        = rt_actionInt2actionLbl(data_new(12));
                        llbehaviorMsg.T1Start         = data_new(13);
                        llbehaviorMsg.T1End           = data_new(14);
                        llbehaviorMsg.T2Start         = data_new(15);
                        llbehaviorMsg.T2End           = data_new(16);
                        llbehaviorMsg.TAverage        = data_new(17);
                        send(llbehaviorPub,llbehaviorMsg);
                        %                         send(llbehaviorPubConstant.Value,llbehaviorMsg);
                    end
                end
                
            else
                % The only cycle
                while (Result.llbehMarker_cl+1 <= Result.llbehIndex)
                    [hasNew_llbc, data_new, Result.llbehLookForRepeat_cl, Result.llbehNumRepeat_cl, Result.llbehMarker_cl] = rt_llbehCompositionCleanUp(Result.llbehData,  Result.llbehLookForRepeat_cl, Result.llbehNumRepeat_cl, Result.llbehMarker_cl);
                    if (hasNew_llbc)
                        % Increase counter
                        Result.llbehIndex_cl = Result.llbehIndex_cl+1;
                        % Keep history of MC layer clean up data
                        Result.llbehData_cl(Result.llbehIndex_cl,:) = data_new;
                        
                        % Publish the new item to post processing
                        llbehaviorMsg.LlbLabel        = llbInt2llbLbl(data_new(1));
                        llbehaviorMsg.AverageVal1     = data_new(2);
                        llbehaviorMsg.AverageVal2     = data_new(3);
                        llbehaviorMsg.AvgMagVal       = data_new(4);
                        llbehaviorMsg.RmsVal1         = data_new(5);
                        llbehaviorMsg.RmsVal2         = data_new(6);
                        llbehaviorMsg.AvgRmsVal       = data_new(7);
                        llbehaviorMsg.AmplitudeVal1   = data_new(8);
                        llbehaviorMsg.AmplitudeVal2   = data_new(9);
                        llbehaviorMsg.AvgAmpVal       = data_new(10);
                        llbehaviorMsg.McLabel1        = rt_actionInt2actionLbl(data_new(11));
                        llbehaviorMsg.McLabel2        = rt_actionInt2actionLbl(data_new(12));
                        llbehaviorMsg.T1Start         = data_new(13);
                        llbehaviorMsg.T1End           = data_new(14);
                        llbehaviorMsg.T2Start         = data_new(15);
                        llbehaviorMsg.T2End           = data_new(16);
                        llbehaviorMsg.TAverage        = data_new(17);
                        send(llbehaviorPub,llbehaviorMsg);
                        %                         send(llbehaviorPubConstant.Value,llbehaviorMsg);
                    end
                end
                
                if (Result.llbehMarker_cl == Result.llbehIndex)
                    if (Result.llbehLookForRepeat_cl) % if looking for repeat
                        nextPrimitive   = Result.llbehNumRepeat_cl;
                        startPrimitive  = Result.llbehMarker_cl-Result.llbehNumRepeat_cl;
                        repeat_Lbl      = Result.llbehData(startPrimitive,behLbl);
                        data_new        = rt_MergeLowLevelBehaviors(startPrimitive,Result.llbehData,llbehLbl,repeat_Lbl,nextPrimitive);
                        Result.llbehIndex_cl = Result.llbehIndex_cl+1;
                        Result.llbehData_cl(Result.llbehIndex_cl,:) = data_new;
                        
                        % Publish the new item to post processing
                        llbehaviorMsg.LlbLabel        = llbInt2llbLbl(data_new(1));
                        llbehaviorMsg.AverageVal1     = data_new(2);
                        llbehaviorMsg.AverageVal2     = data_new(3);
                        llbehaviorMsg.AvgMagVal       = data_new(4);
                        llbehaviorMsg.RmsVal1         = data_new(5);
                        llbehaviorMsg.RmsVal2         = data_new(6);
                        llbehaviorMsg.AvgRmsVal       = data_new(7);
                        llbehaviorMsg.AmplitudeVal1   = data_new(8);
                        llbehaviorMsg.AmplitudeVal2   = data_new(9);
                        llbehaviorMsg.AvgAmpVal       = data_new(10);
                        llbehaviorMsg.McLabel1        = rt_actionInt2actionLbl(data_new(11));
                        llbehaviorMsg.McLabel2        = rt_actionInt2actionLbl(data_new(12));
                        llbehaviorMsg.T1Start         = data_new(13);
                        llbehaviorMsg.T1End           = data_new(14);
                        llbehaviorMsg.T2Start         = data_new(15);
                        llbehaviorMsg.T2End           = data_new(16);
                        llbehaviorMsg.TAverage        = data_new(17);
                        send(llbehaviorPub,llbehaviorMsg);
                        %                         send(llbehaviorPubConstant.Value,llbehaviorMsg);
                    else
                        data_new        = Result.llbehData(Result.llbehMarker_cl,:);
                        Result.llbehIndex_cl = Result.llbehIndex_cl+1;
                        Result.llbehData_cl(Result.llbehIndex_cl,:) = data_new;
                        
                        % Publish the new item to post processing
                        llbehaviorMsg.LlbLabel        = llbInt2llbLbl(data_new(1));
                        llbehaviorMsg.AverageVal1     = data_new(2);
                        llbehaviorMsg.AverageVal2     = data_new(3);
                        llbehaviorMsg.AvgMagVal       = data_new(4);
                        llbehaviorMsg.RmsVal1         = data_new(5);
                        llbehaviorMsg.RmsVal2         = data_new(6);
                        llbehaviorMsg.AvgRmsVal       = data_new(7);
                        llbehaviorMsg.AmplitudeVal1   = data_new(8);
                        llbehaviorMsg.AmplitudeVal2   = data_new(9);
                        llbehaviorMsg.AvgAmpVal       = data_new(10);
                        llbehaviorMsg.McLabel1        = rt_actionInt2actionLbl(data_new(11));
                        llbehaviorMsg.McLabel2        = rt_actionInt2actionLbl(data_new(12));
                        llbehaviorMsg.T1Start         = data_new(13);
                        llbehaviorMsg.T1End           = data_new(14);
                        llbehaviorMsg.T2Start         = data_new(15);
                        llbehaviorMsg.T2End           = data_new(16);
                        llbehaviorMsg.TAverage        = data_new(17);
                        send(llbehaviorPub,llbehaviorMsg);
                        %                         send(llbehaviorPubConstant.Value,llbehaviorMsg);
                    end
                end
            end
            break;
        end
        
    end
    
    fclose(fileR);
    
end