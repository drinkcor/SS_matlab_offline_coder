% function [Result] = sharedMethod

        
%         if(Result.primiMarker+window>row||Wrench(row,1)==0)
%             Wrench = struct2array(load('Wren_loc.mat'));
%             [row,~] = size(Wrench);
% %             fprintf('%d\t',num);
%             num = num+1;
% %             fprintf('%d\t',row);
% %             fprintf('%d\n',Result.primiMarker+window>row);
%             if(row>0)
%                 if(Wrench(row,1)==-1)
%                     %% last Iteration
%                     while(Result.primiMarker+window<=row)
%                         % primitive layer
%                         Result.primiIndex  = Result.primiIndex+1;
%                         Result.primiData(Result.primiIndex,:) = 5*Wrench(Result.primiMarker,:);
%                         Result.primiMarker = Result.primiMarker+1;
%                     end
%                     %                 fprintf('++++++++++++++++++++++++++++++++++++\n');
%                     break;
%                 end
%             end
%         else
%             
%             while(Result.primiMarker+window<=row)
%                 % primitive layer
%                 Result.primiIndex  = Result.primiIndex+1;
%                 Result.primiData(Result.primiIndex,:) = 5*Wrench(Result.primiMarker,:);
%                 Result.primiMarker = Result.primiMarker+1; 
%             end
% %             fprintf('++++++++++++++++++++++++++++++++++++\n');
% %             
% %             while(Result.primiMarker_cl+1 <= Result.primiIndex)
% %                     % primitive layer clean up
% %             end
% %             
% %             while(Result.MCMarker+1 <= Result.primiIndex_cl)
% %                     % mc layer
% %             end
%             
%         end

function [Result] = SharedM_subSnapVerification(axisIndex, StrategyType, FolderName)
    
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
                
                %
                %% Primitive_layer clean up
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
            break;
        end
        
    end
    
    fclose(fileR);
    
end