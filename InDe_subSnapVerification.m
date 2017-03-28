

function [Result] = InDe_subSnapVerification(axisIndex, StrategyType, FolderName)
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
    
    globalIndex = 0;
    Wrench      = zeros(1000,7);
    
    plotType = ['Fx';'Fy';'Fz';'Mx';'My';'Mz'];
    llbehLbl    = [ 1,   2,   3,   4,   5,   6,   7,  8];

    behLbl          = 1;   % action class
        
    localIndex  = 0;
    Result.Wren_loc    = zeros(1000,7);

        
    rosshutdown;
    rosinit;
    
    wrenchPub = rospublisher('/robot/limb/right/endpoint_state', 'baxter_core_msgs/EndpointState');   
    
    wrenchHandle  = rossubscriber('/robot/limb/right/endpoint_state',@wrenchCallback, 'BufferSize', 1000);
    % wrench_node_sub = robotics.ros.Node('/wrench_node_subscriber');
    % wrenchHandle    = robotics.ros.Subscriber(wrench_node_sub,'/robot/limb/right/endpoint_state','baxter_core_msgs/EndpointState',@wrenchCallback, 'BufferSize', 1);

    drawnow;
    
    %     segmentNodeConstant = parallel.pool.Constant(@() robotics.ros.Node('/segment_node'));
%     
%     segmentPubConstant{1} = parallel.pool.Constant(@() robotics.ros.Publisher(segmentNodeConstant.Value,'/topic_segments_fx', 'publish_files/Segments'));
%     segmentPubConstant{2} = parallel.pool.Constant(@() robotics.ros.Publisher(segmentNodeConstant.Value,'/topic_segments_fy', 'publish_files/Segments'));
%     segmentPubConstant{3} = parallel.pool.Constant(@() robotics.ros.Publisher(segmentNodeConstant.Value,'/topic_segments_fz', 'publish_files/Segments'));
%     segmentPubConstant{4} = parallel.pool.Constant(@() robotics.ros.Publisher(segmentNodeConstant.Value,'/topic_segments_mx', 'publish_files/Segments'));
%     segmentPubConstant{5} = parallel.pool.Constant(@() robotics.ros.Publisher(segmentNodeConstant.Value,'/topic_segments_my', 'publish_files/Segments'));
%     segmentPubConstant{6} = parallel.pool.Constant(@() robotics.ros.Publisher(segmentNodeConstant.Value,'/topic_segments_mz', 'publish_files/Segments'));

%     
%     compositeNodeConstant = parallel.pool.Constant(@() robotics.ros.Node('/composite_node'));
%     
%     compositePubConstant{1} = parallel.pool.Constant(@() robotics.ros.Publisher(compositeNodeConstant.Value,'/topic_composites_fx', 'publish_files/Composites'));
%     compositePubConstant{2} = parallel.pool.Constant(@() robotics.ros.Publisher(compositeNodeConstant.Value,'/topic_composites_fy', 'publish_files/Composites'));
%     compositePubConstant{3} = parallel.pool.Constant(@() robotics.ros.Publisher(compositeNodeConstant.Value,'/topic_composites_fz', 'publish_files/Composites'));
%     compositePubConstant{4} = parallel.pool.Constant(@() robotics.ros.Publisher(compositeNodeConstant.Value,'/topic_composites_mx', 'publish_files/Composites'));
%     compositePubConstant{5} = parallel.pool.Constant(@() robotics.ros.Publisher(compositeNodeConstant.Value,'/topic_composites_my', 'publish_files/Composites'));
%     compositePubConstant{6} = parallel.pool.Constant(@() robotics.ros.Publisher(compositeNodeConstant.Value,'/topic_composites_mz', 'publish_files/Composites'));
%     
    % save and load rospublisher obj can't work
    % Can't work     compositePub = rospublisher(strcat('/topic_composites',num2str(axisIndex)), 'publish_files/Composites');
    % Can't work     save(strcat('compositePub',num2str(axisIndex),'.mat'),'compositePub');
    

%     sub = rossubscriber('/topic_composites_fx', 'publish_files/Composites');

%     llbehaviorNodeConstant = parallel.pool.Constant(@() robotics.ros.Node('/llbehavior_node'));
%     
%     llbehaviorPubConstant{1} = parallel.pool.Constant(@() robotics.ros.Publisher(llbehaviorNodeConstant.Value,'/topic_llbehaviors_fx', 'publish_files/llBehaviors'));
%     llbehaviorPubConstant{2} = parallel.pool.Constant(@() robotics.ros.Publisher(llbehaviorNodeConstant.Value,'/topic_llbehaviors_fy', 'publish_files/llBehaviors'));
%     llbehaviorPubConstant{3} = parallel.pool.Constant(@() robotics.ros.Publisher(llbehaviorNodeConstant.Value,'/topic_llbehaviors_fz', 'publish_files/llBehaviors'));
%     llbehaviorPubConstant{4} = parallel.pool.Constant(@() robotics.ros.Publisher(llbehaviorNodeConstant.Value,'/topic_llbehaviors_mx', 'publish_files/llBehaviors'));
%     llbehaviorPubConstant{5} = parallel.pool.Constant(@() robotics.ros.Publisher(llbehaviorNodeConstant.Value,'/topic_llbehaviors_my', 'publish_files/llBehaviors'));
%     llbehaviorPubConstant{6} = parallel.pool.Constant(@() robotics.ros.Publisher(llbehaviorNodeConstant.Value,'/topic_llbehaviors_mz', 'publish_files/llBehaviors'));

%       segmentPub{1} = rospublisher('/topic_segments_fy', 'publish_files/Segments');
%       segmentPub{2} = rospublisher('/topic_segments_fy', 'publish_files/Segments');
%       segmentPub{3} = rospublisher('/topic_segments_fz', 'publish_files/Segments');
%       segmentPub{4} = rospublisher('/topic_segments_mx', 'publish_files/Segments');
%       segmentPub{5} = rospublisher('/topic_segments_my', 'publish_files/Segments');
%       segmentPub{6} = rospublisher('/topic_segments_mz', 'publish_files/Segments');
% 
%       compositePub{1} = rospublisher('/topic_composites_fx', 'publish_files/Composites');
%       compositePub{2} = rospublisher('/topic_composites_fy', 'publish_files/Composites');
%       compositePub{3} = rospublisher('/topic_composites_fz', 'publish_files/Composites');
%       compositePub{4} = rospublisher('/topic_composites_mx', 'publish_files/Composites');
%       compositePub{5} = rospublisher('/topic_composites_my', 'publish_files/Composites');
%       compositePub{6} = rospublisher('/topic_composites_mz', 'publish_files/Composites');
%       
%       llbehaviorPub{1} = rospublisher('/topic_llbehaviors_fy', 'publish_files/llBehaviors');
%       llbehaviorPub{2} = rospublisher('/topic_llbehaviors_fy', 'publish_files/llBehaviors');
%       llbehaviorPub{3} = rospublisher('/topic_llbehaviors_fz', 'publish_files/llBehaviors');
%       llbehaviorPub{4} = rospublisher('/topic_llbehaviors_mx', 'publish_files/llBehaviors');
%       llbehaviorPub{5} = rospublisher('/topic_llbehaviors_my', 'publish_files/llBehaviors');
%       llbehaviorPub{6} = rospublisher('/topic_llbehaviors_mz', 'publish_files/llBehaviors');
    
    %% Very important step: start parpool outside rt_snapVerification, just run the below in command line.
%     % Delete any possible existing pools running previously
%     delete(gcp);
%     parpool(2);
    

        segmentMsg = rosmessage('publish_files/Segments');
        compositeMsg = rosmessage('publish_files/Composites');
        llbehaviorMsg = rosmessage('publish_files/llBehaviors');
        
        % Variables to keep track of sending label
        segmentCur = 0;
        compositeCur = 0;
        llbehCur = 0;
        
        % Variables for Primitive layer
        Result.primiData                = zeros(100,7);     % primitive layer data
        Result.primiIndex               = 0;                % primitive layer index
        Result.primiDataFitPre          = nan;              % dataFit_pre
        Result.primiWStart              = 1;                % wStart
        Result.primiMarker              = 1;                % marker_p

%         ForceCell{1} = zeros(100,7);     % primitive layer data
%         ForceCell{2} = 0;                % primitive layer index
%         ForceCell{7} = nan;              % dataFit_pre
%         ForceCell{8} = 1;                % wStart
%         ForceCell{9} = 1;                % marker_p
        
        % Variables for Primitive layer CleanUp 


        Result.primiData_cl             = zeros(100,7);    % primitive layer CleanUp data
        Result.primiIndex_cl            = 0;               % primitive layer CleanUp index        
        Result.primiLookForRepeat_cl    = 0;               % lookForRepeat, 0 means not looking for repeat, 1 means looking for repeat
        Result.primiNumRepeat_cl        = 0;               % numberRepeated
        Result.primiMarker_cl           = 1;               % marker_pc

%         ForceCell{10} = zeros(100,7);    % primitive layer CleanUp data
%         ForceCell{11} = 0;               % primitive layer CleanUp index        
%         ForceCell{12} = 0;               % lookForRepeat, 0 means not looking for repeat, 1 means looking for repeat
%         ForceCell{13} = 0;               % numberRepeated
%         ForceCell{14} = 1;               % marker_pc
        
        
        % Variables for CompoundMotionComposition layer
        Result.MCData                   = zeros(100,11);   % CompoundMotionComposition layer data
        Result.MCIndex                  = 0;               % CompoundMotionComposition layer index
        Result.MCMarker                 = 1;               % marker_cm

%         ForceCell{3}  = zeros(100,11);   % CompoundMotionComposition layer data
%         ForceCell{4}  = 0;               % CompoundMotionComposition layer index
%         ForceCell{15} = 1;               % marker_cm
        
        % Variables for CompoundMotionComposition layer CleanUp
        Result.MCData_cl                = zeros(100,11);   % CompoundMotionComposition layer CleanUp data
        Result.MCIndex_cl               = 0;               % CompoundMotionComposition layer CleanUp index
        Result.MCLookForRepeat_cl       = 0;               % lookForRepeat, 0 means not looking for repeat, 1 means looking for repeat
        Result.MCNumRepeat_cl           = 0;               % numberRepeated
        Result.MCMaxAmp_cl              = 0;               % maxAmplitude
        Result.MCMarker_cl              = 1;               % marker_cmc
        
%         ForceCell{16} = zeros(100,11);   % CompoundMotionComposition layer CleanUp data
%         ForceCell{17} = 0;               % CompoundMotionComposition layer CleanUp index
%         ForceCell{18} = 0;               % lookForRepeat, 0 means not looking for repeat, 1 means looking for repeat
%         ForceCell{19} = 0;               % numberRepeated
%         ForceCell{20} = 0;               % maxAmplitude
%         ForceCell{21} = 1;               % marker_cmc

        % Variables for low level behaviour layer
        Result.llbehData                = zeros(100,17);    % low level behaviour layer data
        Result.llbehIndex               = 0;                % low level behaviour layer index
        Result.llbehMarker              = 1;                % marker_llb

%         ForceCell{5}  = zeros(100,17);    % low level behaviour layer data
%         ForceCell{6}  = 0;                % low level behaviour layer index
%         ForceCell{22} = 1;                % marker_llb
        
        % Variables for low level behaviour layer CleanUp
        Result.llbehData_cl             = zeros(100,17);    % low level behaviour layer CleanUp data
        Result.llbehIndex_cl            = 0;                % low level behaviour layer CleanUp index
        Result.llbehLookForRepeat_cl    = 0;                % lookForRepeat, 0 means not looking for repeat, 1 means looking for repeat
        Result.llbehNumRepeat_cl        = 0;                % numberRepeated
        Result.llbehMarker_cl           = 1;                % marker_llbc

%         ForceCell{23} = zeros(100,17);    % low level behaviour layer CleanUp data
%         ForceCell{24} = 0;                % low level behaviour layer CleanUp index
%         ForceCell{25} = 0;                % lookForRepeat, 0 means not looking for repeat, 1 means looking for repeat
%         ForceCell{26} = 0;                % numberRepeated
%         ForceCell{27} = 1;                % marker_llbc

    window_length   = 5;

    pType           = plotType(axisIndex,:); 

    segmentPub      = rospublisher(strcat('/topic_segments_',pType), 'publish_files/Segments');
    compositePub    = rospublisher(strcat('/topic_composites_',pType), 'publish_files/Composites');
    llbehaviorPub   = rospublisher(strcat('/topic_llbehaviors_',pType), 'publish_files/llBehaviors');
    
    
    
    while(1)     
        while (localIndex==globalIndex)
           % Wait for input
            pause(3);
        end
        tstart = tic;
        drawnow;
        while (localIndex<globalIndex)
            % Increase Counter for local index
            
            currentIndex = localIndex+5;
            if(currentIndex>globalIndex)
                currentIndex=globalIndex;
            end
            
%             fprintf('Time: %d ',currentIndex);
            
            while(localIndex<currentIndex)
                localIndex=localIndex+1;
                Result.Wren_loc(localIndex,1:7)  = Wrench(localIndex,1:7);
            end
            %Result.Wren_loc_new = Wrench_new;
            
            %
            %                 while(segmentCur < Result.primiIndex_cl||compositeCur < Result.MCIndex_cl||llbehCur < Result.llbehIndex_cl)
            %
            %                     if(segmentCur < Result.primiIndex_cl)
            %                         segmentCur=segmentCur+1;
            %                         data_new = Result.primiData_cl(segmentCur,:);
            %                         segmentMsg.Average   = data_new(1);
            %                         segmentMsg.MaxVal    = data_new(2);
            %                         segmentMsg.MinVal    = data_new(3);
            %                         segmentMsg.TStart    = data_new(4);
            %                         segmentMsg.TEnd      = data_new(5);
            %                         segmentMsg.Gradient  = data_new(6);
            %                         segmentMsg.GradLabel = gradInt2gradLbl(data_new(7));
            %                         send(segmentPub,segmentMsg);
            %                     end
            %
            %                     if(compositeCur < Result.MCIndex_cl)
            %                         compositeCur = compositeCur+1;
            %                         data_new = Result.MCData_cl(compositeCur,:);
            %                         compositeMsg.McLabel     = rt_actionInt2actionLbl(data_new(1));
            %                         compositeMsg.AverageVal  = data_new(2);
            %                         compositeMsg.RmsVal      = data_new(3);
            %                         compositeMsg.AmpVal      = data_new(4);
            %                         compositeMsg.GradLabel1  = gradInt2gradLbl(data_new(5));
            %                         compositeMsg.GradLabel2  = gradInt2gradLbl(data_new(6));
            %                         compositeMsg.T1Start     = data_new(7);
            %                         compositeMsg.T1End       = data_new(8);
            %                         compositeMsg.T2Start     = data_new(9);
            %                         compositeMsg.T2End       = data_new(10);
            %                         compositeMsg.TAverage    = data_new(11);
            %                         send(compositePub,compositeMsg);
            %
            %                     end
            %
            %                     if(llbehCur < Result.llbehIndex_cl)
            %                         llbehCur = llbehCur+1;
            %                         data_new = Result.llbehData_cl(llbehCur,:);
            %                         llbehaviorMsg.LlbLabel        = llbInt2llbLbl(data_new(1));
            %                         llbehaviorMsg.AverageVal1     = data_new(2);
            %                         llbehaviorMsg.AverageVal2     = data_new(3);
            %                         llbehaviorMsg.AvgMagVal       = data_new(4);
            %                         llbehaviorMsg.RmsVal1         = data_new(5);
            %                         llbehaviorMsg.RmsVal2         = data_new(6);
            %                         llbehaviorMsg.AvgRmsVal       = data_new(7);
            %                         llbehaviorMsg.AmplitudeVal1   = data_new(8);
            %                         llbehaviorMsg.AmplitudeVal2   = data_new(9);
            %                         llbehaviorMsg.AvgAmpVal       = data_new(10);
            %                         llbehaviorMsg.McLabel1        = rt_actionInt2actionLbl(data_new(11));
            %                         llbehaviorMsg.McLabel2        = rt_actionInt2actionLbl(data_new(12));
            %                         llbehaviorMsg.T1Start         = data_new(13);
            %                         llbehaviorMsg.T1End           = data_new(14);
            %                         llbehaviorMsg.T2Start         = data_new(15);
            %                         llbehaviorMsg.T2End           = data_new(16);
            %                         llbehaviorMsg.TAverage        = data_new(17);
            %                         send(llbehaviorPub,llbehaviorMsg);
            %
            %                     end
            %                 end
            %
            %             fprintf('\tGlobalIndex: %8f\tLocalIndex: %8f\n',globalIndex,localIndex);
            % fprintf('\tTimeStamp: %8f\tTimeStamp.toSec: %8f\n',Result.Wren_loc(localIndex,1),Result.Wren_loc(localIndex,1).toSec());
            % fprintf('\tTimeStamp: %8f\n',Result.Wren_loc(localIndex,1));rt
            %          fprintf('labindex = %d\n',labindex);
            
            %                 fprintf('labindex = %d\n',labindex);
            
            
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
            
            
            
            drawnow;
        end
        drawnow;
        if(localIndex==globalIndex)
            %% Last iteration to wrap up
            
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
    
    aaaatime = toc(tstart);
    fprintf('Time consuming: %f\n',aaaatime);
    %      fprintf('Last time: %f',sub.LatestMessage.T2End);
    fprintf('Amount of global items: %d\n',globalIndex);
    fprintf('Amount of local items: %d\n',localIndex);
    
    %      plot(1:localIndex,Result.Wren_loc(:,2));
    %     hold on;
    %     plot(1:localIndex,Result.Wren_loc(:,3));
    %     plot(1:localIndex,Result.Wren_loc(:,4));
    %
    %     save('Result.Wren_loc.mat','Result.Wren_loc');
    
    
    
       
end
