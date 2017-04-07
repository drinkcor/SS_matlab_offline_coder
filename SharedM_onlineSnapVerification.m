function [Results,Middles] = SharedM_onlineSnapVerification
% Shared Memory online snapverification, the client is responsible for 
% receiving ROS message, writing the received data to shared memory, and 
% distribute 6 axis jobs to 6 different workers. Each worker read data 
% from shared memory and process its axis's data independently. 


    global globalIndex;
    global Wrench;
    
    globalIndex = 0;
    Wrench      = zeros(1000,7);

    localIndex  = 0;
%     Wren_loc    = zeros(1000,7);

    rosshutdown;
    rosinit;
    
    wrenchPub = rospublisher('/robot/limb/right/endpoint_state', 'baxter_core_msgs/EndpointState');  
    wrenchHandle  = rossubscriber('/robot/limb/right/endpoint_state',@wrenchCallback, 'BufferSize', 1000);
    
%     delete('Wren_loc.mat');
% %     

    
    
%     % Shared file
%     W   = zeros(1,7);
%     save wrench.mat W -v7.3;
%     wrench = matfile('wrench.mat') ;
%     wrench.Properties.Writable = true;
    drawnow;
    
    delete('WrenchData.bin');
    fileID = fopen('WrenchData.bin','w');
%     
    p = gcp();

      f1 = parfeval(p,@SharedM_subSnapVerification,2,5,0.5,3,3,3,'HSA','2feafa'); % axisIndex=1
%     f2 = parfeval(p,@SharedM_subSnapVerification,1,2,'HSA','2feafa'); % axisIndex=2
%     f3 = parfeval(p,@SharedM_subSnapVerification,1,3,'HSA','2feafa'); % axisIndex=3
%     f4 = parfeval(p,@SharedM_subSnapVerification,1,4,'HSA','2feafa'); % axisIndex=4
%     f5 = parfeval(p,@SharedM_subSnapVerification,1,5,'HSA','2feafa'); % axisIndex=5
%     f6 = parfeval(p,@SharedM_subSnapVerification,1,6,'HSA','2feafa'); % axisIndex=6

    while(1)
        while (localIndex==globalIndex)
            % Wait for input
            pause(3.5);
        end
        
        drawnow;
        while (localIndex<globalIndex)
            
            currentIndex = localIndex+2;
            if(currentIndex>globalIndex)
                currentIndex=globalIndex;
            end
            
            %% file I/O method
            for i = localIndex+1 : currentIndex
                fwrite(fileID,Wrench(i,:),'double');
            end
            
            
            
%             fprintf('Time: %d ',currentIndex);
%             
%             %% save mat file method
%             Wren_loc(localIndex+1:currentIndex,1:7)  = Wrench(localIndex+1:currentIndex,1:7);
%             

%             %% 7.3 MAT-file method     
%             wrench.W(localIndex+1:currentIndex,1:7)  = Wrench(localIndex+1:currentIndex,1:7);
            
            drawnow;
    
            %% save and load method
%             save('Wren_loc.mat','Wren_loc','-mat');
%             updateAttachedFiles(p);  
            
            localIndex  = currentIndex;
            drawnow;
           

            %% Usual method
%             Wren_loc(localIndex,1:7)  = Wrench(localIndex,1:7);

            %             fprintf('\tGlobalIndex: %8f\tLocalIndex: %8f\n',globalIndex,localIndex);
            
        end
        drawnow;
        if(localIndex==globalIndex)
%             Wren_loc(localIndex+1,:) = [-1,0,0,0,0,0,0];
%             save('Wren_loc.mat','Wren_loc','-mat');
            tstart = tic;
            lastItem    = [-1,0,0,0,0,0,0];
            fwrite(fileID,lastItem,'double');
            localIndex = localIndex+1;
            
%             updateAttachedFiles(p);
            break;
        end
    end
    
    fclose(fileID);
    
    
    

    
    Results = cell(1,6);
    Middles = cell(1,6);
    
    [Results{1}, Middles{1}] = fetchOutputs(f1);

    
%     value2 = fetchOutputs(f2);
%     Results{2} = value2;

%     value3 = fetchOutputs(f3);
%     Results{3} = value3;

%     value4 = fetchOutputs(f4);
%     Results{4} = value4;

%     value5 = fetchOutputs(f5);
%     Results{5} = value5;

%     value6 = fetchOutputs(f6);
%     Results{6} = value6;    
    
    
    






    aaaatime = toc(tstart);
    fprintf('Time consuming: %f\n',aaaatime);
    fprintf('Amount of global items: %d ',globalIndex);
    fprintf('Amount of local items: %d ',localIndex);

    %% Do some plot here
%     pFxL=subplot(3,2,1); plot(Wrench(:,1),Wrench(:,2));
%     title('Fx Plot'); xlabel('Time (secs)'); ylabel('Force (N)');
%     axis([Wrench(1,1),Wrench(globalIndex,1),min(Wrench(:,2)),max(Wrench(:,2))]);
%     
%     pFyL=subplot(3,2,3); plot(Wrench(:,1),Wrench(:,3));
%     title('Fy Plot'); xlabel('Time (secs)'); ylabel('Force (N)');
%     axis([Wrench(1,1),Wrench(globalIndex,1),min(Wrench(:,3)),max(Wrench(:,3))]);
%     
%     pFzL=subplot(3,2,5); plot(Wrench(:,1),Wrench(:,4));
%     title('Fz Plot'); xlabel('Time (secs)'); ylabel('Force (N)');
%     axis([Wrench(1,1),Wrench(globalIndex,1),min(Wrench(:,4)),max(Wrench(:,4))]);
%     
%     pMxL=subplot(3,2,2); plot(Wrench(:,1),Wrench(:,5));
%     title('Mx Plot'); xlabel('Time (secs)'); ylabel('Moment (N-m)');
%     axis([Wrench(1,1),Wrench(globalIndex,1),min(Wrench(:,5)),max(Wrench(:,5))]);
%     
%     pMyL=subplot(3,2,4); plot(Wrench(:,1),Wrench(:,6));
%     title('My Plot'); xlabel('Time (secs)'); ylabel('Moment (N-m)');
%     axis([Wrench(1,1),Wrench(globalIndex,1),min(Wrench(:,6)),max(Wrench(:,6))]);
%     
%     pMzL=subplot(3,2,6); plot(Wrench(:,1),Wrench(:,7));
%     title('Mz Plot'); xlabel('Time (secs)'); ylabel('Moment (N-m)');
%     axis([Wrench(1,1),Wrench(globalIndex,1),min(Wrench(:,7)),max(Wrench(:,7))]);
%     
    %% Write result to file
    
%     plotType        = ['Fx';'Fy';'Fz';'Mx';'My';'Mz'];
%    
%     for axisIndex=1:1
%         
%         pType           = plotType(axisIndex,:);
%         
%         % 1. Write segments to file
%         SegmentFolder   = './Segments';
%         
%         % Check if directory exists, if not create a directory
%         if(exist(SegmentFolder,'dir')==0)
%             mkdir(SegmentFolder);
%         end
%         
%         FileName = strcat(SegmentFolder,'/Segement_',pType,'.txt');
%         
%         % Open the file
%         fid = fopen(FileName, 'a+t');	% Open/create new file 4 writing in text mode 't'
%         for i = 1:Results{axisIndex}.primiIndex_cl
%             % Append data to end of file.
%             fprintf(fid, 'Iteration : %d\n',   i);
%             fprintf(fid, 'Average   : %.5f\n', Results{axisIndex}.primiData_cl(i,1));
%             fprintf(fid, 'Max Val   : %.5f\n', Results{axisIndex}.primiData_cl(i,2));
%             fprintf(fid, 'Min Val   : %.5f\n', Results{axisIndex}.primiData_cl(i,3));
%             fprintf(fid, 'Start     : %.5f\n', Results{axisIndex}.primiData_cl(i,4));
%             fprintf(fid, 'Finish    : %.5f\n', Results{axisIndex}.primiData_cl(i,5));
%             fprintf(fid, 'Gradient  : %.5f\n', Results{axisIndex}.primiData_cl(i,6));
%             
%             % Convert dLabel to a string
%             if(ischar(Results{axisIndex}.primiData_cl(i,7)))
%                 fprintf(fid, 'Grad Label: %s  \n', Results{axisIndex}.primiData_cl(i,7));
%             else
%                 dLabel = gradInt2gradLbl(Results{axisIndex}.primiData_cl(i,7));
%                 fprintf(fid, 'Grad Label: %s  \n', dLabel);
%                 fprintf(fid, '\n');
%             end
%         
%         end
%         
%         fclose(fid);
%         
%         
%         % 2. Write composites to file
%         CompositeFolder   = './Composites';
%         
%         if(exist(CompositeFolder,'dir')==0)
%             mkdir(CompositeFolder);
%         end
%         
%         FileName = strcat(CompositeFolder,'/Composites_',pType,'.txt');
%         
%         % Open the file
%         fid = fopen(FileName, 'a+t');	% Open/create new file 4 writing in text mode 't'
%         for i = 1:Results{axisIndex}.MCIndex_cl
%             % Append data to end of file.
%             fprintf(fid, 'Iteration     : %d\n',   i);
%             fprintf(fid, 'Label         : %s\n',   rt_actionInt2actionLbl(Results{axisIndex}.MCData_cl(i,1)));
%             fprintf(fid, 'Average Val   : %.5f\n', Results{axisIndex}.MCData_cl(i,2));
%             fprintf(fid, 'RMS Val       : %.5f\n', Results{axisIndex}.MCData_cl(i,3));
%             fprintf(fid, 'Amplitude Val : %.5f\n', Results{axisIndex}.MCData_cl(i,4));
%             fprintf(fid, 'Label 1       : %s\n',   gradInt2gradLbl(Results{axisIndex}.MCData_cl(i,5))); % Modified July 2012
%             fprintf(fid, 'Label 2       : %s\n',   gradInt2gradLbl(Results{axisIndex}.MCData_cl(i,6)));
%             fprintf(fid, 't1Start       : %.5f\n', Results{axisIndex}.MCData_cl(i,7));
%             fprintf(fid, 't1End         : %.5f\n', Results{axisIndex}.MCData_cl(i,8));
%             fprintf(fid, 't2Start       : %.5f\n', Results{axisIndex}.MCData_cl(i,9));
%             fprintf(fid, 't2End         : %.5f\n', Results{axisIndex}.MCData_cl(i,10));
%             fprintf(fid, 'tAvgIndex     : %.5f\n', Results{axisIndex}.MCData_cl(i,11));
%             fprintf(fid, '\n');
%             
%         end
%         
%         fclose(fid);
%         
%         
%         % 3. Write llbehs to file
%         llBehaviorFolder   = './llBehaviors';
%         
%         if(exist(llBehaviorFolder,'dir')==0)
%             mkdir(llBehaviorFolder);
%         end
%         
%         FileName = strcat(llBehaviorFolder,'/llBehaviors_',pType,'.txt');
%         % Open the file
%         fid = fopen(FileName, 'a+t');	% Open/create new file 4 writing in text mode 't'
%         
%         for i = 1:Results{axisIndex}.llbehIndex_cl
%             % Append data to end of file.
%             fprintf(fid, 'Iteration     : %d\n',   i);
%             fprintf(fid, 'llbehLabel    : %s\n',   llbInt2llbLbl(Results{axisIndex}.llbehData_cl(i,1)));
%             fprintf(fid, 'averageVal1   : %.5f\n', Results{axisIndex}.llbehData_cl(i,2));
%             fprintf(fid, 'averageVal2   : %.5f\n', Results{axisIndex}.llbehData_cl(i,3));
%             fprintf(fid, 'AVG_MAG_VAL   : %.5f\n', Results{axisIndex}.llbehData_cl(i,4));
%             fprintf(fid, 'rmsVal1       : %.5f\n', Results{axisIndex}.llbehData_cl(i,5));
%             fprintf(fid, 'rmsVal2       : %.5f\n', Results{axisIndex}.llbehData_cl(i,6));
%             fprintf(fid, 'AVG_RMS_Val   : %.5f\n', Results{axisIndex}.llbehData_cl(i,7));
%             fprintf(fid, 'amplitudeVal1 : %.5f\n', Results{axisIndex}.llbehData_cl(i,8));
%             fprintf(fid, 'amplitudeVal2 : %.5f\n', Results{axisIndex}.llbehData_cl(i,9));
%             fprintf(fid, 'AVG_AMP_VAL   : %.5f\n', Results{axisIndex}.llbehData_cl(i,10));
%             fprintf(fid, 'Label 1       : %s\n',   rt_actionInt2actionLbl(Results{axisIndex}.llbehData_cl(i,11)));
%             fprintf(fid, 'Label 2       : %s\n',   rt_actionInt2actionLbl(Results{axisIndex}.llbehData_cl(i,12)));
%             fprintf(fid, 't1Start       : %.5f\n', Results{axisIndex}.llbehData_cl(i,13));
%             fprintf(fid, 't1End         : %.5f\n', Results{axisIndex}.llbehData_cl(i,14));
%             fprintf(fid, 't2Start       : %.5f\n', Results{axisIndex}.llbehData_cl(i,15));
%             fprintf(fid, 't2End         : %.5f\n', Results{axisIndex}.llbehData_cl(i,16));
%             fprintf(fid, 'tAvgIndex     : %.5f\n', Results{axisIndex}.llbehData_cl(i,17));
%             fprintf(fid, '\n');
%             
%         end
%         
%         fclose(fid);
%     end
end