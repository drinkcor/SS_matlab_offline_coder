function [Results] = SharedM_onlineSnapVerification
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

    f1 = parfeval(p,@SharedM_subSnapVerification,1,1,'HSA','2feafa'); % axisIndex=1
    f2 = parfeval(p,@SharedM_subSnapVerification,1,2,'HSA','2feafa'); % axisIndex=2
    f3 = parfeval(p,@SharedM_subSnapVerification,1,3,'HSA','2feafa'); % axisIndex=3
    f4 = parfeval(p,@SharedM_subSnapVerification,1,4,'HSA','2feafa'); % axisIndex=4
    f5 = parfeval(p,@SharedM_subSnapVerification,1,5,'HSA','2feafa'); % axisIndex=5
    f6 = parfeval(p,@SharedM_subSnapVerification,1,6,'HSA','2feafa'); % axisIndex=6

    while(1)
        while (localIndex==globalIndex)
            % Wait for input
            pause(2);
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
%     
    value1 = fetchOutputs(f1);
    value2 = fetchOutputs(f2);
    value3 = fetchOutputs(f3);
    value4 = fetchOutputs(f4);
    value5 = fetchOutputs(f5);
    value6 = fetchOutputs(f6);
    
    aaaatime = toc(tstart);
    fprintf('Time consuming: %f\n',aaaatime);
    fprintf('Amount of global items: %d ',globalIndex);
    fprintf('Amount of local items: %d ',localIndex);
    
    Results{1} = value1;
    Results{2} = value2;
    Results{3} = value3;
    Results{4} = value4;
    Results{5} = value5;
    Results{6} = value6;


end