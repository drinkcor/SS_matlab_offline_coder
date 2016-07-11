% % Structure of ForcePicture: (Copy to a fixed structure(sliced varialbe in parfor) for post-processing)
% ForceCell= {
%                 Fx = { statData, segmentIndex, motComs, compouIndex, llbehStruc, llbehIndex, dataFit_pre, wStart, marker}
%                 Fy = { statData, segmentIndex, motComs, compouIndex, llbehStruc, llbehIndex }
%                 Fz = { statData, segmentIndex, motComs, compouIndex, llbehStruc, llbehIndex }
%                 Mx = { statData, segmentIndex, motComs, compouIndex, llbehStruc, llbehIndex }
%                 My = { statData, segmentIndex, motComs, compouIndex, llbehStruc, llbehIndex }
%                 Mz = { statData, segmentIndex, motComs, compouIndex, llbehStruc, llbehIndex }
% if you want to get segmentIndex of Fx, you should use: ForceCell{1}{1}
% if you want to get compouIndex of My, you should use: ForceCell{5}{3}


function  [hlbBelief,llbBelief,...
           stateTimes,hlbehStruc,...
           fcAvgData,boolFCData] = rt_snapVerification(StrategyType,FolderName,first,last)
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
    
    % primary_level data and index for it
    statData       = zeros(100,7);             % Growing array that will hold segmented block statistical data (int types).
    segmentIndex   = 1; 
        
    rosshutdown;
    rosinit;
    
    pub = rospublisher('/robot/limb/right/endpoint_state', 'baxter_core_msgs/EndpointState');
    
    wrenchHandle  = rossubscriber('/robot/limb/right/endpoint_state',@wrenchCallback, 'BufferSize', 1000);
    % wrench_node_sub = robotics.ros.Node('/wrench_node_subscriber');
    % wrenchHandle    = robotics.ros.Subscriber(wrench_node_sub,'/robot/limb/right/endpoint_state','baxter_core_msgs/EndpointState',@wrenchCallback, 'BufferSize', 1);
    
    % Delete any possible existing pools running previously
    delete(gcp);
    parpool(2);
    
    rate = 40;
    wStart = 1;
    marker = 1;
    
     while(1)     
        while (localIndex<globalIndex)
            % Increase Counter for local index
            localIndex=localIndex+1;
            
            Wren_loc(localIndex,2:7)  = Wrench(localIndex,2:7);                      
            %Wren_loc_new = Wrench_new;
            
            fprintf('\tGlobalIndex: %8f\tLocalIndex: %8f\n',globalIndex,localIndex);
            % WrenchHandle.LatestMessage;
            
            parfor axisIndex = 2:7
                pType  = plotType(axisIndex,:);  
            %   if (mod(localIndex,window_length)==0)
                    [ForceCell{axisIndex}{1},ForceCell{axisIndex}{2},ForceCell{axisIndex}{7},ForceCell{axisIndex}{8},ForceCell{axisIndex}{9}] = rt_fitRegressionCurves(Wren_loc,localIndex,ForceCell{axisIndex}{1},ForceCell{axisIndex}{2},fPath,StrategyType,StratTypeFolder,FolderName,Type,ForceCell{axisIndex}{7},ForceCell{axisIndex}{8},ForceCell{axisIndex}{9},rate,pType,axisIndex);    % fit window
                    [ForceCell{axisIndex}{3},ForceCell{axisIndex}{4}] = rt_CompoundMotionComposition(ForceCell{axisIndex}{1},ForceCell{axisIndex}{2},ForceCell{axisIndex}{3},ForceCell{axisIndex}{4} );
                    [ForceCell{axisIndex}{5},ForceCell{axisIndex}{6}] = rt_llbehComposition(ForceCell{axisIndex}{3},ForceCell{axisIndex}{4},ForceCell{axisIndex}{5},ForceCell{axisIndex}{6});
                    % fprintf('\tWren_loc(%1.0f', idx)
                    % fprintf(') = %8.4f', Wren_loc(localIndex,idx));
                end
            end
            drawnow;
        end 
        drawnow;
         a=input('please input the centure a: ');
         if(a==2)
             break;
         end
    end
     
     

      plot(1:localIndex,Wren_loc(:,2));
      hold on;
      plot(1:localIndex,Wren_loc(:,3));
      plot(1:localIndex,Wren_loc(:,4));
      
      save('Wren_loc.mat','Wren_loc');
    
       
       
       
end
