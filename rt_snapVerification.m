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
    
    pub = rospublisher('/robot/limb/right/endpoint_state', 'baxter_core_msgs/EndpointState');
    
    wrenchHandle  = rossubscriber('/robot/limb/right/endpoint_state',@wrenchCallback, 'BufferSize', 1000);
    % wrench_node_sub = robotics.ros.Node('/wrench_node_subscriber');
    % wrenchHandle    = robotics.ros.Subscriber(wrench_node_sub,'/robot/limb/right/endpoint_state','baxter_core_msgs/EndpointState',@wrenchCallback, 'BufferSize', 1);
    
    % Delete any possible existing pools running previously
    delete(gcp);
    parpool(2);
    
    for axisIndex = 1:6
        ForceCell{axisIndex}{1} = zeros(100,7);     % primitive level data
        ForceCell{axisIndex}{2} = 1;                % primitive level index
        ForceCell{axisIndex}{7} = nan;              % dataFit_pre
        ForceCell{axisIndex}{8} = 1;                % wStart
        ForceCell{axisIndex}{9} = 1;                % marker
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
                % Primitive_layer: the process won't be execute unless there is enough data for fitting.
                if (ForceCell{axisIndex}{9}+window_length <= localIndex)
                    fprintf('marker: %1.0f  ',ForceCell{axisIndex}{9});
                    fprintf('wStart: %1.0f  ',ForceCell{axisIndex}{8});
                    [ForceCell{axisIndex}{1},ForceCell{axisIndex}{2},ForceCell{axisIndex}{7},ForceCell{axisIndex}{8},ForceCell{axisIndex}{9}] = rt_fitRegressionCurves(Wren_loc,ForceCell{axisIndex}{1},ForceCell{axisIndex}{2},StrategyType,FolderName,pType,ForceCell{axisIndex}{7},ForceCell{axisIndex}{8},ForceCell{axisIndex}{9},rate,axisIndex);    % fit window
                end
                
   %            [ForceCell{axisIndex}{3},ForceCell{axisIndex}{4}] = rt_CompoundMotionComposition(ForceCell{axisIndex}{1},ForceCell{axisIndex}{2},ForceCell{axisIndex}{3},ForceCell{axisIndex}{4} );
   %            [ForceCell{axisIndex}{5},ForceCell{axisIndex}{6}] = rt_llbehComposition(ForceCell{axisIndex}{3},ForceCell{axisIndex}{4},ForceCell{axisIndex}{5},ForceCell{axisIndex}{6});
    
                 % fprintf('\tWren_loc(%1.0f', idx)
                 % fprintf(') = %8.4f', Wren_loc(localIndex,idx));
                
            end
            drawnow;
        end 
        drawnow;
        a=input('please input 2 to run the last iteration: ');
        if(a==2)
%      last iteration of primitive layer            
            parfor axisIndex = 1:6
                % Set the final variables
                wFinish     = localIndex;                            % Set to the last index of statData (the primitives space)
                Range       = ForceCell{axisIndex}{8}:wFinish;               % Save from wStart to the immediately preceeding index that passed the threshold
                Time        = (Range)'; % Wren_loc(Range,1);          % Time indeces that we are working with
                Data        = Wren_loc(Range,axisIndex+1); % Corresponding force data for a given force element in a given window
                
                polyCoeffs  = polyfit(Time,Data,1);            % First-order fit
                dataFit     = polyval(polyCoeffs, Time);

%%              ii) Retrieve the segment's statistical Data and write to file
                [dAvg,dMax,dMin,dStart,dFinish,dGradient,dLabel]=rt_statisticalData(Time(1),Time(length(Time)),dataFit,polyCoeffs,FolderName,StrategyType,axisIndex); % 1+windowlength

                % iii) Keep history of statistical data 
                % All data types are numerical in this version. // Prior
                % versions: Given that the datatypes are mixed, we must use cells. See {http://www.mathworks.com/help/techdoc/matlab_prog/br04bw6-98.html}       
                ForceCell{axisIndex}{1}(ForceCell{axisIndex}{2},:) = [dAvg dMax dMin dStart dFinish dGradient dLabel];
                        
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
