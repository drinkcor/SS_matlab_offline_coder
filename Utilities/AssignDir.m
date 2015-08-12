%%*************************** Documentation *******************************
% Function used for Snap Assembly strategies. 
% Assigns appropriate strategy path based on the kind of approach used.
%
% If new categories are created for HIRO, please do a global
% search for HSA and make sure that any string comparisons include the new
% categories. Need to improve the way we do this. There are several
% instances in the code, where parameters change according to the type of
% experiment that is being run.
%
% I.e. loadData.m, SnapData3.m, InsertStates3.m, WritePrimitivesToFile.m,
% cleanUp.m, WriteCompositesToFile.m, plotMotionCompositions.m,
% GradientClassification.m, CustomizePlotLength.m
%**************************************************************************
function StratTypeFolder = AssignDir(StrategyType)

% Assign a directory path based on the StrategyType used. 
    if strcmp(StrategyType,'S')
        StratTypeFolder = 'PositionControl/StraightLineApproach-NewIKinParams/';            % Straight Line with new IKin params
    
    elseif strcmp(StrategyType,'SN')
        StratTypeFolder = 'PositionControl/StraightLineApproach-NewIkinParams-Noise/';      % Straight Line with new IKin params with noise
    
    elseif strcmp(StrategyType,'P')
        StratTypeFolder = 'PositionControl/PivotApproach-NewIkinParams/';                   % Pivot approach with new IKin Params
    
    elseif strcmp(StrategyType,'PN')
        StratTypeFolder = 'PositionControl/PivotApproach-NewIKin-Noise/';                   % Pivot approach with new IKin Params with noise
    
    elseif strcmp(StrategyType,'FS')
        StratTypeFolder = 'ForceControl/SIM_StraightLineApproach/';                         % Used with PA10 Simulation
    
    %% Simulations: Side Approach 
    elseif strcmp(StrategyType,'SIM_PivotApproach')
        StratTypeFolder = strcat('ForceControl/',StrategyType,'/');                         % Used with PA10 PivotApproach Simulation
    
    elseif strcmp(StrategyType,'SIM_SideApproach')
        StratTypeFolder = strcat('ForceControl/',StrategyType,'/');                         % Used with HIRO SideApproach Simulation and Physical
    
    %% Simulations: Side Approach Error Characterization
    elseif strcmp(StrategyType, 'SIM_SA_ErrorCharac_001')
        StratTypeFolder = strcat('ForceControl/',StrategyType,'/');                         % Used with HIRO SideApproach to compute error characteristics
    
    elseif strcmp(StrategyType, 'SIM_SA_ErrorCharac_002')
        StratTypeFolder = strcat('ForceControl/',StrategyType,'/');                         
        
    elseif strcmp(StrategyType, 'SIM_SA_ErrorCharac_003')
        StratTypeFolder = strcat('ForceControl/',StrategyType,'/');                         
    
    elseif strcmp(StrategyType, 'SIM_SA_ErrorCharac_004')
        StratTypeFolder = strcat('ForceControl/',StrategyType,'/');                         
        
    %% Simulations: Dual Arm Side Approach 
    elseif strcmp(StrategyType, 'SIM_SA_DualArm')
        StratTypeFolder = strcat('ForceControl/',StrategyType,'/');                         % Used with simulations of the HIRO robot performing a side approach strategy with two arms: right and left under a coordination policy of push -hold.           
        
    %% Robot: Side Approach
    elseif strcmp(StrategyType, 'HIRO_SideApproach')
        StratTypeFolder = strcat('ForceControl/',StrategyType,'/');                         % Used with real hiro robot performing the side approach strategy.
    
    %% Robot: Side Approach Error Characterization
    elseif strcmp(StrategyType, 'HIRO_SA_ErrorCharac')
        StratTypeFolder = strcat('ForceControl/',StrategyType,'/');                         % Used with real hiro robot performing error deviations in the side approach strategy.
    
    else
        StratTypeFolder = '';
%        FolderName='';
    end
end