% function Wrench = WrenchCallback(~,message)
function wrenchCallback(~,message)
% Subscribe to /robot/limb/right/endpoint_state
% Method of use: 
%   clear Wrench
%   clear dataIndex
%
%   global dataIndex
%   global Wrench
%   global Wrench_new
% 
%   dataIndex = 1
%   Wrench = zeros(30,7)
%
%    WrenchHandle = rossubscriber('/robot/limb/right/	',@WrenchCallback);

%%

global Wrench;
global Wrench_new;
global globalIndex;

   Fx = message.Wrench.Force.X;
   Fy = message.Wrench.Force.Y;
   Fz = message.Wrench.Force.Z;
   Mx = message.Wrench.Torque.X; 
   My = message.Wrench.Torque.Y; 
   Mz = message.Wrench.Torque.Z;  
   
   if((Fx~=0)||(Fy~=0)||(Fz~=0)||(Mx~=0)||(My~=0)||(Mz~=0)) 
        Wrench_new = [Fx Fy Fz Mx My Mz];
        globalIndex = globalIndex+1;
        Wrench(globalIndex,:) = [globalIndex Fx Fy Fz Mx My Mz];
   end
   

end

