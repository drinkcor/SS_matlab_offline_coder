function [ amplitude,amp1,amp2 ] = rt_computeAmp( p1maxmin, p2maxmin )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
      amp1=p1maxmin(1,1)-p1maxmin(1,2);
      amp2=p2maxmin(1,1)-p2maxmin(1,2);

      high = max([p1maxmin p2maxmin]); low = min([p1maxmin p2maxmin]);
      amplitude = high-low;

end

