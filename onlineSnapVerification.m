function [Results] = onlineSnapVerification


% delete(gcp);
% parpool(2);

p = gcp();


f1 = parfeval(p,@rt_snapVerification,1,1,'HSA','2feafa');
f2 = parfeval(p,@rt_snapVerification,1,2,'HSA','2feafa');
% f3 = parfeval(p,@rt_snapVerification,1,3,'HSA','2feafa');
% f4 = parfeval(p,@rt_snapVerification,1,4,'HSA','2feafa');
% f5 = parfeval(p,@rt_snapVerification,1,5,'HSA','2feafa');
% f6 = parfeval(p,@rt_snapVerification,1,6,'HSA','2feafa');

Results = cell(1,6);

value1 = fetchOutputs(f1);
value2 = fetchOutputs(f2);
% value3 = fetchOutputs(f3);
% value4 = fetchOutputs(f4);
% value5 = fetchOutputs(f5);
% value6 = fetchOutputs(f6);

Results{1} = value1;
Results{2} = value2;

% for idx = 1:2
%   % fetchNext blocks until next results are available.
%   [completedIdx,value] = fetchNext(f);
%   Results{completedIdx} = value;
% %   fprintf('Got result with index: %d.\n', completedIdx);
% end


% Timing method
% tstart = tic;
% aaaatime = toc(tstart);

% fprintf('Time Consuming: %f',aaaatime);
% fprintf('Value1: %d',value1);
% fprintf('Value2: %d',value2);




end