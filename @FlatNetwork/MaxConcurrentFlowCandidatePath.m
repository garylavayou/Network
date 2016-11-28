%% Max-Concurrent Flow Problem with Candidate Paths
% 
%% Method Declaration
%      function [lambda, flow_rate, flow_variables] = MaxConcurrentFlowCandidatePath
%
% |fi|: If |fi| is a scalar, then |fi| is the traffic matrix index. Otherwise, |fi| must
% be a triple-value list representing the |flow_list|.
function [lambda, flow_rate, flow_variables] = MaxConcurrentFlowCandidatePath(this, fi, NUM_PATH)

graph = this.graph;
path_set = this.path_set;
if isscalar(fi)
    flow_list = this.TrafficProfiles{fi};
else
    flow_list = fi;
end
L = length(graph.Capacity);
K = size(flow_list, 1);
if nargin == 3
    P = path_set.MaxId;
else
    P = path_set.Number;
end

%% Flow-Path/Link-Path Incidence Matrix
Gs = sparse(K, P);
Hs = sparse(L, P);
for k = 1:K
	src = flow_list(k,1);
	dest = flow_list(k,2);
    if nargin == 3
        path_num = min(NUM_PATH, path_set.Count(src,dest));
    else
        path_num = path_set.Count(src,dest);
    end
	for j = 1:path_num
		path = path_set.Element{src,dest}(j);
		pid = path.id;
		Gs(k,pid) = 1; %#ok<SPRIX>
		for i = 1:(path.Length-1)
			e = path.Link(i);
			eid = graph.Inverse(e(1), e(2));
			Hs(eid, pid) = 1; %#ok<SPRIX>
		end
	end
end
d = flow_list(:,3);
O1s = sparse(K,1);

%% Linear programming
opt = optimoptions(@linprog, 'Diagnostics', 'on', 'Algorithm', 'interior-point', 'MaxIter', 400);
%% objective 1: max-concurrent
% min  [0 0 ... 0 -1]*|p|
%                     |¦Ë|
% s.t. |H 0|.|p| <= |C|
%            |¦Ë|    
%
%      |Gs -d|.|p|  = 0    --> O1s
%              |¦Ë|
%
%              |p| >= 0    --> O2s
%              |¦Ë|
% Note: Since limited by the concurrent constraint, it does not necessarily
% achieve max thoughput
f = [sparse(P,1); -1];
As = [Hs sparse(L, 1)];
b = [graph.Capacity];
Aeqs = [Gs -d];
O2s = sparse(P+1,1);
% [x,fval,exitflag,output]=linprog(f,A, b,Aeq, beq,lb, ub,x0, opt)
[F, ~, exitflag, output] = linprog(f,As,b,Aeqs,O1s,O2s,[],[], opt);
if exitflag ~= 1
    switch exitflag
        case 0
            fprintf('iteration number %d\n', output.iterations);
            error('error: linear programming does not converge.');
        otherwise
            error('error: linear programming failed.');
    end
end
F(F<10^-5) = 0;
lambda = F(end);
flow_rate = Gs*F(1:end-1);
fprintf('\tLinear programming optimal: ¦Ë = %.4f, system throughput = %.4f\n',...
    lambda, sum(flow_rate));

for p = 1:P
    pid = path_set.reverse_index(p,:);
    path_set.Element{pid(1),pid(2)}(pid(3)).bandwidth = F(p);
end

flow_variables = zeros(K,L);
for k = 1:K
	src = flow_list(k,1);
	dest = flow_list(k,2);
    if nargin == 3
        path_num = min(NUM_PATH, path_set.Count(src,dest));
    else
        path_num = path_set.Count(src,dest);
    end
    pid = zeros(path_num, 1);
    for j = 1:path_num
        pid(j) = path_set.Element{src,dest}(j).id;
    end
    p_sub = F(pid);
    flow_variables(k,:) = Hs(:,pid)*p_sub;
end
end