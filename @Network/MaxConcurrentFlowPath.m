%% Full Polynomial Time Approximation Schemes(FPTAS)
%  FPTAS for for multi-commodity flow problems (i.e. maximum concurrent flow problems)
%  This implementation is based on the work of [Garg2007], [Karakostas2008,3.1] and
%  [Fleischer2000,3]  
%% Mehtod declaration
% [lambda_fptas, path_set] = MaxConcurrentFlowPath(this, w, traffic_matrix_id)
%
% |lambda_fptas|: objective value of FPTAS and i-FPTAS.
%
% |path_set|: Set of paths that allocated to flows.
function [lambda_fptas, path_set] = MaxConcurrentFlowPath(this, w, traffic_matrix_id)

fprintf('Maximum concurrent multi-commodity problem with path-flow output:\n');
%% Graph parameters
graph = this.graph;
flow_list=this.TrafficProfiles{traffic_matrix_id};
L = length(graph.Capacity);              % number of edges;
N = size(graph.Adjacent,1);

%% assort flows by sources(or destination)
SourceFlowIndex = cell(1,N);
K = size(flow_list,1);
for src = 1:N
    SourceFlowIndex{src}=find(flow_list(:,1)==src);
end
% DestFlowIndex = cell(1,N);
% for i=1:K   % visit each flow
%     dest = flow_list(i,2);
%     DestFlowIndex{dest}=[DestFlowIndex{dest};i];
% end
%% approximation parameters
epsilon = 1-(1+w)^(-1/3);
delta =  (1+epsilon)^((epsilon-1)/epsilon)*((1-epsilon)/L)^(1/epsilon);
z = log((1+epsilon)/delta)/log(1+epsilon);

origin_flow = flow_list(:,3);

%% path specifications
path_set = PathSet(N);
path_set_temp = PathSet(N);

%% FPTAS
le = ones(L,1)*delta./(graph.Capacity); 	% length of edges;
Gl0 = ones(N)*inf;
for i=1:N
    Gl0(i,i) = 0;
end
stop_cond = dot(graph.Capacity,le);
% iterations_counter = 0;
phases_counter = 0;
total_phases_counter = 0;
total_steps_counter = 0;
b_full_iter = 1;
b_full_step = 1;
while stop_cond<1  	% phases
    phases_counter = phases_counter + 1;
    % this process is for the situation beta > 2;
    if phases_counter > 2*z
        total_phases_counter = total_phases_counter + phases_counter;
        % update the demand and restart the algorithm: the state need not to be reset.
        for i=1:N
            if ~isempty(SourceFlowIndex{i})
                flow_list(SourceFlowIndex{i},3) = flow_list(SourceFlowIndex{i},3)*2; 
            end
        end
        phases_counter = 1;
    end
    b_full_iter = 1;
    for j=1:N		% iterations
        if stop_cond >= 1
            b_full_iter = 0;    % TEST
            break;
        elseif isempty(SourceFlowIndex{j})
            continue;
        end
%        iterations_counter = iterations_counter + 1;
        SingleSourceFlow = flow_list(SourceFlowIndex{j},:);
        n_flow = size(SingleSourceFlow,1);
        src = SingleSourceFlow(1,1);
        b_full_step = 0;
        while stop_cond<1	% steps
            total_steps_counter = total_steps_counter + 1;
            %% construct a short path tree with common source node by dijksta algorithm
            %  If a flow is defined as (source, destination) pair, it is equivalent to
            %  organize these flows with common destination node. If a flow is defined as
            %  (source set, destination), it only can be organized by common destination.
            dest_list = SingleSourceFlow(:,2);
            % construct adjacent matrix with length function
            Gl = Gl0;
            for i=1:L
                Gl(graph.Head(i),graph.Tail(i)) = le(i);
            end
            % path_list corresponds to dest_list/src_list            
            path_list = Graph.DijkstraTree(Gl,src,dest_list); 
            f = zeros(L,1);     % total flow amount to be routed on each edge;
            for i=1:n_flow
                p = path_list(i);    % =>dest_set(i) => flow(i)
                for r=1:length(p)-1
                    ei = graph.Inverse(p(r),p(r+1));
                    f(ei) =f(ei) + SingleSourceFlow(i,3);  % accumulate flow;
                end
            end
            %% route part of the residual flow;
            sigma = max([1; f./graph.Capacity]);
			path_set.Add(path_list, SingleSourceFlow(:,3)/sigma);
            SingleSourceFlow(:,3) = SingleSourceFlow(:,3)*(1-1/sigma);
            f = f/sigma;
            %% update length function
            le = le.*(1+epsilon*f./(graph.Capacity));
            stop_cond = dot(graph.Capacity,le);
            %% update residual flow set
            t = find(abs(SingleSourceFlow(:,3))>eps);
            n_flow = length(t);
            if n_flow == 0   % all flow have been routed
                b_full_step = 1;
                break;
            end
            SingleSourceFlow = SingleSourceFlow(t,:);
        end %end steps
    end	% end iterations
	if b_full_iter && b_full_step
		path_set_temp.Copy(path_set);
	end
end	% end phases
%% ASSERTION
flow_list(:,3) = origin_flow;
flow_bandwidth = path_set.MultiplyBandwidth(1/z);
lambda = zeros(1, K);
for k = 1:K
    src = flow_list(k,1);
	dest = flow_list(k,2);
    % total flow amount of a flow is total flow at its source.
    lambda(k) = flow_bandwidth(src,dest)/flow_list(k,3);
end
lambda_fptas(1) = min(lambda);
fprintf('\tFPTAS objective value: λ = %.4f. (%.1f-approximate)\n', lambda_fptas(1), 1+w);

%% i-FPTAS
if ~(b_full_iter && b_full_step)
	path_set.Copy(path_set_temp);
end
%% calculate flow amount on each edge;
edge_bandwidth = path_set.CountEdgeFlow(graph, 1/z);
g = min(graph.Capacity./edge_bandwidth);
flow_bandwidth = path_set.MultiplyBandwidth(g);
lambda = zeros(1, K);
for k = 1:K
    src = flow_list(k,1);
	dest = flow_list(k,2);
    % total flow amount of a flow is total flow at its source.
    lambda(k) = flow_bandwidth(src,dest)/flow_list(k,3);
end
lambda_fptas(2) = min(lambda);
fprintf('\tGap to reach the capacity constraint: t = %.4f\n', g);
fprintf('\tApproximate objective value: λ = %.4f. (Modified)\n', lambda_fptas(2));

