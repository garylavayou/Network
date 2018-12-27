%% Full Polynomial Time Approximation Schemes(FPTAS)
% FPTAS for multi-commodity flow problems (i.e. maximum concurrent flow problems)
% this implementation is based on the work of [Garg2007], [Karakostas2008,3.1], and
% [Fleischer2000,3]  
%% Mehtod declaration
%   [lambda, flow_variables, phases, steps, rt] = MaximizeConcurrentFlow(this, w,
%   traffic_matrix_id, arg4) 
%
% |lambda|: objective value of FPTAS and i-FPTAS.
%
% |flow_variables|: a matrix that flow_variables (k, e) represents the flow k's traffic
%     amount on edge e, edge(head, tail)], each flow's traffic amount is:
%     sum(flow_variables(k, e)), e is the out-link at source of k.
%
% If the output parameter |RT| is feeded, then this function will count the running time.
function [lambda_fptas, flow_variables, total_phases_counter, total_steps_counter, rt] = ...
    MaxConcurrentFlow(this, w, traffic_matrix_id, arg4)
fprintf('Maximum concurrent multi-commodity problem with edge-flow output:\n');
if nargout >= 5
    tic;
end
%% Graph parameters
graph = this.graph;
flow_list=this.TrafficProfiles{traffic_matrix_id};
L = length(graph.Capacity);              % number of edges;
N = size(graph.Adjacent,1);

%% Assort flows by sources(or destination)
% Assort flows by target node. 
%
%   DestFlowIndex = cell(1,N);
% for dest = 1:N
%    DestFlowIndex{dest}=find(flow_list(:,2)==dest);
% end
SourceFlowIndex = cell(1,N);
K = size(flow_list,1);
for src = 1:N
    SourceFlowIndex{src}=find(flow_list(:,1)==src);
end

%% approximation parameters
epsilon = 1-(1+w)^(-1/3);
delta =  (1+epsilon)^((epsilon-1)/epsilon)*((1-epsilon)/L)^(1/epsilon);
z = log((1+epsilon)/delta)/log(1+epsilon);

%% m-approximation of  maximum single commodity flow 
% Capacity is a non-additive property of path, so the traditional dijkstra algorithm based
% on distance which is an additive property is invalid.
%
%   xi = inf;       % estimation on maximum single_flow
%                 % xi_approx <= xi_opt <= inf
% for j=1:N		% iterations
%     if isempty(SourceFlowIndex{j})
% %         if isempty(DestFlowIndex{j})
%         continue;
%     end
%     SingleSourceFlow = flow_list(SourceFlowIndex{j},:);
%     src = SingleSourceFlow(1,1);
%     dest_list = SingleSourceFlow(:,2);
%     capacity_list = max_capacity(Gc,src,dest_list);
%     xi = min([capacity_list./SingleSourceFlow(:,3); xi]);
% end
%
% multiply the initial demand by xi/K, thus 1 <= beta <= KL.
%
%   flow_list(:,3)=flow_list(:,3)*(xi/K);
origin_flow = flow_list(:,3);

flow_variables = zeros(K, L);

%---------------------------------- FPTAS-------------------------------------------
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
% b_full_iter = 1;
% b_full_step = 1;
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
%     b_full_iter = 1;
    for j=1:N		% iterations
        if stop_cond >= 1
%             b_full_iter = 0;    % TEST
            break;
        elseif isempty(SourceFlowIndex{j})
%         elseif isempty(DestFlowIndex{j})
            continue;
        end
%         iterations_counter = iterations_counter + 1;
        SingleSourceFlow = flow_list(SourceFlowIndex{j},:);
        n_flow = size(SingleSourceFlow,1);
        src = SingleSourceFlow(1,1);
%         dest = SingleDestFlow(1,2);
%         SingleDestFlow = flow_list(DestFlowIndex{j},:);
%         n_flow = size(SingleDestFlow,1);
%         b_full_step = 0;
        while stop_cond<1	% steps
            total_steps_counter = total_steps_counter + 1;
            %% Construct a short path tree with common source node by dijkstra algorithm
            %  If a flow is defined as (source, destination) pair, it is equivalent to
            %  organize these flows with common destination node. If a flow is defined as
            %  (source set, destination), it only can be organized by common destination.
            dest_list = SingleSourceFlow(:,2);
%             src_list = SingleDestFlow(:,1);
            % construct adjacent matrix with length function
            Gl = Gl0;
            for i=1:L
                Gl(graph.Head(i),graph.Tail(i)) = le(i);
            end
            % path_list corresponds to dest_list/src_list            
            path_list = Graph.DijkstraTree(Gl,src,dest_list); 
%             path_list = dijkstra_tree(Gl,dest,src_list,1); 
%             [path_uni_set,new_path_index] = path_set_merge(path_uni_set, path_list);
% keep records of path can be time expensive as dijkstra
            fe = zeros(n_flow,L);     %  flow amount to be routed on each edge;
            for i=1:n_flow
                p = path_list(i);    % =>dest_set(i) => flow(i)
                for r=1:length(p)-1
                    ei = graph.Inverse(p(r),p(r+1));
                    fe(i,ei) =fe(i,ei) + SingleSourceFlow(i,3);  % accumulate flow;
                end
%                     f(ei) =f(ei) + SingleDestFlow(i,3);  % accumulate flow;
            end
            %% route part of the residual flow;
            f = (sum(fe,1))';
            sigma = max([1; f./graph.Capacity]);
            fe = fe/sigma;
            for i=1:n_flow
                fi = SingleSourceFlow(i,4);
                flow_variables(fi,:) =  flow_variables(fi,:) + fe(i,:);
            end
            SingleSourceFlow(:,3) = SingleSourceFlow(:,3)*(1-1/sigma);
%             SingleDestFlow(:,3) = SingleDestFlow(:,3)*(1-1/sigma);
            f = f/sigma;
            % update length function
            le = le.*(1+epsilon*f./(graph.Capacity));
            stop_cond = dot(graph.Capacity,le);
            % update residual flow set
            t = find(abs(SingleSourceFlow(:,3))>eps);
%             t = find(abs(SingleDestFlow(:,3))>eps);
            n_flow = length(t);
            if n_flow == 0   % all flow have been routed
%                 b_full_step = 1;
                break;
            end
            SingleSourceFlow = SingleSourceFlow(t,:);
%             SingleDestFlow = SingleDestFlow(t,:);
        end %end steps
    end	% end iterations
end	% end phases
% end_iter = j;

%% normalize the results
flow_list(:,3) = origin_flow;
flow_variables = flow_variables/z;
lambda = zeros(1, K);
for k = 1:K
    ei = graph.Inverse(flow_list(k,1),:);
    ei = ei(ei~=0);   % out-edges at source
    % total flow amount of a flow is total flow at its source.
    lambda(k) = sum(flow_variables(k,ei))/flow_list(k,3);
end
lambda_fptas(1) = min(lambda);
if nargout >= 5
    rt(1) = toc;
end
fprintf('\tFPTAS objective value: λ = %.4f. (%.1f-approximate)\n', lambda_fptas(1), 1+w);
if nargout >= 5
    tic;
end
for k = 1:K
    % let AF = λd
    % lambda_fptas <= lambda(k)
    flow_variables(k,:) = flow_variables(k,:)*lambda_fptas(1)/lambda(k);    
end
[gp, id] = min((graph.Capacity)'./sum(flow_variables,1));
if nargin==4 && ~isempty(find(id>arg4, 1))
    warning('Backhaul capacity is deficient.');
    fprintf('(\t link id = %d)\n', id(1));
    pause;
end
flow_variables = flow_variables*gp;
lambda_fptas(2) = lambda_fptas(1)*gp;
fprintf('\tGap to reach the capacity constraint: t = %.4f\n', gp);
fprintf('\tApproximate objective value: λ = %.4f. (Modified)\n', lambda_fptas(2));
if nargout >= 5
    rt(2) = toc;
    rt(1) = rt(2) + rt(1);
end

%%% Iteration details
if total_phases_counter == 0
    total_phases_counter = phases_counter;
end
%%%
%   fprintf('\tTotal running phases: %d\n', total_phases_counter);
% fprintf('\tLast stage total running phases: %d\n', phases_counter);
% fprintf('\tTotal running iterations: %d\n', iterations_counter);
% fprintf('\tTotal running steps: %d\n', steps_counter);
% if b_full_step == 0
%     fprintf('\tAlgorithm ends in the inner step.\n');
% elseif b_full_iter == 0
%     fprintf('\tAlgorithm ends in the %dth iteration.\n', end_iter);
% else
%     fprintf('\tAlgorithm ends with complete phase.\n');
% end
end