%% MFODA-TE LP relaxation method for MFODA-TE problem
% LP relaxation method

%% Function declaration
%   function [theta, a] = MFODA_TE( this, flow_table_index, w )
function [theta, a] = MFODA_TE( this, traffic_matrix_index, w )
 
%% Initialization
graph = this.graph;
flow_list = this.TrafficProfiles{traffic_matrix_index};
K = size(flow_list,1);
L = length(graph.Capacity);
BS_index = this.UpperSize + (1:this.BottomSize);
core_index = 1:this.UpperSize;
DGW_identifier = (find(sum(graph.Adjacent(core_index, BS_index),2)))';
n_BS = length(BS_index);
MAX_DGW = DGW_identifier(end);

%%% Linear programing
% Find path-flow variables by i-FPTAS with graph and flow_list.
[lambda, path_set] = this.MaxConcurrentFlowPath(w, traffic_matrix_index);
path_set.MultiplyBandwidth(1/lambda(2));

%% Rounding the Linear Program solution
% Find a feasible solution for the MFODA-TE problem.
b_processd_flow = zeros(K, 1);
residual_capacity = graph.Capacity;
a = zeros(this.UpperSize, K);
flow_group_d = cell(n_BS,1);
flow_group_u = cell(n_BS,1);
for j=1:n_BS
    BS_j = BS_index(j);
    flow_group_d{j} = find(flow_list(:,2)==BS_j);
    flow_group_u{j} = find(flow_list(:,1)==BS_j);
end
visited_index_d = ones(n_BS,1);
visited_index_u = ones(n_BS,1);
while ~isempty(find(b_processd_flow == 0,1))
    for j = 1:n_BS
%         BS_j = BS_index(j);
        for s = 1:2
            %% Flow selection method
            % Each time, we select one BS in a (weighted) round robin method, and then we
            % select one unassociated flow $k$ from the selected BS.
            % * If the weighted round robin method is used, the weight of a BS $i$ can be
            % calculated according to its aggregated demand, take downlink as an example:
            % $w(i)=\sum_{k:t(k)=i}{d(k)}$.
            %
            % Each time we select an unassociated flow respectively for uplink and
            % downlink.
            k = [];
            if s==1 && visited_index_d(j) <= length(flow_group_d)
                % find an unprocessed downlink flow
                k = flow_group_d{j}(visited_index_d(j));
                visited_index_d(j) = visited_index_d(j) + 1;
            end
            if s==2 && visited_index_u(j) <= length(flow_group_u)
                % find an unprocessed uplink flow
                k = flow_group_u{j}(visited_index_u(j));
                visited_index_u(j) = visited_index_u(j) + 1;
            end
            %% The path selection method
            % The path selection for flow is vital to guarantee the quality of the
            % rounding process. Our method considers using all paths from the flow's
            % source to a common D-GW to serves the flow. We select the set of paths that
            % has largest aggregated bandwidth to serve the flow. At the same time the
            % serving D-GW is also determined. 
            % * If the aggregated bandwidth is not enough to serve the flow, the path-flow
            % variables are increased with a factor, such that the flow demand is meeted,
            % unde the condition that the link capacity constraint is not violated.
            % * If the link capacity constraint cannot meet, this flow cannot be admit
            % (actually we can try the set of flows with second large aggregate
            % bandwidth). And a feasible solution of the original problem cannot be found.
            % * We can use a linear program to find how to increase the bandwidth of the
            % paths that will serve the flow. The problem can be formulated as
            %
            % $$
            % \begin{array}{rl}
            % \min &~ \theta \\
            % {\rm s.t.} &\sum\limits_{p\in\mathcal{P}_e}{y(p)}\le\theta c(e), \forall e\in\mathcal{L},\\
            % &\sum\limits_{p\in\mathcal{P}_k}{y(p)\ge d(k)},\\
            % &y(p)\ge 0, ~\forall p\in\mathcal{P},\\
            % & 0\le\theta\le 1,
            % \end{array}
            % $$
            %
            if ~isempty(k)
                % DGW_traffic(DGW_i) is the total traffic of flow k that passes through
                %     DGW_i.  
                DGW_traffic = zeros(this.UpperSize,1);
                % DGW_path_set(DGW_i) is the path set that serves flow i and passes
                %     through DGW_i
                DGW_path_index_set = cell(this.UpperSize,1);
                src = flow_list(k,1);
                dest = flow_list(k,2);
                num_path = length(path_set.Element{src,dest});
                for path_index = 1:num_path
                    node_list = path_set.Element{src,dest}(path_index).node_list;
                    % we find the D-GW that is closet to the BS.
                    %   since the serving D-GW is the first core node on the path, we can
                    %       perform the following operation to find the serving D-GW.
                    if s == 1
                        i = find(node_list<=MAX_DGW,1,'last');
                    else
                        i = find(node_list<=MAX_DGW,1);
                    end
                    DGW_i = node_list(i);
                    DGW_traffic(DGW_i) = DGW_traffic(DGW_i) + ...
                        path_set.Element{src,dest}(path_index).bandwidth;
                    % store the index of paths that pass through DGW_i;
                    DGW_path_index_set{DGW_i} = [DGW_path_index_set{DGW_i} path_index];
                end
                if num_path ~= 1
                    [~, ix] = sort(DGW_traffic, 'descend');
                else
                    ix = DGW_i;
                end
                b_admit = 0;
                for i = ix'
                    if DGW_traffic(i) == 0
                        break;
                    end
                    alpha = flow_list(k,3)/DGW_traffic(i);
                    fe = zeros(L,1);    % count the flow amount of flow k on each edge.
                    for path_index=DGW_path_index_set{i}
                        node_list = path_set.Element{src,dest}(path_index).node_list;
                        for r = 1:(length(node_list)-1)
                            ei = graph.Inverse(node_list(r),node_list(r+1));
                            fe(ei) = fe(ei) + path_set.Element{src,dest}(path_index).bandwidth;
                        end
                    end
                    if isempty(find(residual_capacity < alpha*fe, 1))
                        a(i,k) = 1;
                        if num_path ~= 1
                            for path_index=DGW_path_index_set{i}
                                path_set.Element{src,dest}(path_index).MultiplyBandwidth(alpha);
                            end
                        end
                        residual_capacity = residual_capacity - fe*alpha;
                        b_admit = 1;
                        break;
                    else
                        for path_index=DGW_path_index_set{i}
                            path_set.Element{src,dest}(path_index).bandwidth = 0;
                        end
                    end
                end
                if b_admit == 0
                    warning('a flow %d(%d,%d) cannot be admmited.',...
                        k, src, dest);
                end
                b_processd_flow(k) = 1;
            end
        end
    end
end
theta = max((graph.Capacity-residual_capacity)./graph.Capacity);
end

