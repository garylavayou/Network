function theta = MBODA_TE(this, flow_table_index, w)
%% Initilization
graph = this.graph;
flow_table = this.flow_table_list{flow_table_index};

% only nodes connect to RAN is D-GW
core_index = 1:this.CoreSize;
BS_index = this.CoreSize + (1:this.BackhaulSize);
DGW_index = (find(sum(graph.Adjacent(core_index, BS_index),2)))';
n_DGW = length(DGW_index);

%% D-GW weight
w_d = zeros(n_DGW,1);     % downlink
w_u = zeros(n_DGW,1);     % uplink
for i = 1:n_DGW
    % downlink D-GW weight
    DGW_i = DGW_index(i);
	w_d(i) = min(sum(graph.Adjacent(DGW_i, BS_index)), sum(graph.Adjacent(core_index,DGW_i)));
    % uplink D-GW weight
	w_u(i) = min(sum(graph.Adjacent(BS_index, DGW_i)), sum(graph.Adjacent(DGW_i, core_index)));
end
% TODO: only part of the backhaul nodes is BS, while other is pure router.
% flow_table entry with value (j,t,d) denote flow from BS j to target serving node t with
%      demand d.
bs_demand_d = zeros(this.Size, 1);      % downlink aggregated demand, ther former this.CoreSize elements is not used.
bs_demand_u = zeros(this.Size, 1);      % uplink aggregated demand
backhaul_demand = zeros(this.Size);
for j = BS_index
    bs_demand_d(j) = sum(flow_table(flow_table(:,2)==j, 3));
    bs_demand_u(j) = sum(flow_table(flow_table(:,1)==j, 3));
end

%% determine the shortest path from D-GW to the reachable BSs
shortest_path_list_d = cell(n_DGW,1);
shortest_path_list_u = cell(n_DGW,1);
for i = 1:n_DGW
    DGW_i = DGW_index(i);
    subgraph_index = zeros(1, this.Size); %row vector
    n = 0;
    for j = 2:this.ClusterNumber
        node_index = sum(this.ClusterSize(1:(j-1)))+(1:this.ClusterSize(j));
        if ~isempty(find(graph.Adjacent(DGW_i,node_index), 1)) || ...
                ~isempty(find(graph.Adjacent(node_index,DGW_i), 1)) 
            subgraph_index(n+(1:this.ClusterSize(j))) = node_index;
            n = n + this.ClusterSize(j);
        end
    end
    subgraph_index = [DGW_i, subgraph_index(1:n)]; % row vector;
    subgraph = graph.Adjacent(subgraph_index, subgraph_index);
    % downlink (D-GW to BS) shortest path
    subgraph = 1./subgraph;
    for diag_i = 1:n
        subgraph(diag_i, diag_i) = 0;
    end
    [path, flag] = Graph.DijkstraPath(subgraph, 1);
    if flag
        error('error: Shortest path not found.');
    end
    shortest_path_list_d{i} = subgraph_index(path);
    [path, flag] = Graph.DijkstraPath(subgraph', 1);
    if flag
        error('error: Shortest path not found.');
    end
    shortest_path_list_u{i} = subgraph_index(path);
end

%% Downlink D-GW BS Association
% residual weight
w_d = w_d/(min(w_d));
r = ones(n_DGW,1).*w_d;
b_associate_bs = zeros(this.Size,1);
% downlink D-GW BS Association Variable
%     ther former [this.CoreSize] column of elements is not used.
b_d = zeros(this.CoreSize, this.Size); 
visited_index = ones(n_DGW,1)+1;
while sum(b_associate_bs) < this.BackhaulSize
    b_succeed = 0;                  % Flag indicates if some BSs have associated to D-GW
    temp_demand = zeros(n_DGW,1);
    for i = 1:n_DGW
        DGW_i = DGW_index(i);
        for vi = visited_index(i):length(shortest_path_list_d{i})
            % there is unvisited BS for D-GW i
            j = shortest_path_list_d{i}(vi);
            if b_associate_bs(j) == 0
                if r(i) >= bs_demand_d(j)
                    b_d(DGW_i,j) = 1;
                    backhaul_demand(DGW_i,j) = bs_demand_d(j);
                    r(i) = r(i) - bs_demand_d(j);
                    b_succeed = 1;
                    b_associate_bs(j) = 1;
                    visited_index(i) = visited_index(i) + 1;
                else
                    temp_demand(i) = bs_demand_d(j);
                end
                break;
            else
                visited_index(i) = visited_index(i) + 1;
            end
        end
    end
    if ~b_succeed
        index = temp_demand>0;
        cap = (temp_demand - r)./w_d;
        beta = mean(cap(index));
        r = r + w_d.*beta;
    end
end
%% Uplnk D-GW BS Association
% residual weight
w_u = w_u/(min(w_u));
r = ones(n_DGW,1).*w_u;
b_associate_bs = zeros(this.Size,1);
% uplink BS D-GW Association Variable
%     ther former [this.CoreSize] column of elements is not used.
b_u = zeros(this.Size, this.CoreSize);
visited_index = ones(n_DGW,1)+1;
while sum(b_associate_bs) < this.BackhaulSize
    b_succeed = 0;                  % Flag indicates if some BSs have associated to D-GW
    temp_demand = zeros(n_DGW,1);
    for i = 1:n_DGW
        DGW_i = DGW_index(i);
        for vi = visited_index(i):length(shortest_path_list_u{i})
            % there is unvisited BS for D-GW i
            j = shortest_path_list_u{i}(vi);
            if b_associate_bs(j) == 0
                if r(i) >= bs_demand_u(j)
                    b_u(j,DGW_i) = 1;
                    backhaul_demand(j,DGW_i) = bs_demand_u(j);
                    r(i) = r(i) - bs_demand_u(j);
                    b_succeed = 1;
                    b_associate_bs(j) = 1;
                    visited_index(i) = visited_index(i) + 1;
                else
                    temp_demand(i) = bs_demand_u(j);
                end
                break;
            else
                visited_index(i) = visited_index(i) + 1;
            end
        end
    end
    if ~b_succeed
        index = temp_demand>0;
        cap = (temp_demand - r)./w_u;
        beta = mean(cap(index));
        r = r + w_u.*beta;
    end
end

%% Core network traffic engineering
core_graph = Graph();
core_graph.Adjacent = this.graph.Adjacent(core_index, core_index);
[core_graph.Head, core_graph.Tail, core_graph.Capacity] = find(core_graph.Adjacent);
core_graph.Inverse = zeros(core_graph.Size);
for e=1:length(core_graph.Capacity)
    % link e from head(e) to tail(e)
    core_graph.Inverse(core_graph.Head(e),core_graph.Tail(e))=e;
end

core_demand = zeros(this.CoreSize);
%% count downlink traffic from BD-GW to D-GW
for t=core_index
    for i =1:n_DGW
        DGW_i = DGW_index(i);
        if t == DGW_i       % the aggregated flow has the BD-GW and D-GW at the same node.
            continue;
        end
        dest_BS_index = flow_table(flow_table(:,1)==t,2);
        core_demand(t,DGW_i) = dot(b_d(DGW_i,dest_BS_index),flow_table(dest_BS_index,3));
    end
end
%% count uplink traffic from D-GW to BD-GW
for i=1:n_DGW
    DGW_i = DGW_index(i);
    for t = core_index
        if t == DGW_i       % the aggregated flow has the BD-GW and D-GW at the same node.
            continue;
        end
        src_BS_index = flow_table(flow_table(:,2)==t,1);
        core_demand(DGW_i,t) = core_demand(DGW_i,t) + ...
            dot(b_u(src_BS_index,DGW_i),flow_table(src_BS_index,3));
    end
end
[src, dest, bandwidth] = find(core_demand);
lambda_fptas= max_con_flow_fptas(w, core_graph, [src, dest, bandwidth, (1:length(src))']);
% the result of i-FPTAS, the maximum link ultilization of core network
theta(1) = 1/lambda_fptas(2);    
%% Backhaul network traffic engineering
backhaul_graph = Graph;
backhaul_graph.Adjacent = this.graph.Adjacent;
backhaul_graph.Adjacent(core_index,core_index) = 0;
[backhaul_graph.Head, backhaul_graph.Tail, backhaul_graph.Capacity] = find(backhaul_graph.Adjacent);
backhaul_graph.Inverse = zeros(backhaul_graph.Size);
for e=1:length(backhaul_graph.Capacity)
    % link e from head(e) to tail(e)
    backhaul_graph.Inverse(backhaul_graph.Head(e),backhaul_graph.Tail(e))=e;
end% backhaul_demand = zeros(this.Size);
% %% count downlink traffic from D-GW to BS
% for i=core_index
%     for j =BS_index
%         backhaul_demand(i,j) = b_d(i,j)*bs_demand_d(j);
%     end
% end
% %% count uplink traffic from BS to D-GW
% for j=BS_index
%     for i =core_index
%         backhaul_demand(j,i) = b_u(j,i)*bs_demand_u(j);
%     end
% end
[src, dest, bandwidth] = find(backhaul_demand);
lambda_fptas= max_con_flow_fptas(w, backhaul_graph, [src, dest, bandwidth, (1:length(src))']);
% the result of i-FPTAS, the maximum link ultilization of backhaul network
theta(2) = 1/lambda_fptas(2);
end