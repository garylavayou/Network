function [theta, b_d, b_u] = MBODA_TE(this, traffic_matrix_index, w)
%% Initilization
graph = this.graph;
flow_list = this.TrafficProfiles{traffic_matrix_index};

% only nodes connect to RAN is D-GW
core_index = 1:this.UpperSize;
BS_index = this.UpperSize + (1:this.BottomSize);
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
% flow_list entry with value (j,t,d) denote flow from BS j to target serving node t with
%      demand d.
bs_demand_d = zeros(this.Size, 1);      % downlink aggregated demand, ther former this.UpperSize elements is not used.
bs_demand_u = zeros(this.Size, 1);      % uplink aggregated demand
backhaul_demand = zeros(this.Size);
for j = BS_index
    bs_demand_d(j) = sum(flow_list(flow_list(:,2)==j, 3));
    bs_demand_u(j) = sum(flow_list(flow_list(:,1)==j, 3));
end

%% determine the shortest path from D-GW to the reachable BSs

% sp_list_d = cell(n_DGW,1);
% sp_list_u = cell(n_DGW,1);
sp_node_list_d = cell(n_DGW,1);
sp_node_list_u = cell(n_DGW,1);
sp_cap_list_d = cell(n_DGW,1);
sp_cap_list_u = cell(n_DGW,1);

for i = 1:n_DGW
    modified_adj = graph.Adjacent;
    modified_adj(core_index,core_index) = 0;
    DGW_i = DGW_index(i);
    ci =core_index;
    ci(DGW_i) = [];
    modified_adj(ci,BS_index) = 0;
    modified_adj(BS_index,ci) = 0;
    [subgraph, subgraph_index] = Graph.SubGraph(modified_adj, DGW_i);
    % downlink (D-GW to BS) shortest path
    subgraph = 1./subgraph;
    for diag_i = 1:length(subgraph_index)
        subgraph(diag_i, diag_i) = 0;
    end
    [path_list, node_list] = Graph.ShortestPathTree(subgraph, 1, 0);
    sp_node_list_d{i} = subgraph_index(node_list);
    sp_cap_list_d{i} = zeros(length(sp_node_list_d{i}),1);
    sp_cap_list_d{i}(1) = inf;  % source-source;
%     sp_list_d{i} = cell(length(node_list),1); % first element is source, not used
    for di = 2:length(sp_node_list_d{i})
        path = subgraph_index(path_list(di));
%         sp_list_d{i}{di} = path;
        cap = inf;
        for r = 1:length(path)-1
            cap = min(cap, modified_adj(path(r),path(r+1)));
        end
        sp_cap_list_d{i}(di) = cap;
    end
    [path_list, node_list] = Graph.ShortestPathTree(subgraph, 1, 1);
    sp_node_list_u{i} = subgraph_index(node_list);
    sp_cap_list_u{i} = zeros(length(sp_node_list_u{i}),1);
    sp_cap_list_u{i}(1) = inf;  % source-source;
%     sp_list_u{i} = cell(length(node_list),1); % first element is source, not used
    for di = 2:length(sp_node_list_u{i})
        path = subgraph_index(path_list(di));
%         sp_list_u{i}{di} = path;
        cap = inf;
        for r = 1:length(path)-1
            cap = min(cap, modified_adj(path(r),path(r+1)));
        end
        sp_cap_list_u{i}(di) = cap;
    end
end

%% Downlink D-GW BS Association
% residual weight
w_d = w_d/(min(w_d));
r = ones(n_DGW,1).*w_d;
b_associate_bs = zeros(this.Size,1);
% downlink D-GW BS Association Variable
%     ther former [this.UpperSize] column of elements is not used.
b_d = zeros(this.UpperSize, this.Size); 
visited_index = ones(n_DGW,1)+1;
b_augment = 0;
while sum(b_associate_bs) < this.BottomSize
    b_succeed = 0;                  % Flag indicates if some BSs have associated to D-GW
    temp_demand = zeros(n_DGW,1);
    for i = 1:n_DGW
        DGW_i = DGW_index(i);
        for vi = visited_index(i):length(sp_node_list_d{i})
            % there is unvisited BS for D-GW i
            j = sp_node_list_d{i}(vi);
            if b_associate_bs(j) == 0 && bs_demand_d(j) <= sp_cap_list_d{i}(vi);
                if r(i) >= bs_demand_d(j) 
                    b_d(DGW_i,j) = 1;
                    backhaul_demand(DGW_i,j) = bs_demand_d(j);
                    r(i) = r(i) - bs_demand_d(j);
                    b_succeed = 1;
                    b_augment = 0;
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
        if ~b_augment
            index = temp_demand>0;
            cap = (temp_demand - r)./w_d;
            beta = mean(cap(index));
            r = r + w_d.*beta;
            b_augment = 1;
        else
            %% TO BE EXAMINED
            un_associate_bs = find(b_associate_bs==0);
            un_associate_bs = un_associate_bs(un_associate_bs>this.UpperSize);
            for j = un_associate_bs
                associate_candidate = zeros(n_DGW,2);
                n_associate_candidate = 0;
                for i = 1:n_DGW
                    dj = find(sp_node_list_d{i}==j,1);
                    if ~isempty(dj)
                        n_associate_candidate = n_associate_candidate + 1;
                        DGW_i = DGW_index(i);
                        associate_candidate(n_associate_candidate) = ...
                            [DGW_i sp_cap_list_d{i}(dj)];
                    end
                end
                if n_associate_candidate == 0
                    error('Error: cannot find a available D-GW.');
                end
                [~, associate_i] = max(associate_candidate(1:n_associate_candidate,2));
                DGW_i = DGW_i(associate_i);
                b_d(DGW_i,j) = 1;
                backhaul_demand(DGW_i,j) = bs_demand_d(j);
                b_associate_bs(j) = 1;
            end
        end
    end
end
%% Uplnk D-GW BS Association
% residual weight
w_u = w_u/(min(w_u));
r = ones(n_DGW,1).*w_u;
b_associate_bs = zeros(this.Size,1);
% uplink BS D-GW Association Variable
%     ther former [this.UpperSize] column of elements is not used.
b_u = zeros(this.Size, this.UpperSize);
visited_index = ones(n_DGW,1)+1;
b_augment = 0;
while sum(b_associate_bs) < this.BottomSize
    b_succeed = 0;                  % Flag indicates if some BSs have associated to D-GW
    temp_demand = zeros(n_DGW,1);
    for i = 1:n_DGW
        DGW_i = DGW_index(i);
        for vi = visited_index(i):length(sp_node_list_u{i})
            % there is unvisited BS for D-GW i
            j = sp_node_list_u{i}(vi);
            if b_associate_bs(j) == 0 && bs_demand_u(j) <= sp_cap_list_u{i}(vi);
                if r(i) >= bs_demand_u(j)
                    b_u(j,DGW_i) = 1;
                    backhaul_demand(j,DGW_i) = bs_demand_u(j);
                    r(i) = r(i) - bs_demand_u(j);
                    b_succeed = 1;
                    b_augment = 0;
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
        if ~b_augment
            index = temp_demand>0;
            cap = (temp_demand - r)./w_u;
            beta = mean(cap(index));
            r = r + w_u.*beta;
            b_augment = 1;
        else
            un_associate_bs = find(b_associate_bs==0);
            un_associate_bs = un_associate_bs(un_associate_bs>this.UpperSize);
            for j = un_associate_bs
                associate_candidate = zeros(n_DGW,2);
                n_associate_candidate = 0;
                for i = 1:n_DGW
                    dj = find(sp_node_list_u{i}==j,1);
                    if ~isempty(dj)
                        n_associate_candidate = n_associate_candidate + 1;
                        DGW_i = DGW_index(i);
                        associate_candidate(n_associate_candidate) = ...
                            [DGW_i sp_cap_list_u{i}(dj)];
                    end
                end
                if n_associate_candidate == 0
                    error('Error: cannot find a available D-GW.');
                end
                [~, associate_i] = max(associate_candidate(1:n_associate_candidate,2));
                DGW_i = DGW_i(associate_i);
                b_u(j,DGW_i) = 1;
                backhaul_demand(j,DGW_i) = bs_demand_u(j);
                b_associate_bs(j) = 1;   
            end
        end
    end
end

%% Core network traffic engineering
core_graph = Graph();
core_graph.SetAdjacent(graph.Adjacent(core_index,core_index));
core_demand = zeros(this.UpperSize);
%% count downlink traffic from BD-GW to D-GW
for t=core_index
    for i =1:n_DGW
        DGW_i = DGW_index(i);
        if t == DGW_i       % the aggregated flow has the BD-GW and D-GW at the same node.
            continue;
        end
        dest_BS_index = flow_list(flow_list(:,1)==t,2);
        core_demand(t,DGW_i) = dot(b_d(DGW_i,dest_BS_index),flow_list(dest_BS_index,3));
    end
end
%% count uplink traffic from D-GW to BD-GW
for i=1:n_DGW
    DGW_i = DGW_index(i);
    for t = core_index
        if t == DGW_i       % the aggregated flow has the BD-GW and D-GW at the same node.
            continue;
        end
        src_BS_index = flow_list(flow_list(:,2)==t,1);
        core_demand(DGW_i,t) = core_demand(DGW_i,t) + ...
            dot(b_u(src_BS_index,DGW_i),flow_list(src_BS_index,3));
    end
end
[src, dest, bandwidth] = find(core_demand);
traffic_profile = {[src, dest, bandwidth, (1:length(src))']};
network = FlatNetwork.Build(core_graph, traffic_profile);
lambda_fptas= network.MaxConcurrentFlow(w, 1);
% the result of i-FPTAS, the maximum link ultilization of core network
theta(1) = 1/lambda_fptas(2);    
%% Backhaul network traffic engineering
backhaul_graph = Graph();
Adj = this.graph.Adjacent;
Adj(core_index,core_index) = 0;
backhaul_graph.SetAdjacent(Adj);
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
traffic_profile = {[src, dest, bandwidth, (1:length(src))']};
network = FlatNetwork.Build(backhaul_graph, traffic_profile);
lambda_fptas= network.MaxConcurrentFlow(w, 1);
% the result of i-FPTAS, the maximum link ultilization of backhaul network
theta(2) = 1/lambda_fptas(2);
end