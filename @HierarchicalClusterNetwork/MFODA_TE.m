function [theta, a] = MFODA_TE( this, flow_table_index, w )
%MFODA_TE LP relaxation method for MFODA-TE problem.
%   Detailed explanation goes here
%% Initialization
graph = this.graph;
flow_table = this.flow_table_list{flow_table_index};
K = size(flow_table,1);
L = length(graph.Capacity);
BS_index = this.CoreSize + (1:this.BackhaulSize);
core_index = 1:this.CoreSize;
DGW_index = (find(sum(graph.Adjacent(core_index, BS_index),2)))';
n_DGW = length(DGW_index);
n_BS = length(BS_index);

%% find path-flow variables by i-FPTAS with graph and flow_table.
[lambda, path_set] = max_con_flow_fptas_path(w, graph, flow_table);
path_set.MultiplyBandwidth(1/lambda(2));

%% Rounding the Linear Program solution
b_processd_flow = zeros(K, 1);
r_capacity = graph.Capacity;
a = zeros(this.CoreSize, K);
flow_group_d = cell(n_BS,1);
flow_group_u = cell(n_BS,1);
for j=1:n_BS
    BS_j = BS_index(j);
    flow_group_d{j} = find(flow_table(:,2)==BS_j);
    flow_group_u{j} = find(flow_table(:,1)==BS_j);
end
visited_index_d = ones(n_BS,1);
visited_index_u = ones(n_BS,1);
while ~isempty(find(b_processd_flow == 0,1))
    for j = 1:n_BS
%         BS_j = BS_index(j);
        for s = 1:2
            k = [];
            if s==1 && visited_index_d(j) <= length(flow_group_d)
                % find an unprocessed downlink flow
                k = flow_group_d{j}(visited_index_d(j));
                visited_index_d(j) = visited_index_d(j) + 1;
            elseif visited_index_u(j) <= length(flow_group_u)
                % find an unprocessed downlink flow
                k = flow_group_u{j}(visited_index_u(j));
                visited_index_u(j) = visited_index_u(j) + 1;
            end
            if ~isempty(k)
                d = zeros(n_DGW,1);
                dp = cell(n_DGW,1);
                src = flow_table(k,1);
                dest = flow_table(k,2);
                num_path = length(path_set.Element{src,dest});
                for p = 1:num_path
                    node_list = path_set.Element{src,dest}(p).node_list;
                    for i = 1:n_DGW
                        DGW_i = DGW_index(i);
                        if ~isempty(find(node_list==DGW_i,1))
                            d(i) = d(i) + path_set.Element{src,dest}(p).bandwidth;
                            dp{i} = [dp{i} p];  %record the sub set of path(id);
                            break
                        end
                    end
                end
                [~, ix] = sort(d, 'descend');
                b_admit = 0;
                for i = ix'
                    if d(i) == 0
                        break;
                    end
                    alpha = flow_table(k,3)/d(i);
                    fe = zeros(L,1);
                    for p=dp{i}
                        node_list = path_set.Element{src,dest}(p).node_list;
                        for r = 1:(length(node_list)-1)
                            ei = graph.Inverse(node_list(r),node_list(r+1));
                            fe(ei) = fe(ei) + path_set.Element{src,dest}(p).bandwidth;
                        end
                    end
                    if isempty(find(~(r_capacity >= alpha*fe), 1))
                        a(i,k) = 1;
                        for p=dp{i}
                            path_set.Element{src,dest}(p).MultiplyBandwidth(alpha);
                        end
                        r_capacity = r_capacity - fe*alpha;
                        b_admit = 1;
                        break;
                    else
                        for p=dp{i}
                            path_set.Element{src,dest}(p).bandwidth = 0;
                        end
                    end
                end
                if b_admit == 0
                    warning('warning: A flow %d(%d,%d) cannot be admmited.',...
                        k, src, dest);
                end
                b_processd_flow(k) = 1;
            end
        end
    end
end
theta = max((graph.Capacity-r_capacity)./graph.Capacity);
end

