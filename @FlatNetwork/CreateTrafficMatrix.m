%% CreateTrafficMatrix Inherited from class Network
% Randomly generate flow between any node pairs.
%% Function Declaration
%   function flow_list = CreateTrafficMatrix( this, seed, traffic_density )
%
% |traffic_density|: controls the number of flow originated from each node .
%
%    If traffic_density < 1, the number of flow is proportional to the number of nodes;
%    Otherwise, traffic_density is the number of flows originated from each node.
function flow_list = CreateTrafficMatrix( this, seed, traffic_density )
graph_size = this.Size;
flow_list = zeros(graph_size*graph_size,4);
flow_list(:,4) = (1:graph_size*graph_size)';

rng(seed);
K = 0;
for i = 1:graph_size
    % TODO: number of dest can be set mannualy.
    if nargin >=3
        %% Randomly choose flow target
        % We first select one more target nodes, in case that a seleced target node is the
        % source node. If source node is not selected, then we remove one node from the
        % target node set.
        %
        % To gurantee the normal running of |unique_randi|, the |nd| should not exceed
        % |graph_size|.
        if traffic_density < 1
            nd = min(ceil(graph_size*traffic_density)+1, graph_size);
        else
            nd =min(traffic_density+1, graph_size);
        end
        dest = unique_randi(graph_size, nd);
        dest(dest==i)=[];    % exclude the same src and dest.
        ld =length(dest);
        if ld == nd
            dest(randi(ld,1)) = [];
            ld = ld - 1;
        end
    else
        dest = unique(randi(graph_size,graph_size,1));
        ld =length(dest);
    end

	flow_list(K+1:K+ld,1)=i;
    flow_list(K+1:K+ld,2)=dest;
    flow_list(K+1:K+ld,3)=...
        round(unifrnd(FlatNetwork.DEMAND_MIN,FlatNetwork.DEMAND_MAX,ld,1));
    K = K+ld;
end
flow_list = flow_list(1:K,:);
end

