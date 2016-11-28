%% CreateTrafficMatrix
% randomly generate inter-domain flows between cluster nodes and core nodes, 
% and intra-domain flows between nodes in one cluster.
%% Method Declaration
%   flow_list = CreateTrafficMatrix(this, seed, inter_density, b_intra, intra_density)
%
% |inter_density|: controls the number of inter-flow originated from each node. this
%     argument can be a scalar or an (1*2) vector or an (n*2) matrix to denote the
%     uplink/downlink traffic density, where n is the number of domains to the core.  
%
% |b_intra|: true, intra-domain flow is enabled.
%
% |intra_density|: controls the number of intra-flow originated from each node. this
%     argument can be a scalar or an (n*1) vector to denote each cluster's traffic
%     density. 
%
% * If flow_density(inter/intra) < 1, the number of flows is proportional to the
%         number of nodes; 
% * Otherwise, flow_density is the number of flows originated from each node.
%
% NOTE: this function should be called after the graph has been created.
function flow_list = CreateTrafficMatrix(this, seed, inter_density, b_intra, intra_density)
up_demand_max = 10;            % tunable parameter
up_demand_min = 1;
up_down_ratio = 3;
down_demand_max = up_down_ratio*up_demand_max;
down_demand_min = up_down_ratio*up_demand_min;
intra_demand_max = 10;
intra_demand_min = 1;
intra_demand_max0 = 50;
intra_demand_min0 = 5;

graph = this.graph;
flow_list = zeros(graph.Size*graph.Size,4);
flow_list(:,4) = (1:graph.Size*graph.Size)';

rng(seed);
K = 0;
%% generate inter-domain traffic
if nargin == 2
	inter_density = ones(this.ClusterNumber, 2)*(-1);
else
	if numel(inter_density) == 1
		inter_density = repmat(inter_density, this.ClusterNumber-1, 2);
	elseif size(inter_density,1)==1 && size(inter_density,2)==2
		inter_density = repmat(inter_density, this.ClusterNumber-1, 1);
	elseif size(inter_density,1)==2 && size(inter_density,2)==1
		inter_density = repmat(inter_density', this.ClusterNumber-1, 1);
	end
end
core_size = this.ClusterSize(1);
for i = 2:this.ClusterNumber
    di = i-1;       % since only (this.ClusterNumber-1) backhaul clusters
	for j = 1:this.ClusterSize(i)
		node_id = sum(this.ClusterSize(1:i-1))+j;		% level 2 cluster node id
		% downlink
		if inter_density(di,1) == -1
			core_peer = unique(randi(core_size,core_size,1)); 
			% core_peer is different from node_id;
			num_core_peer = length(core_peer);
		elseif inter_density(di,1) == 0
			continue;
		else
			if inter_density(di,1) < 1
				num_core_peer = ceil(core_size*inter_density(di,1));
			else
				num_core_peer = min(inter_density(di,1), core_size);
			end
			% core_peer is different from node_id;
			core_peer = unique_randi(core_size, num_core_peer);
		end
		flow_list(K+1:K+num_core_peer,1)=core_peer;
		flow_list(K+1:K+num_core_peer,2)=node_id;
		flow_list(K+1:K+num_core_peer,3)=...
            round(unifrnd(down_demand_min,down_demand_max,num_core_peer,1));
		K = K+num_core_peer;
		%% uplink
		if inter_density(di,2) == -1
			core_peer = unique(randi(core_size,core_size,1)); 
			% core_peer is different from node_id;
			num_core_peer = length(core_peer);
		elseif inter_density(di,2) == 0
			continue;
		else
			if inter_density(di,2) < 1
				num_core_peer = ceil(core_size*inter_density(di,2));
			else
				num_core_peer = min(inter_density(di,2), core_size);
			end
			% core_peer is different from node_id;
			core_peer = unique_randi(core_size, num_core_peer);
		end
		flow_list(K+1:K+num_core_peer,1)=node_id;
		flow_list(K+1:K+num_core_peer,2)=core_peer;
		flow_list(K+1:K+num_core_peer,3)=...
            round(unifrnd(up_demand_min,up_demand_max,num_core_peer,1));
		K = K+num_core_peer;
	end
end
%% generate intra-domain traffic
if nargin <= 3
	flow_list = flow_list(1:K,:);
	return;
end
if b_intra == false
	flow_list = flow_list(1:K,:);
	return;
end
if nargin == 4
	intra_density = zeros(this.ClusterNumber, 1); 
elseif numel(intra_density) == 1
	intra_density = repmat(intra_density, this.ClusterNumber, 1);
end
% core intra-domain traffic
% dest_offset = 0;
for i = 1:this.ClusterNumber
	for j = 1:this.ClusterSize(i)
		node_id = sum(this.ClusterSize(1:i-1))+j;
		if intra_density(i) == -1
			dest = sum(this.ClusterSize(1:i-1)) + unique(randi(this.ClusterSize(i),this.ClusterSize(i),1)); 
			dest(dest==node_id)=[];    % exclude the same src and dest.
			ld = length(dest);
		elseif intra_density(i) == 0
			continue;
		else
			if intra_density(i) < 1
				nd = ceil(this.ClusterSize(i)*intra_density(i))+1;
			else
				nd = min(intra_density(i), this.ClusterSize(i))+1;
			end
			% core_peer is different from node_id;
			dest = unique_randi(this.ClusterSize(i), nd);
			dest(dest==node_id)=[];    % exclude the same src and dest.
			ld =length(dest);
			if ld == nd
				dest(randi(ld,1)) = [];
				ld = ld - 1;
			end
		end
		flow_list(K+1:K+ld,1)=node_id;
		flow_list(K+1:K+ld,2)=dest;
		if i == 1
			flow_list(K+1:K+ld,3)=round(unifrnd(intra_demand_min0,intra_demand_max0,ld,1));
		else
			flow_list(K+1:K+ld,3)=round(unifrnd(intra_demand_min,intra_demand_max,ld,1));
		end
		K = K+ld;		
	end
% dest_offset = dest_offset + this.ClusterSize(i);
end
	
flow_list = flow_list(1:K,:);
end