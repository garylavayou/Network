function [ FlowTable ] = hierarchical_flow_generator(this, seed, inter_flow_density, b_intra_flow, intra_flow_density)
% randomly generate inter-domain flows between cluster nodes and intra-domain flows between nodes in one cluster.
% inter_flow_density: controls the number of inter-flow originated from each node. this argument can
% 	be a scalar or an (n*1) matrix to denote the traffic density. 
% b_intra_flow: true, intra-domain flow is enabled.
% intra_flow_density: controls the number of intra-flow originated from each node. this argument can
% 	be a scalar or an (n*1) vector to denote each cluster's traffic density.
%  if flow_density(inter/intra) < 1, the number of flows is proportional to the number of nodes;
%  otherwise, flow_density is the number of flows originated from each node.

demand_max = 10;            % tunable parameter
demand_min = 1;

graph = this.graph;
FlowTable = zeros(graph.Size*graph.Size,4);
FlowTable(:,4) = (1:graph.Size*graph.Size)';

rng(seed);
K = 0;
%% generate inter-domain traffic
if nargin == 2
	inter_flow_density = ones(this.cluster_number, 2)*(-1);
else
	if numel(inter_flow_density) == 1
		inter_flow_density = repmat(inter_flow_density, this.cluster_number-1, 2);
	elseif size(inter_flow_density,1)==1 && size(inter_flow_density,2)==2
		inter_flow_density = repmat(inter_flow_density, this.cluster_number-1, 1);
	elseif size(inter_flow_density,1)==2 && size(inter_flow_density,2)==1
		inter_flow_density = repmat(inter_flow_density', this.cluster_number-1, 1);
	end
end
core_size = this.cluster_size(1);
for i = 2:this.cluster_number
	for j = 1:this.cluster_size(i)
		node_id = sum(this.cluster_size(1:i-1))+j;		% level 2 cluster node id
		% downlink
		if inter_flow_density(i,1) == -1
			core_peer = unique(randi(core_size,core_size,1)); 
			% core_peer is different from node_id;
			num_core_peer = length(core_peer);
		elseif inter_flow_density(i,1) == 0
			continue;
		else
			if inter_flow_density(i,1) < 1
				num_core_peer = ceil(core_size*inter_flow_density(i,1));
			else
				num_core_peer = min(inter_flow_density(i,1), core_size);
			end
			% core_peer is different from node_id;
			core_peer = unique_randi(core_size, num_core_peer);
		end
		FlowTable(K+1:K+ld,1)=core_peer;
		FlowTable(K+1:K+ld,2)=node_id;
		FlowTable(K+1:K+ld,3)=round(unifrnd(down_demand_min,down_demand_max,num_core_peer,1));
		K = K+ld;
		%% uplink
		if inter_flow_density(i,2) == -1
			core_peer = unique(randi(core_size,core_size,1)); 
			% core_peer is different from node_id;
			num_core_peer = length(core_peer);
		elseif inter_flow_density(i,1) == 0
			continue;
		else
			if inter_flow_density(i,2) < 1
				num_core_peer = ceil(core_size*inter_flow_density(i,2));
			else
				num_core_peer = min(inter_flow_density(i,2), core_size);
			end
			% core_peer is different from node_id;
			core_peer = unique_randi(core_size, num_core_peer);
		end
		FlowTable(K+1:K+ld,1)=node_id;
		FlowTable(K+1:K+ld,2)=core_peer;
		FlowTable(K+1:K+ld,3)=round(unifrnd(up_demand_min,up_demand_max,num_core_peer,1));
		K = K+ld;
	end
end
%% generate intra-domain traffic
if nargin <= 3
	FlowTable = FlowTable(1:K,:);
	return;
end
if b_intra_flow == false
	FlowTable = FlowTable(1:K,:);
	return;
end
if nargin == 4
	intra_flow_density = zeros(this.cluster_number, 2);
elseif numel(intra_flow_density) == 1
	intra_flow_density = repmat(intra_flow_density, this.cluster_number, 1);
end
% core intra-domain traffic
% dest_offset = 0;
for i = 1:this.cluster_number
	for j = 1:this.cluster_size(i)
		node_id = sum(this.cluster_size(1:i-1))+j;
		if intra_flow_density(i) == -1
			dest = sum(this.cluster_size(1:i-1)) + unique(randi(this.cluster_size(i),this.cluster_size(i),1)); 
			dest(dest==node_id)=[];    % exclude the same src and dest.
			ld = length(dest);
		elseif intra_flow_density(i) == 0
			continue;
		else
			if intra_flow_density(i) < 1
				nd = ceil(this.cluster_size(i)*intra_flow_density(i))+1;
			else
				nd = min(intra_flow_density(i), this.cluster_size(i))+1;
			end
			% core_peer is different from node_id;
			dest = unique_randi(this.cluster_size(i), nd);
			dest(dest==node_id)=[];    % exclude the same src and dest.
			ld =length(dest);
			if ld == nd
				dest(randi(ld,1)) = [];
				ld = ld - 1;
			end
		end
		FlowTable(K+1:K+ld,1)=node_id;
		FlowTable(K+1:K+ld,2)=dest;
		if i == 1
			FlowTable(K+1:K+ld,3)=round(unifrnd(intra_demand_min0,intra_demand_max0,ld,1));
		else
			FlowTable(K+1:K+ld,3)=round(unifrnd(intra_demand_min,intra_demand_max,ld,1));
		end
		K = K+ld;		
	end
% dest_offset = dest_offset + this.cluster_size(i);
end
	
FlowTable = FlowTable(1:K,:);
end