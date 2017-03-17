%% Create a Traffic Matrix for the Network
%% Method Declaration
%   function flow_list = CreateTrafficMatrix( this, seed, inter_density, b_intra, intra_density)
function flow_list = CreateTrafficMatrix( this, seed, inter_density, b_intra, intra_density)

core_size = this.UpperSize;
backhaul_size = this.BottomSize;
flow_list = zeros(backhaul_size*core_size*2,4);
flow_list(:,4) = (1:(size(flow_list,1)))';

rng(seed);
K=0;

%% create inter-domain traffic
if nargin == 2
	inter_density = ones(1, 2)*(-1);
else
	if numel(inter_density) == 1
		inter_density = ones(1, 2)*inter_density;
    elseif ~(isvector(inter_density) && length(inter_density)==2)
        error('error: Input argument INTER_DENSITY has invalid size.');
	end
end
for j = 1:backhaul_size
    node_id = core_size+j;
    % downlink
    if inter_density(1) == -1
        core_peer = unique(randi(core_size,core_size,1));
        num_core_peer = length(core_peer);
    elseif inter_density(1) == 0
        continue;
    else
        if inter_density(1) < 1
            num_core_peer = ceil(core_size*inter_density(1));
        else
            num_core_peer = min(inter_density(1), core_size);
        end
        core_peer = unique_randi(core_size, num_core_peer);
    end
    flow_list(K+1:K+num_core_peer,1)=core_peer;
    flow_list(K+1:K+num_core_peer,2)=node_id;
    flow_list(K+1:K+num_core_peer,3)=...
        round(unifrnd(TwoLayerNetwork.DOWN_DEMAND_MIN,TwoLayerNetwork.DOWN_DEMAND_MAX,...
        num_core_peer,1));
    K = K+num_core_peer;
    %% uplink
    if inter_density(2) == -1
        core_peer = unique(randi(core_size,core_size,1));
        num_core_peer = length(core_peer);
    elseif inter_density(2) == 0
        continue;
    else
        if inter_density(2) < 1
            num_core_peer = ceil(core_size*inter_density(2));
        else
            num_core_peer = min(inter_density(2), core_size);
        end
        core_peer = unique_randi(core_size, num_core_peer);
    end    
    flow_list(K+1:K+num_core_peer,1)=node_id;
    flow_list(K+1:K+num_core_peer,2)=core_peer;
    flow_list(K+1:K+num_core_peer,3)=...
        round(unifrnd(TwoLayerNetwork.UP_DEMAND_MIN,TwoLayerNetwork.UP_DEMAND_MAX,...
        num_core_peer,1));
    K = K+num_core_peer;
end
%% create intra-domain traffic
if nargin <= 3 || isempty(b_intra) || b_intra == false
	flow_list = flow_list(1:K,:);
	return;
end
if nargin == 4
	intra_density = 0;
end

for i = 1:core_size
    node_id = i;
    if intra_density == -1
        dest = unique(randi(core_size, core_size,1));
        dest(dest==node_id)=[];    % exclude the same src and dest.
        ld = length(dest);
    elseif intra_density == 0
        continue;
    else
        %% Randomly choose flow target
        % We first select one more target nodes, in case that a seleced target node is the
        % source node. If source node is not selected, then we remove one node from the
        % target node set.
        %
        % To gurantee the normal running of |unique_randi|, the |nd| should not exceed
        % |graph_size|.
        if intra_density < 1
            nd = min(ceil(core_size*intra_density)+1, core_size);
        else
            nd = min(intra_density+1, core_size);
        end
        % core_peer is different from node_id;
        dest = unique_randi(core_size, nd);
        dest(dest==node_id)=[];    % exclude the same src and dest.
        ld =length(dest);
        if ld == nd
            dest(randi(ld,1)) = [];
            ld = ld - 1;
        end
    end
    flow_list(K+1:K+ld,1)=node_id;
    flow_list(K+1:K+ld,2)=dest;
    flow_list(K+1:K+ld,3)=round(unifrnd(TwoLayerNetwork.INTRA_DEMAND_MIN0,...
        TwoLayerNetwork.INTRA_DEMAND_MAX0,ld,1));
    K = K+ld;
end
flow_list = flow_list(1:K,:);
end