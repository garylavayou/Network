%% ConvergeNetwork
%   ConvergeNetwork < Network
%
% To use the converge model, node weight must be specified in the graph.
classdef ConvergeNetwork < FlatNetwork
    properties (Constant)
        DEMAND_MAX = 10;            % tunable parameter
        DEMAND_MIN = 1;
    end
    %% Inherited Methods
    methods
        function flow_list = CreateTrafficMatrix( this, seed, traffic_density )
            graph_size = this.Size;
            flow_list = zeros(graph_size*graph_size,4);
            flow_list(:,4) = (1:graph_size*graph_size)';
            
            rng(seed);
            K = 0;
            for i = 1:graph_size
                if nargin >=3
                    %% Randomly choose flow target
                    % We first select one more target nodes, in case that a seleced target
                    % node is the source node. If source node is not selected, then we
                    % remove one node from the target node set.
                    %
                    % To gurantee the normal running of |unique_randi|, the |nd| should
                    % not exceed |graph_size|.
                    if traffic_density < 1
                        nd = min(ceil(graph_size*traffic_density)+1, graph_size);
                    else
                        nd = min(traffic_density+1, graph_size);
                    end
                    dest = unique_randi(graph_size,nd);
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
                flow_list(K+1:K+ld,3)= ...
                    round(unifrnd(ConvergeNetwork.DEMAND_MIN,ConvergeNetwork.DEMAND_MAX,ld,1)...
                    .*this.graph.NodeWeight(dest));
                K = K+ld;
            end
            flow_list = flow_list(1:K,:);
        end
    end
    methods
        function this = ConvergeNetwork(traffic_seed, traffic_density, ...
                filename, num_candidate_path)
            if nargin == 0
                this.graph = FlatGraph;
                return;
            end
            
            if nargin>=3
                this.graph = ISPGraph(filename);
            else
                error('Error: absence of input arguments.');
            end
            % Re-Configuration of link capacity
            if nargin >= 2 && ~isempty(traffic_seed) && ~isempty(traffic_density)
                this.CreateTrafficProfiles(traffic_seed, traffic_density);
                this.reconfigure_capacity;
            end
            if nargin >= 5 && ~isempty(num_candidate_path)
                this.BuildCandidate(num_candidate_path);
            end
        end
    end
end