classdef FatTreeNetwork < FlatNetwork
%% Inhireted properties
%   properties (Constant)
%       DEMAND_MAX = 10;            % tunable parameter
%       DEMAND_MIN = 1;
%   end
    methods
        function flow_list = CreateTrafficMatrix( this, seed, traffic_density )
            N = length(this.graph.edge_id);
			flow_list = zeros(N*N, 4);
			flow_list(:,4) = (1:N*N)';
            
            rng(seed);
			K = 0;
			for s_index = 1:N
				if nargin >=3
                    %% Randomly choose flow target
                    % We first select one more target nodes, in case that a seleced target
                    % node is the source node. If source node is not selected, then we
                    % remove one node from the target node set.
                    %
                    % To gurantee the normal running of |unique_randi|, the |nd| should
                    % not exceed |graph_size|.
					if traffic_density < 1
						nd = min(ceil(N*traffic_density)+1, N);
					else
						nd = min(N, traffic_density+1);
					end
					dest = unique_randi(N,nd);
                    dest(dest==s_index)=[];    % exclude the same src and dest.
                    ld =length(dest);
                    if ld == nd
                        dest(randi(ld,1)) = [];
                        ld = ld - 1;
                    end
                else
				    dest = unique(randi(N,N,1));
                    ld =length(dest);
				end

				flow_list(K+1:K+ld,1) = this.graph.edge_id(s_index);
				flow_list(K+1:K+ld,2) = this.graph.edge_id(dest);
				flow_list(K+1:K+ld,3) = ...
                    round(unifrnd(FatTreeNetwork.DEMAND_MIN, FatTreeNetwork.DEMAND_MAX,ld,1));
				K = K+ld;
			end
			flow_list = flow_list(1:K,:);   		
		end
    end
	methods
		function this = FatTreeNetwork(traffic_seed, traffic_density, ...
                K, C, num_candidate_path)
            if nargin < 3
                K = 4;
                C = 100;
            elseif nargin == 3
                C = 100;
            end
            this.graph = FatTreeGraph(K,C);
            if nargin >= 2 && ~isempty(traffic_seed) && ~isempty(traffic_density)
                % Create traffic profiles
                this.CreateTrafficProfiles(traffic_seed, traffic_density);
                % Re-Configuration of link capacity
               this.reconfigure_capacity(1);
            end
            if nargin >= 5 && ~isempty(num_candidate_path)
                this.BuildCandidate(num_candidate_path);
            end
        end
	end
end