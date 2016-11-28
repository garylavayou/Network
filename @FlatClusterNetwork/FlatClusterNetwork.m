classdef FlatClusterNetwork < RandomNetwork
    properties
        cluster_number;
        cluster_size;		% a vector:
    end
    
    methods
        function this = FlatClusterNetwork(cluster_number, cluster_size, seed, ...
                num_traffic_profile, traffic_seed, inter_flow_density, intra_flow_density)
            progress = 0;
            progress_all = num_traffic_profile;
            message = '                 Progress of Calculation';
            hwait=waitbar(progress/progress_all,message);
            
            this.cluster_number = cluster_number;
            if numel(cluster_size) == 1
                cluster_size = ones(cluster_number,1)*cluster_size;
            end
            this.cluster_size = cluster_size;
            
            theta = 0.5;
            this.graph = FlatClusterGraph.Build(this.cluster_number, this.cluster_size, seed);
            edge_flow = zeros(length(this.graph.Capacity),1); % total flow
            this.flow_table_list = cell(num_traffic_profile,1);
            for j = 1:num_traffic_profile
                % this.flow_table_list{j} = random_flow_generator(traffic_seed(j), this.graph, inter_flow_density(1));
                [~, flow_variables] = multi_flow_lp_sparse(this.graph, this.flow_table_list{j});
                edge_flow = max(edge_flow,sum(flow_variables,2)/theta);
                %% Progress
                progress = progress + 1;
                waitbar(progress/progress_all,hwait);
            end
            close(hwait);
            for le = 1:length(edge_flow)
                ei = this.graph.Head(le);
                ej = this.graph.Tail(le);
                this.graph.Adjacent(ei,ej) = edge_flow(le);
            end
            this.graph.Adjacent = ceil(this.graph.Adjacent);
            this.graph.Capacity = this.graph.Adjacent(this.graph.Adjacent~=0);
            this.path_set = PathSet.BuildCandidate(this.graph, 3);
        end
        % function Run(this, fi)   derived from RandomNetwork
    end
    methods(Access=private)
        hierarchical_flow_generator(this, seed, inter_flow_density, b_intra_flow, intra_flow_density);
    end
end