classdef HierarchicalClusterNetwork < Network
    %HierarchicalClusterNetwork: The network is divided in clusters.
    % Cluster 1 is the high level cluster(core) and the other clusters are the low level
    % cluster (RAN cluster).
    properties
        CoreSize;
        BackhaulSize;
        ClusterNumber;
        ClusterSize;			% a vector:
        Size;					% network size
    end
    
    methods
        function this = HierarchicalClusterNetwork(cluster_number, cluster_size, graph_seed,...
                num_traffic_profile, traffic_seed, inter_flow_density, intra_flow_density,...
                link_capacity, b_candidate_path)
            % cluster_number: Number of cluster.
            % cluster_size: Size of each cluster. If cluster_number is a scalar, all
            %     cluster have the same size. Otherwise cluster_number should be a vector
            %     and its length is equal to cluster_number.
            % graph_seed: Seed for initilize the random number generator when build the network
            %     topology.
            % num_traffic_profile: Number of traffic profiles that used to determine the
            %     capacity of network.
            % traffic_seed: Seed for initilize the random number generator when create
            %     flow.
            % inter_flow_density: the flow density between RAN cluster node and core
            %     cluster node.
            % intra_flow_density: the flow density between the same cluster.
            % link_capacity: see the help about HierarchicalClusterGraph.Build.
            % b_candidate_path: specifies building candidate paths if this parameter is
            %     supplied and the value is true. Otherwise do not build coandidate path.
            progress = 0;
            progress_all = num_traffic_profile;
            message = 'Build HierarchicalClusterNetwork in Progress...';
            hwait=waitbar(progress/progress_all,message);
            
            if numel(cluster_size) == 1
                cluster_size = ones(cluster_number,1)*cluster_size;
            end
            
            theta = 0.5;
            
            if nargin>=8 && isempty(link_capacity)==0
                this.graph = HierarchicalClusterGraph.Build(cluster_number, ...
                    cluster_size, graph_seed, link_capacity);
            else %not specify the link capacity
                this.graph = HierarchicalClusterGraph.Build(cluster_number, ...
                    cluster_size, graph_seed);
            end
            this.flow_table_list = cell(num_traffic_profile,1);
            for j = 1:num_traffic_profile
                if isempty(intra_flow_density)
                    this.flow_table_list{j} = this.hierarchical_flow_generator(...
                        traffic_seed(j), inter_flow_density);
                else
                    this.flow_table_list{j} = this.hierarchical_flow_generator(...
                        traffic_seed(j), inter_flow_density, true, intra_flow_density);
                end
            end
            if ~(nargin>=8 && isempty(link_capacity)==0)
                edge_flow = zeros(length(this.graph.Capacity),1); % total flow
                for j = 1:num_traffic_profile
                    [~, flow_variables] = multi_flow_lp_sparse(this.graph, this.flow_table_list{j});
                    edge_flow = max(edge_flow,(sum(flow_variables,1)/theta)');
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
            end
            if nargin>=9 && isempty(b_candidate_path)==0 && b_candidate_path==true
                this.path_set = PathSet.BuildCandidate(this.graph, 3);
            end
        end
        
        function n = get.CoreSize(this)
            n = this.graph.cluster_size(1);
        end
        function n =get.ClusterNumber(this)
            n = this.graph.cluster_number;
        end
        function n =get.ClusterSize(this)
            n = this.graph.cluster_size;
        end
        function n=get.Size(this)
            n = sum(this.graph.cluster_size);
        end
        function n = get.BackhaulSize(this)
            n = sum(this.graph.cluster_size(2:end));
        end
        function Run(this,j)
            Network.candidate_path_multi_flow(this,j);
        end
        theta = MBODA_TE( this, flow_table_index, w );
        theta = MFODA_TE( this, flow_table_index, w );
    end
    methods(Access=private)
        flow_list = CreateTrafficMatrix(this, seed, inter_flow_density, b_intra_flow, intra_flow_density);
    end
end