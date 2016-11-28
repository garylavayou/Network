classdef DualLayerNetwork < Network
    %DUALLAYERNETWORK incapsulate the DualLayerGraph class.
    %   Detailed explanation goes here
    properties (Constant)
        UP_DEMAND_MAX = 10;            % tunable parameter
        UP_DEMAND_MIN = 1;
        UP_DOWN_RATIO = 1;
        % INTRA_DEMAND_MAX = 10;
        % INTRA_DEMAND_MIN = 1;
        INTRA_DEMAND_MAX0 = 50;
        INTRA_DEMAND_MIN0 = 5;
        DOWN_DEMAND_MAX = DualLayerNetwork.UP_DOWN_RATIO*DualLayerNetwork.UP_DEMAND_MAX;
        DOWN_DEMAND_MIN = DualLayerNetwork.UP_DOWN_RATIO*DualLayerNetwork.UP_DEMAND_MIN;
    end
    properties
        index;
        node;
        link;
        
        BottomSize;
        UpperSize;
        IndirectedLinkNumber;
    end
    
    methods  % Property Mehtods
        function n = get.UpperSize(this)
            n = length(this.index.Router);
        end
        function n = get.BottomSize(this)
            n = length(this.index.GBS) + length(this.index.NBS);
        end
        function n = get.IndirectedLinkNumber(this)
            n = length(this.link);
        end
    end
    
    methods
        %% Constructor
        %   DualLayerNetwork(layer_size, graph_seed, traffic_seed, link_capacity, inter_flow_density, intra_flow_density)
        %   DualLayerNetwork(model)
        %
        % |layer_size|: the graph size of each layer. If |layer_size| is scalar, the two
        %     layer have the same size.
        %
        % |seed|: Seed for initilize the random number generator when build the network
        %     topology.
        %
        % |traffic_seed|: Seed for initilize the random number generator when create
        %     flow.
        %
        % |link_capacity|: the link capacity of each layer. If this argument is not
        %     specified, the link capcity is set to 1. If |link_capcity| is a scalar,
        %     the link capacity of the two layer is the same.
        %
        % |inter_flow_density|: the flow density between RAN cluster node and core
        %     cluster node.
        %
        % |intra_flow_density|: the flow density between the same cluster.
        function this = DualLayerNetwork(arg1, graph_seed, traffic_seed, ...
                link_capacity, inter_flow_density, intra_flow_density)
            if ischar(arg1)
                model = arg1;
                load_sample_graph(this, model);
            else
                layer_size = arg1;
                if nargin < 5
                    error('Error:not enough input arguments.');
                end
                if nargin>=8 && isempty(link_capacity)==0
                    this.graph = DualLayerNetwork.Build(layer_size(1), ...
                        cluster_size(2), graph_seed, link_capacity);
                else %not specify the link capacity
                    this.graph = DualLayerNetwork.Build(cluster_number, ...
                        cluster_size, graph_seed);
                end
            end
            
            % do not produce traffic profiles.
            if nargin <= 2
                return;
            end
            num_traffic_profile = length(traffic_seed);
            this.TrafficProfiles = cell(num_traffic_profile,1);
            for j = 1:num_traffic_profile
                if isempty(intra_flow_density)
                    this.TrafficProfiles{j} = this.CreateTrafficMatrix(...
                        traffic_seed(j), inter_flow_density);
                else
                    this.flow_table_list{j} = this.CreateTrafficMatrix(...
                        traffic_seed(j), inter_flow_density, true, intra_flow_density);
                end
            end
            if ~(nargin>=4 && isempty(link_capacity)==0)
                this.reconfigure_capacity;
            end
        end
        
        plot(this);

        flow_list = CreateTrafficMatrix(this, seed, inter_density, b_intra, intra_density);

        [theta, b_d, b_u] = MBODA_TE(this, flow_table_index, w);
        [theta, a] = MFODA_TE( this, flow_table_index, w );
    end
    
    methods (Access = private)
        load_sample_network(this, model);
        flow_table = create_flow( this, seed, inter_density, b_intra, intra_density);
    end
    
end