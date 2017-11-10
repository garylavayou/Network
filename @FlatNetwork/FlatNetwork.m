%% FlatNetwork
%   FlatNetwork < Network
%
% FlatNetwork is used to represent the network with flat topology and flat traffic distributions
%%
classdef FlatNetwork < Network
    %% Inherited Methods
    methods
        flow_list = CreateTrafficMatrix( this, seed, traffic_density )
    end
    
    properties (Constant)
        DEMAND_MAX = 10;            % tunable parameter
        DEMAND_MIN = 1;
    end
    methods
        function this = FlatNetwork(model, traffic_seed, traffic_density, ...
                size, seed, num_candidate_path)
            if nargin == 0
                this.graph = RandomGraph;
                return;
            end
            if strcmpi(model,'Random')
                if nargin == 4
                    this.graph = RandomGraph(size);
                elseif nargin >=5
                    this.graph = RandomGraph(size,seed);
                else
                    error('Error: absence of input arguments.');
                end
            else
                % Samples include 'NSFNET', 'smallnet', 'ARPA2', 'BellCore'
                % 'sample1', 'sample2'
                this.graph = SampleGraph(model);
            end
            % Generate Traffic Profiles and Re-Configuration of link capacity
            if nargin >= 3 && ~isempty(traffic_seed) && ~isempty(traffic_density)
                % Create traffic profiles
                this.CreateTrafficProfiles(traffic_seed, traffic_density);
                % Re-Configuration of link capacity
                if strcmpi(model,'Random')
                    this.reconfigure_capacity;
                else
                    this.reconfigure_capacity(1);
                end
            end
            if nargin >= 6 && ~isempty(num_candidate_path)
                this.BuildCandidate(num_candidate_path);
            end
        end
        
        function Copy(this, net)
            Copy@Network(this, net);
        end
        %% Network Optimization Method
        [lambda, flow_rate, flow_variables] = ...
            MaxConcurrentFlowCandidatePath(this, fi, NUM_PATH);
        function BuildCandidate(this, K, flow_list)
            %% Path calculation can be performed offline or event-driven.
            % Method-1: K-disjoint Path
            
            % Method-2: K-bandwidth-disjoint Path
            this.path_set = PathSet(this.graph.Size, this.graph.Size);
            if nargin > 2
                % for each flow pair, find at most K path.
                for i=1:size(flow_list,1)
                    src = flow_list(i,1);
                    dest = flow_list(i,2);
                    [path_list, k] = this.graph.CandidatePaths(K, src, dest);
                    for j=1:k
                        this.path_set.AddPath(path_list{j});
                    end
                end
            else
                for src = 1:this.graph.Size
                    for dest = 1:this.graph.Size
                        if dest==src
                            continue;
                        end
                        [path_list, k] = this.graph.CandidatePaths(K, src, dest);
                        for j=1:k
                            this.path_set.AddPath(path_list{j});
                        end
                    end
                end
            end
            this.path_set.AllocateId();
        end
    end
    
    methods(Access=protected)
        
        function CreateTrafficProfiles(this, traffic_seed,traffic_density)
            num_traffic_profile = length(traffic_seed);
            this.TrafficProfiles = cell(num_traffic_profile,1);
            for j = 1:num_traffic_profile
                this.TrafficProfiles{j} = ...
                    this.CreateTrafficMatrix(traffic_seed(j), traffic_density);
            end
        end
    end
    
    methods (Static)
        function network = Build(graph,traffic_profiles,num_path)
            %% TODO:
            % Input parameters must be checked to ensure the consistence.
            network = FlatNetwork;
            network.graph.Copy(graph);
            if nargin >= 2 && ~isempty(traffic_profiles)
                network.TrafficProfiles = traffic_profiles;
            end
            if nargin >=3 && ~isempty(num_path)
                network.path_set = PathSet.BuildCandidate(graph, num_path);
            end
        end
    end
end