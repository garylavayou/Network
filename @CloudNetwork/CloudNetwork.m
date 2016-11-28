classdef CloudNetwork < FlatNetwork
    
    properties
        PhysicGraph;
        server_id;
        source_demands;
        
        NumberServer;
    end
    
    methods
        function n = get.NumberServer(this)
            n = length(this.server_id);
        end
        
        %% Server Capacity
        % server capacity is the processing capacity to process all in/out flows.
        function v = ServerCapacity(this, i)
            v = sum(this.graph.Adjacent(i,:)) + sum(this.graph.Adjacent(:,i));
        end
    end
    methods
        function this = CloudNetwork(model, servers, traffic_seed, size, seed)
            % Call the super class to generate the network topology
            this@FlatNetwork(model, [], [], size, seed);
            this.server_id = servers;
            
            % Generate Traffic Profiles and Re-Configuration of link capacity
            this.CreateTrafficProfiles(traffic_seed);
            % Re-Configuration of link capacity
            if strcmpi(model,'Random') || strcmpi(model,'Sample2')
                this.reconfigure_capacity;
            else
                this.reconfigure_capacity(1);
            end
            
%             this.BuildCandidate(num_candidate_path);
        end
        
        function Copy(this, net)
            Copy@Network(this, net);
            this.server_id = net.server_id;
        end
        
        function BuildCandidate(this, K, sd_index)
            %% Path calculation can be performed offline or event-driven.
            % _See FlatNetwork.BuildCandidate.
            this.path_set = PathSet(this.graph.Size, this.graph.Size);
            demands = this.source_demands{sd_index};
            % for each source demand, find at most K path.
            for i = 1:size(demands,1)
                src = demands(i,1);
                dest_set = this.server_id;
                [path_list, k] = this.graph.CandidatePaths(K, src, dest_set);
                for j=1:k
                    this.path_set.AddPath(path_list{j});
                end
            end
            
            this.path_set.AllocateId();
        end
        
    end
    
    methods(Access=protected)
        %% Override the super class method
        % The traffic profiles created by this method only record the source of the
        % traffic and its demand.
        % 
        %   CreateTrafficProfiles(this, traffic_seed,traffic_density)
        %
        % |traffic_seed|:
        %
        function CreateTrafficProfiles(this, traffic_seed)
            num_traffic_profile = length(traffic_seed);
            this.source_demands = cell(num_traffic_profile,1);
            
            for j = 1:num_traffic_profile
                rng(traffic_seed(j));
                
                candidate_sources = this.NodeId;
                candidate_sources(this.server_id) = [];
                ns = length(candidate_sources);
                
                demands = zeros(ns, 2);
                demands(:,1)= candidate_sources;
                demands(:,2) = ...
                    round(unifrnd(CloudNetwork.DEMAND_MIN,CloudNetwork.DEMAND_MAX,ns,1));
                this.source_demands{j} = demands;
                this.TrafficProfiles{j} = this.CreateTrafficMatrix(j);
            end
        end
        
    end
    
    methods        
        % Override inherited method        
        flow_list = CreateTrafficMatrix( this, source_demand_index)
    end
end

