%% Class Network
% Network is an abstract class, containing the basic network properties
%% Class Definition
%   classdef Network  < handle
classdef (Abstract) Network  < handle  
   	properties (Constant)
% 		NUM_CANDIDATE_PATH = 3;
	end
	properties
		graph;
		path_set;
		fobj;
        
        TrafficProfiles;
        Size;               % TODO: rename as NumberNodes
        LinkNumber;         % TODO: rename as NumberLinks
    end
    properties(Abstract=true)
    end
	
    methods
        function set.TrafficProfiles(this, traffic_profiles)
            if iscell(traffic_profiles)
                this.TrafficProfiles = traffic_profiles;
            else
                this.TrafficProfiles = {traffic_profiles};
            end
        end
        function n = get.Size(this)
            n = this.graph.Size;
        end
        function n = get.LinkNumber(this)
            n = this.graph.EdgeNumber;
        end
        function Id = NodeId(this)
            Id = 1:this.Size;
        end
        function Copy(this, net)
            this.path_set = net.path_set;
            this.fobj = net.fobj;
            this.TrafficProfiles = net.TrafficProfiles;
            this.graph.Copy(net.graph);
        end

        %% Network Optimization Method
        [theta, flow_variables] = MinMaxLinkUtilization(this, traffic_matrix_id);
        [theta, flow_variables] = MinMaxLinkUtilizationMinResource(this, traffic_matrix_id);
        [lambda_fptas, flow_variables, total_phases_counter, total_steps_counter, rt] = ...
                MaxConcurrentFlow(this, w, traffic_matrix_id, arg4);
        [lambda_fptas, path_set] = MaxConcurrentFlowPath(this, w, traffic_matrix_id);
    end
    
    methods (Access = protected)
        %%
        % |b_equal_ratio|: if |b_equal_ratio=1|,  every link is upscaled with the same
        % factor.
		function reconfigure_capacity(this, b_equal_ratio)
            if nargin == 1
                b_equal_ratio = 0;
            end
			num_traffic_profile = length(this.TrafficProfiles);
            progress = 0;
            progress_all = num_traffic_profile;
            message = '        Progress of Reconfiguration Capacity';
            hwait=waitbar(progress/progress_all,message);
            if b_equal_ratio
                capacity_factor = 0;
            else
                edge_flow = zeros(1,length(this.graph.Capacity)); % total flow
            end
            for j = 1:num_traffic_profile
                %% Progress
                progress = progress + 1;
                waitbar(progress/progress_all,hwait);
                % [flow_variables] sattisfy that  AF=d, AF<=¦ÈC
                [theta, flow_variables] = this.MinMaxLinkUtilizationMinResource(j);
                if b_equal_ratio
                    capacity_factor = max(capacity_factor, theta/Graph.THETA);
                else
                    edge_flow = max(edge_flow,sum(flow_variables,1));
                end
            end
            close(hwait);
            if b_equal_ratio
                edge_flow = this.graph.Capacity*capacity_factor;
            else
                edge_flow = edge_flow/Graph.THETA;
            end
            for le = 1:length(edge_flow)
                ei = this.graph.Head(le);
                ej = this.graph.Tail(le);
                if edge_flow(le)~=0
                    this.graph.Adjacent(ei,ej) = edge_flow(le);
                else
                    warning('no flow pass through edge(%d,%d)', ei,ej);
                    % Turn-off the unused links
                    this.graph.Adjacent(ei,ej) = 0;
                end
            end
            this.graph.Adjacent = ceil(this.graph.Adjacent);
            this.graph.Renew;
		end

    end
    
    methods (Abstract=true)
        %% Method Run
        % The concrete subclass must implement this method to perform their own tasks.
        %
        %   function Run(this, flow_table, NUM_PATH)
        %   if nargin == 3
        %       Network.candidate_path_multi_flow(this, flow_table, NUM_PATH);
        %   else
        %       Network.candidate_path_multi_flow(this, flow_table);
        %   end
        %   end
        %% Method CreateTrafficMatrix
        % The concrete subclass must implement this method to generate traffic flows for
        % the network instance. 
        CreateTrafficMatrix(this);
    end
end