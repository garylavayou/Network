classdef RadioAccessNetwork < handle
    %RADIOACCESSNETWORK Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Constant)
        ETA = 3;
        
        PLOT_WIRE_NETWORK = 1;
        PLOT_DISTRIBUTION = 2;
        PLOT_USER_BS_ASSOCIATION = 3;
        PLOT_BS_USER_ASSOCIATION = 4;
        PLOT_BS_USER_ASSOCIATION_SPECIFIC = 5;
        PLOT_BS_USER_ASSOCIATION_GRAPH = 6;
    end
    
    properties
        wire_node;              % wire nodes include BS and router.
        index;
        wire_link;
        users;
        user_bs;
        bs_user;
        % b_feasible;
        feasible_user_index;    % user index is 1-based, user id = user index + UserIdOffset
        active_user_index;      % active_user_index included in feasible_user_index
        fake_user_id;
        active_bs_index;        % bs id is equal to bs index
        fake_bs_user;
%         fake_user_bs;
        flow_table;
        ran_graph;
        path_set;
        fobj;
        coverage = 200;         % meter
        
        NumberWireNode;
        NumberWireLink;
        UserIdOffset;
        NumberBS;
        NumberActiveBS;
        NumberRouter;
        NumberUser;
        NumberFeasibleUser;
        NumberActiveUser;
        BSBandwidth;                    %  MHz
        NumberFlow;
    end
    
    methods
        function n = get.NumberWireNode(this)
            n = size(this.wire_node,1);
        end
        function n = get.NumberBS(this)
            n = length(this.index.GBS) + length(this.index.NBS);
        end
        function n = get.NumberActiveBS(this)
            n = length(this.active_bs_index);
        end
        function n = get.NumberFeasibleUser(this)
            n = length(this.feasible_user_index);
        end
        function n =get.NumberActiveUser(this)
            n = length(this.active_user_index);
        end
        function n = get.UserIdOffset(this)
            n = size(this.wire_node,1);
        end
        function n = get.NumberWireLink(this)
            n = size(this.wire_link,1);
        end
        function n = get.NumberUser(this)
            n = size(this.users,1);
        end
        function n = get.NumberRouter(this)
            n = length(this.index.Router);
        end
        function n = get.NumberFlow(this)
            n = size(this.flow_table,1);
        end
        function this = RadioAccessNetwork(model, request_prob, coverage, range_user, ...
                range_demand, sir_th, bs_bandwidth)
            %% RadioAccessNetwork RadioAccessNetwork(model, request_prob, range_user, range_demand, sir_th, bs_bandwidth)
            % model = SD-RAN, SingleBS, DoubleBS, DoubleBSDoubleRouter, grid9
            % default user number 1~6
            % default user demand 1~5Mbps
            % default SIR threshold 0.4
            % default BS bandwidth 10
            % Network nodes location
            bs_location(this,model);
            % user location
            if nargin >= 3
                this.coverage = coverage;
            end
            if nargin >= 4
                user_location(this, range_user(1), range_user(2));
            else
                user_location(this);
            end
            % user and BS association
            if nargin >= 6 
                user_association(this, sir_th);
            else
                user_association(this);
            end
            %% User demand
            if nargin >= 5
                CreateFlow(this, 'random', request_prob, range_demand(1), range_demand(2));
            elseif nargin >= 2
                CreateFlow(this, 'random', request_prob);
            else
                CreateFlow(this, 'random');
            end
            update_user_association(this);
            this.ran_graph = RANGraph.Build(this, 1);
            %% Construct Paths
            this.path_set = PathSet.Build(this);
            if nargin >= 7
                this.BSBandwidth = bs_bandwidth;
            else
                this.BSBandwidth = 10;
            end
        end
    end
    methods (Access=private)          % prototype declaration
        lp_multi_flow_path(this, objective, max_rate);
        admm_multi_flow(this, objective);
    end
end

