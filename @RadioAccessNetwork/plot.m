function plot(this, plot_type)
switch plot_type
    case RadioAccessNetwork.PLOT_WIRE_NETWORK
        figure('Name', 'Wired Backhaul of Radio Access Network');
        plot(this.wire_node(this.index.NBS,1), this.wire_node(this.index.NBS,2),'^', ...
            this.wire_node(this.index.GBS,1), this.wire_node(this.index.GBS,2),'s',...
            this.wire_node(this.index.Router,1), this.wire_node(this.index.Router,2),'d');
        if isempty(this.index.NBS)
            legend({'Gateway BS', 'Router'}, 'Location', 'northwest');
        else
            legend({'Normal BS', 'Gateway BS', 'Router'}, 'Location', 'northwest');
        end
        hold on;
        for i = 1:this.NumberWireLink
            src = this.wire_link(i,1);
            dest = this.wire_link(i,2);
            switch this.wire_link(i,3)
                case 0
                    plot(this.wire_node([src dest], 1), this.wire_node([src dest], 2), ...
                        'r-', 'LineWidth',1.5);
                case 1
                    plot(this.wire_node([src dest], 1), this.wire_node([src dest], 2), ...
                        'b-', 'LineWidth',1.5);
                case 2
                    plot(this.wire_node([src dest], 1), this.wire_node([src dest], 2), 'k-');
                case 3
                    plot(this.wire_node([src dest], 1), this.wire_node([src dest], 2), 'g-');
                otherwise
                    plot(this.wire_node([src dest], 1), this.wire_node([src dest], 2), 'c.-');
            end
        end
        title('Wired Backhaul of Radio Access Network');
    case RadioAccessNetwork.PLOT_DISTRIBUTION
        bs_index = [this.index.NBS; this.index.GBS];
        
        figure('Name', 'BS and User Distribution');
        plot(this.users(:,1), this.users(:,2), '.');
        hold on;
        plot(this.wire_node(bs_index,1), this.wire_node(bs_index,2), '^');
        legend('Base Station','User');
        for i=1:this.NumberUser
            %    x -user      x - BS               y - user  y - BS
            plot([this.users(i,1) this.wire_node(this.users(i,4),1)], ...
                [this.users(i,2) this.wire_node(this.users(i,4),2)],'k:');
        end
        title('BS and User Distribution');
    case RadioAccessNetwork.PLOT_USER_BS_ASSOCIATION
        bs_index = [this.index.NBS; this.index.GBS];
        
        figure('Name', 'User to BS Association');
        plot(this.users(:,1),this.users(:,2), '.');
        hold on;
        plot(this.wire_node(bs_index,1), this.wire_node(bs_index,2), '^');
        legend('Base Station','User');
        for i = 1:this.NumberFeasibleUser
            user_index = this.feasible_user_index(i);
            for j = 1:size(this.user_bs{user_index},1)
                bs_id = this.user_bs{user_index}(j,1);
                plot([this.users(user_index,1) this.wire_node(bs_id,1)], ...
                    [this.users(user_index,2) this.wire_node(bs_id,2)],'k-');
            end
        end
        title('User to BS Association');
    case RadioAccessNetwork.PLOT_BS_USER_ASSOCIATION
        bs_index = [this.index.NBS; this.index.GBS];
        
        figure('Name', 'BS to User Association');
        plot(this.users(:,1),this.users(:,2), '.');
        hold on;
        plot(this.wire_node(bs_index,1), this.wire_node(bs_index,2), '^');
        legend('Base Station','User');
        for i = 1:this.NumberActiveBS
            bs_id = this.active_bs_index(i);
            for j = 1:size(this.bs_user{bs_id},1)
                user_id = this.feasible_user_index(this.bs_user{bs_id}(j,1));
                plot([this.wire_node(bs_id,1) this.users(user_id,1)], ...
                    [this.wire_node(bs_id,2) this.users(user_id,2)],'k-');
            end
        end
        title('BS to User Association');
    case {RadioAccessNetwork.PLOT_BS_USER_ASSOCIATION_SPECIFIC, RadioAccessNetwork.PLOT_BS_USER_ASSOCIATION_GRAPH}
        figure('Name', 'Specified BS and User Association.');
        hold on;
        plot(this.users(:,1), this.users(:,2), 'r.', ...
            this.users(this.feasible_user_index,1), ...
            this.users(this.feasible_user_index,2), 'g.', ...
            this.users(this.active_user_index,1), ...
            this.users(this.active_user_index,2), 'b.', ...
            this.wire_node(this.index.NBS,1), ...
            this.wire_node(this.index.NBS,2),'^', ...
            this.wire_node(this.index.GBS,1), ...
            this.wire_node(this.index.GBS,2),'s');
        if isempty(this.index.NBS)
            legend({'Infeasible User', 'Inactive User', 'Active User', ...
                'Gateway BS'}, 'Location', 'northwest');
        else
            legend({'Infeasible User', 'Inactive User', 'Active User', 'Normal BS', ...
                'Gateway BS'}, 'Location', 'northwest');
        end
        if plot_type == RadioAccessNetwork.PLOT_BS_USER_ASSOCIATION_SPECIFIC
            for i = 1:this.NumberBS
                for j = 1:size(this.bs_user{i},1)
                    user_id = this.feasible_user_index(this.bs_user{i}(j,1));
                    plot([this.wire_node(i,1) this.users(user_id,1)], ...
                        [this.wire_node(i,2) this.users(user_id,2)],'g--');
                end
            end
            
            for i = 1:this.NumberBS
                for j = 1:size(this.fake_bs_user{i},1)
                    user_id = this.active_user_index(this.fake_bs_user{i}(j,1));
                    plot([this.wire_node(i,1) this.users(user_id,1)], ...
                        [this.wire_node(i,2) this.users(user_id,2)],'k-');
                end
            end
        else
            graph = this.ran_graph;
            for i = 1 : graph.NumberBS
                for j = 1 : i
                    if graph.Adjacent(i,j) ~= 0
                        plot([this.wire_node(i,1) this.wire_node(j,1)], ...
                            [this.wire_node(i,2) this.wire_node(j,2)], 'b-', 'LineWidth',2);
                    end
                end
            end
            for i = 1 : graph.NumberBS
                for j = 1 : graph.NumberUser
                    if graph.Adjacent(i, j + this.UserIdOffset) ~= 0
                        user_id = this.active_user_index(j);
                        plot([this.wire_node(i,1) this.users(user_id,1)], ...
                            [this.wire_node(i,2) this.users(user_id, 2)],...
                            'k-');
                    end
                end
            end
        end
end
end
