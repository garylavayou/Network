%% Create Traffic Matrix 
% This function overrides the same function of FlatNetwork
% The source demands is directed to the closest server.

%% Function Declaration
%   flow_list = CreateTrafficMatrix( this, source_demand_index)
%
% |source_demand_index|: indicate which source_demand matrix is used for creating traffic
% matrix. 
%
function flow_list = CreateTrafficMatrix( this, source_demand_index )
n_demand = size(this.source_demands{source_demand_index},1);
flow_list = zeros(n_demand,4);
flow_list(:,4) = (1:n_demand)';

for i = 1:n_demand
    %% Find the closest server to the node
    src_node = this.source_demands{source_demand_index}(i,1);
    [dest_node, ~, ~] = ...
        this.graph.DistanceOrderedNodes(src_node, this.server_id, 0);
    
    flow_list(i,1) = src_node;
    flow_list(i,2) = dest_node(1);
    flow_list(i,3) = this.source_demands{source_demand_index}(i,2);
end
end

%%%
% *Useage*: If one needs to select the nodes that do not pass a intermediate selected
% node, use the follwing code.
%
%   b = true(size(dest_node));
%   for j = 2:length(dest_node)
%       for k = 1:j-1
%           if ~isempty(find(path_list{j}, dest_node(k),1))
%               b(j) = false;
%               break;
%           end
%       end
%   end
%   dest_node = dest_node(b);