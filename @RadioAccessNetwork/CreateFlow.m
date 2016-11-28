%% Flow Specification
% Generate user flows.
% Flows are represented as (source, dest, demand, index).
%% Function Definition
% Create(mode, request_prob, min_bw, max_bw)
% min_bw: Minimum bandwidth demand of users.
% max_bw: Maximum bandwidth demand of users.
function CreateFlow(this, mode, request_prob, min_bw, max_bw)
switch nargin
    case 2
        REQUEST_PROBABILITY = 0.9;
        MIN_REQUEST_BANDWIDTH = 1;
        MAX_REQUEST_BANDWIDTH = 5;
    case 3
        REQUEST_PROBABILITY = request_prob;
        MIN_REQUEST_BANDWIDTH = 1;
        MAX_REQUEST_BANDWIDTH = 5;
    case 4
        REQUEST_PROBABILITY = request_prob;
        MIN_REQUEST_BANDWIDTH = min_bw;
        MAX_REQUEST_BANDWIDTH = 5;
    case 5
        REQUEST_PROBABILITY = request_prob;
        MIN_REQUEST_BANDWIDTH = min_bw;
        MAX_REQUEST_BANDWIDTH = max_bw;
    otherwise
        error('error: wrong number of input arguments.');
end

if strcmpi(mode, 'random')
    rng(20151204);
    this.flow_table = zeros(this.NumberFeasibleUser, 4);
    k = 0;
    for i = 1:this.NumberFeasibleUser
        if rand() < REQUEST_PROBABILITY
          k = k+1;
          src = this.index.Router(randi([1 this.NumberRouter]));
          dest = this.feasible_user_index(i);
          demand = randi([MIN_REQUEST_BANDWIDTH MAX_REQUEST_BANDWIDTH]);
          this.flow_table(k,:) = [src dest demand k];
        end
    end
    this.flow_table((k+1):end,:) = [];
end

% users has no demand can be temporarily eliminated from the network
this.active_user_index = unique(this.flow_table(:,2));
this.fake_user_id = this.UserIdOffset+(1:this.NumberActiveUser);
this.flow_table(:,2) = this.fake_user_id;
% TODO if one user has multiple flow, then the user_id mapping should be done one by one,
% since the length of flow table and fake_user_id are not the same.
% for i= 1:size(this.flow_table,1)
%     fake_user_index = find(this.flow_table(i,2));
%     this.flow_table(i,2) = this.fake_user_id(fake_user_index);
%     % the reverse is:
%     % this.flow_table(i,2) = this.active_user_index(this.flow_table(i,2)-this.UserIdOffset);
% end
end