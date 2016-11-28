%% Interference and user-BS association
% given the user and base station location, determine the user-BS association.
% the determination criterion is SIR/SNR.
function user_association(this, sir_th)
% user_bs: store the index of associated BS
% bs_user: store the index of associated feasible users
% feasible_user_index: users(feasible_user_id,3) = feasible user id;
switch nargin
    case 1
        SIR_TH = 0.4;
    case 2
        SIR_TH = sir_th;
    otherwise
        error('error: worng number of input arguments.');
end

this.user_bs = cell(this.NumberUser, 1);
b_feasible = zeros(this.NumberUser, 1);
for i = 1:this.NumberUser
    d = this.wire_node(1:this.NumberBS,1:2);
    d(:,1) = d(:,1) - this.users(i,1);
    d(:,2) = d(:,2) - this.users(i,2);
    dist = sqrt(sum(d.^2,2));
    E_sum = sum(dist.^(-RadioAccessNetwork.ETA));
    if length(dist) > 1
        %% Evaluation of SINR at user side.
        % each BS has the same transmit power, and the transimit power is uniformly
        % allocate within the working frequency band.
        SIR = dist.^(-RadioAccessNetwork.ETA) ./ ...
            (E_sum - dist.^(-RadioAccessNetwork.ETA));    % S/I
    else
        %% Evaluation of SINR at user side.
        % total transmit power of BS: 35dBm
        % path loss: 37+30log(d), d in meters
        % gaussian noise on the BS frequency band: -104dBm
        SIR = 10^((35-37-30*log10(dist)+104)/10);        % S/N
    end
    
    log_SIR = log2(1+SIR);
    id = find(log_SIR >= SIR_TH);
    this.user_bs{i} = [id log_SIR(id)];
    if isempty(id) == 0
        b_feasible(i) = true;
    end
end
this.feasible_user_index = this.users(logical(b_feasible),3) - this.UserIdOffset;
% this.feasible_user_bs = user_bs(logical(b_feasible));
% clear user_bs;
this.bs_user = cell(this.NumberBS,1);
for i = 1:this.NumberBS
    user_list = zeros(this.NumberFeasibleUser,2);
    l = 0;
    for j = 1:this.NumberFeasibleUser
        candidate_bs = this.user_bs{this.feasible_user_index(j)};
        for k = 1:size(candidate_bs,1)
            if candidate_bs(k,1) == i
                l = l + 1;
                user_list(l,:) = [j candidate_bs(k,2)];
                break;
            end
        end
    end
    this.bs_user{i} = user_list(1:l,:);
end
end