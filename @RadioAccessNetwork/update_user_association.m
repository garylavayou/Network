%% update user-BS association
function update_user_association(this)
% since excluding the inactive user, a new fake_bs_user must be stored, while user_bs
% association does not change.
% fake_bs_user store the index of fake_user_id
active_user_bs = this.user_bs(this.active_user_index);
this.fake_bs_user = cell(this.NumberBS,1);
b_active_bs = zeros(this.NumberBS,1);
for i = 1:this.NumberBS
    user_list = zeros(this.NumberActiveUser,2);
    l = 0;
    for j = 1:this.NumberActiveUser
        for k = 1:size(active_user_bs{j},1)
            if active_user_bs{j}(k,1) == i    % --> user(j)'s BS is BS(i)
                l = l + 1;
                user_list(l,:) = [j active_user_bs{j}(k,2)];
                break;
            end
        end
    end
    this.fake_bs_user{i} = user_list(1:l,:);
    if l > 0
        b_active_bs(i) = 1;
    end
end
bs_index = [this.index.NBS; this.index.GBS];
this.active_bs_index = bs_index(logical(b_active_bs));
end