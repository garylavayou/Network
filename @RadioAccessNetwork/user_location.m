%% User Location
% users = user_location(BSs, id_offset, coverage, min_user, max_user)
% users are randomly distributed in the coverage area of each base station.
function user_location(this, min_user, max_user)
% users(1:2): coordinates; users(3): index; user(4)default BS index.
switch nargin
    case 1
        MIN_USER = 1;
        MAX_USER = 6;
    case 2
        MIN_USER = min_user;
        MAX_USER = 6;
    case 3
        MIN_USER = min_user;
        MAX_USER = max_user;        
    otherwise
        error('error: worng number of input arguments.');
end
this.users = zeros(MAX_USER*this.NumberBS,4);
size_user = 0;
rng(20151203);
BSs = this.wire_node(1:this.NumberBS,:);
for i = 1:this.NumberBS    
    num_user = randi([MIN_USER,MAX_USER]);
    while num_user>0
        size_user = size_user + 1;
        a = rand()*2*pi;
        r = sqrt(rand())*this.coverage;         % uniform distribution
%         r = rand()*COVERAGE;                  % central distribution
        this.users(size_user,[1 2 4]) = [[cos(a)*r, sin(a)*r]+BSs(i,1:2), i];
        num_user = num_user - 1;
    end
end
this.users((size_user+1):end,:) = [];
this.users(:,3) = this.UserIdOffset+(1:size_user);
%% coverage

% dist = zeros(num_BS,1);
% for i=1:num_BS
%     d = wire_node(1:num_BS,2:3);
%     d(:,1) = d(:,1)-wire_node(i,2);
%     d(:,2) = d(:,2)-wire_node(i,3);
%     d(i,:) = [];
%     dist(i) = min(sqrt(sum(d.^2,2)))*4/sqrt(3);
% end
% COVERAGE = max(dist)/2;
end