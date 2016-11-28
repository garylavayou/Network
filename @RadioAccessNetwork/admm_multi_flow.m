function admm_multi_flow(this, objective )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
switch nargin
    case 1
        OBJ = Objective.MAX_CONCURRENT;
    case {2,3}
        OBJ = objective;
    otherwise
        error('error: wrong number of input arguments.');
end
graph = this.ran_graph;
path_set = this.path_set;
L1 = length(graph.WireCapacity);
K = this.NumberFlow;
P = path_set.Number;

%% User-Path/BS-Path/Wirelink-Path Incidence Matrix
Gs = zeros(graph.NumberUser, P);
% L2 = length(this.ran_graph.DownlinkSE);
Ss = zeros(graph.NumberBS, P);
Hs = zeros(L1, P);
for p = 1:P
    pid = path_set.reverse_index(p,:);
    path = path_set.Element{pid(1),pid(2)}(pid(3));
    Gs(path.Tail-this.UserIdOffset,p) = 1;
    e = path.TailLink;
    b = e(1);
    Ss(b,p) = 1/graph.Adjacent(e(1), e(2));         % 1/DownlinkSE
    for j = 1:path.Length-2
        e = path.Link(j);
        eid = graph.Inverse(e(1), e(2));
        Hs(eid, p) = 1;
    end
end
d = this.flow_table(:,3);
R = this.BSBandwidth*ones(graph.NumberBS,1);
C = graph.WireCapacity;

% opt = optimoptions(@quadprog, 'Diagnostics', 'off');
obj_id = bitand(OBJ,15);
rho = 0.01;             % TODO: how to decide the value of ¦Ñ during the iteration.
switch obj_id
    case Objective.MAX_CONCURRENT
        x = ones(P+1,1);
%         y = ones(P+1,1);
        w = ones(P+1,1);
        Qs = rho*eye(P+1);
        A1s = [Hs sparse(L1, 1)];
        A2s = [Ss sparse(graph.NumberBS,1)];
        A1eqs = [Gs -d];
        O1s = sparse(K,1);
        O2s = sparse(P+1,1);
        w0 = [zeros(P,1); 1];
        while true
            f = -rho*x - w;
%             x = quadprog(H,f,A,b,Aeq,beq,lb,ub,x0,options)
            [y, fval] = quadprog(Qs, f, A2s, R, [],[], O2s);
            fprintf('L2 = %.4f.\n', fval);
            f = w - rho*y-w0;
            [x, fval] = quadprog(Qs, f, A1s, C, A1eqs, O1s, O2s);
            fprintf('L1 = %.4f.\n', fval);
            stop_cond = mean(abs(x-y));
            if stop_cond < 10^-5    % TODO: stop criterion
                break;
            end
            w = w + rho*(x-y);
        end
        this.fobj.objective = x(end);
        this.fobj.rate = this.fobj.objective*this.flow_table(:,3);
        fprintf('\tLinear programming optimal: ¦Ë = %.4f, system throughput = %.4f\n',...
            this.fobj.objective, sum(this.fobj.rate));  
    otherwise
        error('error: unidentified objective.');
end
for p = 1:P
    pid = path_set.reverse_index(p,:);
    path_set.Element{pid(1),pid(2)}(pid(3)).bandwidth = x(p);
end
end

