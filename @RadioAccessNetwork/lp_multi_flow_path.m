function lp_multi_flow_path(this, objective, max_rate)
% OPTIMIZE Optimzing concurrent commodity flows in wireless network, this function is
% based on flow-path model.
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
K = this.NumberFlow;        % = graph.NumberUser = this.NumberActiveUser
P = path_set.Number;

%% User-Path/BS-Path/Wirelink-Path Incidence Matrix
Gs = zeros(K, P);
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
O1s = sparse(K,1);
R = this.BSBandwidth*ones(graph.NumberBS,1);

%% Linear programming
opt = optimoptions(@linprog, 'Diagnostics', 'on');
obj_id = bitand(OBJ,15);
switch obj_id
    case Objective.MAX_CONCURRENT
        %% objective 1: max-concurrent
        % min  [0 0 ... 0 -1]*|x|
        %                     |¦Ë|
        % s.t. |H 0|.|x| <= |C|
        %      |S 0|.|¦Ë|    |R|    -->  b     ****** S*F <= R if n and l are not
        %      incidented, then S(n,l) = 0
        %
        %      |Gs -d|.|x|  = 0    --> O1s
        %              |¦Ë|
        %
        %              |x| >= 0    --> O2s
        %              |¦Ë|
        % Note: Since limited by the concurrent constraint, it does not necessarily
        % achieve max thoughput
        f = [sparse(P,1); -1];
        As = [Hs sparse(L1, 1);...
              Ss sparse(graph.NumberBS,1)];
        b = [graph.WireCapacity; R];
        Aeqs = [Gs -d];
        O2s = sparse(P+1,1);
        % [x,fval,exitflag,output]=linprog(f,A, b,Aeq, beq,lb, ub,x0, opt)
        [F, ~, exitflag, output] = linprog(f,As,b,Aeqs,O1s,O2s,[],[], opt);
        if exitflag ~= 1
            switch exitflag
                case 0
                    fprintf('iteration number %d\n', output.iterations);
                    error('error: linear programming does not converge.');
                otherwise
                    error('error: linear programming failed.');
            end
        end
        this.fobj.objective = F(end);
        this.fobj.rate = this.fobj.objective*this.flow_table(:,3);
        fprintf('\tLinear programming optimal: ¦Ë = %.4f, system throughput = %.4f\n',...
            this.fobj.objective, sum(this.fobj.rate));    
    case {Objective.MAX_MIN, Objective.MAX_MIN_WEIGHT, Objective.PROPORTION_FAIRNESS}
        %% objective 2/3/4: max-min , weighted max-min or proportional fairness
        % min  [0 0 ... 0 -1] * [x; r; r_min]
        % s.t. |H  0  0| |x|       |C|
        %      |S  0  0|.|r|    <= |R| --> b
        %      |0 -I  W| |r_min|   |0|
        %
        %                |x|
        %      |G -Id 0|.|r|     = 0   --> O1s
        %                |r_min|
        %
        %            [x;r;r_min] >= 0   --> O2s
        % Note: weighted max-min is equal to max-concurrent
        f = [sparse(P+K,1); -1];
        if obj_id == Objective.MAX_MIN
            W = ones(K,1);
        elseif obj_id == Objective.MAX_MIN_WEIGHT
            W = ones(K,1).*this.flow_table(:,3)/min(this.flow_table(:,3));
        elseif obj_id == Objective.PROPORTION_FAIRNESS
            downlink_adjacent = graph.Adjacent(1:graph.NumberBS, ...
                (1:K) + (graph.NumberBS + graph.NumberRouter));
            W = sum(downlink_adjacent, 1);
            W = W'./min(W);
        end
        As = [Hs sparse(L1, K+1);...
            Ss sparse(graph.NumberBS,K+1);...
            sparse(K,P) -eye(K) W];
        b = [graph.WireCapacity; R; zeros(K,1)];
        Aeqs = [Gs -eye(K) zeros(K,1)];
        O2s = sparse(P+K+1,1);
        % [x,~,exitflag,output]  = linprog(f,A, b, Aeq,  beq, lb, ub)
        [F, ~, exitflag, output] = linprog(f,As,b, Aeqs, O1s, O2s,[]);
        if exitflag ~= 1
            switch exitflag
                case 0
                    fprintf('iteration number %d\n', output.iterations);
                    error('error: linear programming does not converge.');
                otherwise
                    error('error: linear programming failed.');
            end
        end
        this.fobj.objective = F(end);
        this.fobj.rate = F(P+(1:K));
        fprintf('\tLinear programming optimal: r_min = %.4f, system throughput = %.4f\n',...
            this.fobj.objective, sum(this.fobj.rate));
    case {Objective.MAX_THROUGHPUT,Objective.MAX_THROUGHPUT_LIMIT}
        %% objective 8/9: max-throughput and max-throughput with max-user rate constrain.
        % min  [0 ... 0, -1 ... -1]*[x; r]
        % s.t. |H  0| |x|    |C|
        %      |S  0|.|r| <= |R| --> b
        %
        %             |x|
        %      |G -I|.|r|  = 0   --> O1s
        %
        %           [x;r] >= 0   --> O2s
        %              r  <= r0  --> UBs
        f = [sparse(P,1); -ones(K,1)];
        As = [Hs sparse(L1, K);...
            Ss sparse(graph.NumberBS,K)];
        b = [graph.WireCapacity; R];
        Aeqs = [Gs -eye(K)];
        O2s = sparse(P+K,1);
        if obj_id == Objective.MAX_THROUGHPUT_LIMIT
            if nargin <= 2
                UBs = [Inf*ones(P,1); this.flow_table(:,3)];
            else
                UBs = [Inf*ones(P,1); ones(K,1)*max_rate];
            end
        else
            UBs = [];
        end
        % [x,~,exitflag,output]  = linprog(f,A, b, Aeq,  beq, lb, ub)
        [F, ~, exitflag, output] = linprog(f,As,b, Aeqs, O1s, O2s,UBs);
        if exitflag ~= 1
            fprintf('iteration number %d\n', output.iterations);
            error('error: linear programming does not converge.');
        end
        this.fobj.rate = F(P+1:end);
        this.fobj.rate(abs(this.fobj.rate)<10^-5)=0;
        this.fobj.objective = sum(this.fobj.rate);
        fprintf('\tLinear programming optimal: system throughput = %.4f\n', ...
            this.fobj.objective);
    otherwise
        error('error: unidentified objective.');
end
if bitand(OBJ,128) ~= 0
    %% step 2: maxmize throughput
    % min  [0 ... 0, -1 ... -1]*[x; r]
    % s.t. |H 0|.|x|     |C|
    %      |S 0| |r|  <= |R| --> b
    %
    %             |x|  
    %      |G -I|.|r|  = 0   --> O1s
    %
    %              x  >= 0   --> 
    %              r  >= r0  --> LBs
    f = [sparse(P,1); -ones(K,1)];
    As = [Hs sparse(L1, K);...
          Ss sparse(graph.NumberBS, K)];
    b = [graph.WireCapacity; R];
    Aeqs = [Gs -eye(K)];
    LBs = [sparse(P,1); F(P+(1:K))];
    % [x,~,exitflag,output]  = linprog(f,A, b, Aeq,  beq, lb, ub, x0)
    [F, ~, exitflag, output] = linprog(f,As,b, Aeqs, O1s, LBs,[], F(1:P+K));
    if exitflag ~= 1
        fprintf('iteration number %d\n', output.iterations);
        error('error: linear programming does not converge.');
    end
    this.fobj.rate0 = this.fobj.rate;
    this.fobj.rate = F(P+(1:K));
    this.fobj.rate(abs(this.fobj.rate)<10^-5)=0;
    this.fobj.objective0 = this.fobj.objective;
    this.fobj.objective = sum(this.fobj.rate);
    if OBJ == Objective.MAX_THROUGHPUT_LIMIT_T
        fprintf('\tLinear programming optimal: system throughput = %.4f\n', ...
            this.fobj.objective);
    else
        fprintf('\tLinear programming optimal: r_min = %.4f, system throughput = %.4f\n', ...
            min(this.fobj.rate), this.fobj.objective);
    end
end
for p = 1:P
    pid = path_set.reverse_index(p,:);
    path_set.Element{pid(1),pid(2)}(pid(3)).bandwidth = F(p);
end
end