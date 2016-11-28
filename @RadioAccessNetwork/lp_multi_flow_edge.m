function lp_multi_flow_edge(this, objective, max_rate )
%OPTM_MULTI_FOW Optimzing concurrent commodity flows in wireless network
% TODO: not fullly debugged after revision.
switch nargin
    case 1
        OBJ = Objective.MAX_CONCURRENT;
    case {2,3}
        OBJ = objective;
    otherwise
        error('error: wrong number of input arguments.');
end
graph = this.ran_graph;
% Graph parameters;
L = length(graph.Head);
L1 = length(graph.WireCapacity);
L2 = length(graph.DownlinkSE);
N = size(graph.Adjacent, 1);
% Flow parameters
K = size(this.flow_table, 1);
n_fv = L*K;
% Incident matrxi of Graph, convert to index-tuple for spasre matrix
n = [graph.Head; graph.Tail];  % link e from head(e) to tail(e)
e = [(1:L)'; (1:L)'];
b = [-ones(L,1); ones(L,1)];   % flow-out is negative, flow-in is positive
                               % this setting should be consistent with demand (demand at
                               % source is positive, at sink is negative.)  
Gi = zeros(K*L*2,1);
Gj = zeros(K*L*2,1);
Gv = zeros(K*L*2,1);
% fill each diagonal block
for t=0:2:2*(K-1)
    Gi(t*L+(1:2*L)) = n+t/2*N;    % index in A-sparse, offset by (t/2)*N from A
    Gj(t*L+(1:2*L)) = e+t/2*L;    % index in A-sparse, offset by (t/2)*L from A
    Gv(t*L+(1:2*L)) = b;          % value in A-sparse, each block has the same value.
end
Gs = sparse(Gi,Gj,Gv,K*N,K*L);
ds = zeros(K*N,1);
for i=1:K
%     src = this.flow_table(i,1);
%     dest = this.flow_table(i,2);
%     demand = this.flow_table(i,3);
    ds((i-1)*N+this.flow_table(i,1)) = -this.flow_table(i,3); % net flow at source is negative
    ds((i-1)*N+this.flow_table(i,2)) = this.flow_table(i,3);  % net flow at destination is positive
end
ds=sparse(ds);

IOs = repmat(sparse([eye(L1) zeros(L1, L2)]),1,K);
SOs = repmat(sparse([zeros(graph.NumberBS, L1) graph.BSIncidence]),1,K);
R = BS_BANDWIDTH*ones(graph.NumberBS,1);
O1s = sparse(K*N,1);

%% Linear programming
opt = optimoptions(@linprog, 'Diagnostics', 'on');
obj_id = bitand(OBJ,15);
switch obj_id
    case Objective.MAX_CONCURRENT
        %% objective 1: max-concurrent
        % min  [0 0 ... 0 -1]*|F|
        %                     |¦Ë|
        % s.t. |IOs 0| |F|    |C|
        %      |SOs 0|.|¦Ë| <= |R|    -->  b     ****** S*F <= R if n and l are not
        %      incidented, then S(n,l) = 0
        %
        %      |Gs -d|.|F|  = 0
        %              |¦Ë|
        %
        %              |F| >= 0
        %              |¦Ë|
        % Note: Since limited by the concurrent constraint, it does not necessarily
        % achieve max thoughput
        f = [sparse(K*L,1); -1];
        As = [IOs sparse(L1, 1);...
            SOs sparse(Graph.NumberBS,1)];
        b = [Graph.WireCapacity; R];
        Aeqs = [Gs -ds];
        O2s = sparse(K*L+1,1);
        % [x,fval,exitflag,output]=linprog(f,A, b,Aeq, beq,lb, ub)
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
        fprintf('\tLinear programming optimal: ¦Ë = %.4f\n',this.fobj.objective);
        this.fobj.f1 = reshape(F(1:L*K), L, K);
        this.fobj.f1(abs(this.fobj.f1)<10^-5) = 0;
    case {Objective.MAX_MIN, Objective.MAX_MIN_WEIGHT, Objective.PROPORTION_FAIRNESS}
        %% objective 2/3/4: max-min , weighted max-min or proportional fairness
        % min  [0 0 ... 0 -1] * [F; d; d_min]
        % s.t. |IOs 0  0| |F|       |C|
        %      |SOs 0  0|.|d|    <= |R| --> b
        %      |0  -I  W| |d_min|   |0|
        %
        %                  |F|
        %      |Gs -Ids 0|.|d|    = 0   --> O1s
        %                  |d_min|
        %
        %            [F;d;d_min] >= 0   --> O2s
        % Note: weighted max-min is equal to max-concurrent
        f = [sparse(K*L+K,1); -1];
        if obj_id == 2
            W = ones(K,1);
        elseif obj_id == 3
            W = ones(K,1).*this.flow_table(:,3)/min(this.flow_table(:,3));
        elseif obj_id == 4
            DownlinkAdjacent = Graph.Adjacent(1:Graph.NumberBS, ...
                (1:Graph.size_User) + (Graph.NumberBS + Graph.size_Router));
            W = sum(DownlinkAdjacent, 1);
            W = W'./min(W);
        end
        As = [IOs sparse(L1, K+1);...
            SOs sparse(Graph.NumberBS, K+1);...
            sparse(K,K*L), -sparse(eye(K)), W];
        b = [Graph.WireCapacity; R; zeros(K,1)];
        Id = zeros(K*N,K);   % indicator of source and desination
        for k=1:K
            Id((k-1)*N+this.flow_table(k,1),k) = -1;   % indicator of source of flow i
            Id((k-1)*N+this.flow_table(k,2),k) = 1;
        end
        Ids=sparse(Id);
        clear Id;
        Aeqs = [Gs -Ids zeros(K*N,1)];
        O2s = sparse(K*L+K+1,1);
        % [x,~,exitflag,output]  = linprog(f,A, b, Aeq,  beq, lb, ub)
        [F, ~, exitflag, output] = linprog(f,As,b, Aeqs, O1s, O2s,[]);
        if exitflag ~= 1
            fprintf('iteration number %d\n', output.iterations);
            error('error: linear programming does not converge.');
        end
        this.fobj.rate = F(end-K:end-1);
        this.fobj.rate(abs(this.fobj.rate)<10^-5)=0;
        this.fobj.objective = F(end);
        this.fobj.f1 = reshape(F(1:L*K), L, K);
        this.fobj.f1(abs(this.fobj.f1)<10^-5) = 0;
    case {Objective.MIN_RESOURCE, Objective.MIN_RADIO_RESOURCE}
        %% objective 5/6: min resource or min radio resource.
        % min  [1 1 ... 1] * F   / sum(SOs*F)
        % s.t. |IOs|.F    |C|
        %      |SOs|   <= |R|
        %            AF = d
        %            F >= 0
        % Note: this problem may be infeasible, when demand is large.
        if obj_id == 5
            f = ones(K*L,1);
        else
            f = sum(SOs, 1);
        end
        As = [IOs; SOs];
        b = [Graph.WireCapacity; R];
        O2s = sparse(K*L,1);
        % [x,~,exitflag,output]  = linprog(f,A, b, Aeq,beq,lb, ub)
        [F, ~, exitflag, output] = linprog(f,As,b, Gs, ds, O2s,[]);
        if exitflag ~= 1
            fprintf('iteration number %d\n', output.iterations);
            error('error: linear programming does not converge.');
        end
        this.fobj = f'*F;
        F(abs(F)<10^-5)=0;
        this.fobj.flow_variables = reshape(F, L, K);
    case {Objective.MAX_THROUGHPUT,Objective.MAX_THROUGHPUT_LIMIT}
        %% objective 8/9: max-throughput and max-throughput with max-user rate constrain.
        % min  [0 ... 0, -1 ... -1]*[F; d]
        % s.t. |IOs 0|.|F|     |C|
        %      |SOs 0| |d|  <= |R| --> b
        %
        %                |F|
        %      |Gs -Ids|.|d| = 0   --> O1s
        %
        %            [F;d]  >= 0   --> O2s
        %               d   <= d0  --> UBs
        f = [sparse(K*L,1); -ones(K,1)];
        As = [IOs sparse(L1, K);...
            SOs sparse(Graph.NumberBS, K)];
        b = [Graph.WireCapacity; R];
        Id = zeros(K*N,K);   % indicator of source and desination
        for k=1:K
            Id((k-1)*N+this.flow_table(k,1),k) = -1;   % indicator of source of flow i
            Id((k-1)*N+this.flow_table(k,2),k) = 1;
        end
        Ids=sparse(Id);
        clear Id;
        Aeqs = [Gs -Ids];
        O2s = sparse(K*L+K,1);
        if obj_id == 9
            if nargin <=4
                UBs = [Inf*ones(K*L,1); this.flow_table(:,3)];
            else
                UBs = [Inf*ones(K*L,1); ones(K,1)*max_rate];
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
        this.fobj.rate = F(end-K+1:end);
        this.fobj.rate(abs(this.fobj.rate)<10^-5)=0;
        this.fobj.objective = sum(this.fobj.rate);
        this.fobj.f1 = reshape(F(1:L*K), L, K);
        this.fobj.f1(abs(this.fobj.f1)<10^-5) = 0;
    otherwise
        error('error: unidentified objective.');
end
if bitand(OBJ,128) ~= 0
    %% step 2: maxmize throughput
    % min  [0 ... 0, -1 ... -1]*[F; d]
    % s.t. |IOs 0|.|F|     |C|
    %      |SOs 0| |d|  <= |R| --> b
    %
    %                |F|  
    %      |Gs -Ids|.|d| = 0   --> O1s
    %
    %                F  >= 0   --> 
    %                d  >= d0  --> LBs
    f = [sparse(K*L,1); -ones(K,1)];
    As = [IOs sparse(L1, K);...
          SOs sparse(Graph.NumberBS, K)];
    b = [Graph.WireCapacity; R];
    Aeqs = [Gs -Ids];
    LBs = [sparse(K*L,1); F(end-K:end-1)];
    % [x,~,exitflag,output]  = linprog(f,A, b, Aeq,  beq, lb, ub, x0)
    [F, ~, exitflag, output] = linprog(f,As,b, Aeqs, O1s, LBs,[], F(1:end-1));
    if exitflag ~= 1
        fprintf('iteration number %d\n', output.iterations);
        error('error: linear programming does not converge.');
    end
    this.fobj.rate1 = this.fobj.rate;
    this.fobj.rate = F(end-K+1:end);
    this.fobj.rate(abs(this.fobj.rate)<10^-5)=0;
    this.fobj.throughput = sum(this.fobj.rate);
    this.fobj.f2 = reshape(F(1:L*K), L, K);
    this.fobj.f2(abs(this.fobj.f1)<10^-5) = 0;
end
if bitand(OBJ,64) == 0 && bitand(OBJ,32) == 0
    %% step 3: min-resource to eliminate loop
    % min  [1 1 ... 1]*F
    % s.t. Gs*F = ¦Ëd
    %
    %     0 <= F <= F0 
    f = ones(K*L,1);
    O2s = sparse(K*L,1);
    %                                            Gs*F(t)=¦Ëd
%     opt = optimoptions(@linprog, 'MaxIter', 1000);
    [F, ~, exitflag, output] = linprog(f,[],[],Gs,Gs*F(1:n_fv),O2s,F(1:n_fv));
    if exitflag ~= 1
        fprintf('iteration number %d\n', output.iterations);
        error('error: linear programming does not converge.');
    end
    F(abs(F)<10^-5)=0;
    this.fobj.flow_variables = reshape(F, L, K);    
end
if bitand(OBJ,32) ~= 0
   F(abs(F)<10^-5)=0;
   this.fobj.flow_variables = reshape(F(1:K*L), L, K);
end
end