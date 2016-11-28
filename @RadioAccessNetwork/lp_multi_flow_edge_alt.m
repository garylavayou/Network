function lp_multi_flow_edge_alt(this, objective, max_rate )
%OPTM_MULTI_FOW2 Optimzing concurrent commodity flows in wireless network
% Note: the diffenence between OPTM_MULTI_FLOW2 and OPTM_MULTI_FLOW lies in the 
%   organization of the variables. this method need to reorganize the flow varibles as
%   well incidence matrix, while OPTM_MULTI_FLOW only needs to adjust the arrangement of
%   the inequality coefficient. Therefore, OPTM_MULTI_FLOW is simpler.
% TODO: not fullly debugged after revision.
switch nargin
    case 1
        OBJ = Objective.MAX_CONCURRENT;
    case {2,3}
        OBJ = objective;
    otherwise
        error('error: wrong number of input arguments.');
end
% Graph parameters;
L = length(graph.Head);
L1 = length(graph.WireCapacity);
L2 = length(graph.DownlinkSE);
N = size(graph.Adjacent, 1);
% Flow parameters
K = size(this.flow_table, 1);
% Incident matrxi of Graph, convert to index-tuple for spasre matrix
n = [graph.Head; graph.Tail];  % link e from head(e) to tail(e)
e = [(1:L)'; (1:L)'];
b = [-ones(L,1); ones(L,1)];   % flow-out is negative, flow-in is positive
                               % this setting should be consistent with demand (demand at
                               % source is positive, at sink is negative.)  
%% construct G1 and G2
% 2 methods: (1) split G; (2) separately construct G1 and G2
% opt = 1;   
opt = 2;
if opt == 1
    Gi = zeros(K*L*2,1);
    Gj = zeros(K*L*2,1);
    Gv = zeros(K*L*2,1);
    index_1 = zeros(1,K*L1);
    index_2 = zeros(1,K*L2);
    % fill each diagonal block
    for t=0:2:2*(K-1)
        Gi(t*L+(1:2*L)) = n+t/2*N;    % index in A-sparse, offset by (t/2)*N from A
        Gj(t*L+(1:2*L)) = e+t/2*L;    % index in A-sparse, offset by (t/2)*L from A
        Gv(t*L+(1:2*L)) = b;          % value in A-sparse, each block has the same value.
        index_1(t/2*L1+(1:L1)) = t/2*L+(1:L1);
        index_2(t/2*L2+(1:L2)) = t/2*L+L1+(1:L2);
    end
    Gs = sparse(Gi,Gj,Gv,K*N,K*L);
    G1s = Gs(:,index_1);
    G2s = Gs(:,index_2);
    clear Gi Gj Gv index_1 index_2 Gs;
elseif opt == 2
    % G1
    index_1 = [1:L1 (1:L1)+L];
    Gi = zeros(K*L1*2,1);
    Gj = zeros(K*L1*2,1);
    Gv = zeros(K*L1*2,1);
    % fill each diagonal block
    for t=0:2:2*(K-1)
        Gi(t*L1+(1:2*L1)) = n(index_1)+t/2*N;    % index in A-sparse, offset by (t/2)*N from A
        Gj(t*L1+(1:2*L1)) = e(index_1)+t/2*L1;   % index in A-sparse, offset by (t/2)*L from A
        Gv(t*L1+(1:2*L1)) = b(index_1);          % value in A-sparse, each block has the same value.
    end
    G1s = sparse(Gi,Gj,Gv,K*N,K*L1);
    % G2
    index_2 = [L1+1:L (L1+1:L)+L];
    Gi = zeros(K*L2*2,1);
    Gj = zeros(K*L2*2,1);
    Gv = zeros(K*L2*2,1);
    % fill each diagonal block
    for t=0:2:2*(K-1)
        Gi(t*L2+(1:2*L2)) = n(index_2)+t/2*N;    % index in A-sparse, offset by (t/2)*N from A
        Gj(t*L2+(1:2*L2)) = e(index_2)-L1+t/2*L2;% index in A-sparse, offset by (t/2)*L from A
        Gv(t*L2+(1:2*L2)) = b(index_2);          % value in A-sparse, each block has the same value.
    end
    G2s = sparse(Gi,Gj,Gv,K*N,K*L2);
    clear Gi Gj Gv index_1 index_2;
end

ds = zeros(K*N,1);
for i=1:K
%     src = this.flow_table(i,1);
%     dest = this.flow_table(i,2);
%     demand = this.flow_table(i,3);
    ds((i-1)*N+this.flow_table(i,1)) = -this.flow_table(i,3); % net flow at source is negative
    ds((i-1)*N+this.flow_table(i,2)) = this.flow_table(i,3);  % net flow at destination is positive
end
ds=sparse(ds);
Is = repmat(sparse(eye(L1)),1,K);
Ss = repmat(graph.BSIncidence,1,K);
R = BS_BANDWIDTH*ones(graph.NumberBS,1);

%% Linear programming
% opt = optimoptions(@linprog, 'Diagnostics', 'on');
obj_id = bitand(OBJ,15);
switch obj_id
    case Objective.MAX_CONCURRENT
        %% objective 1: max-concurrent
        %                     |F1|
        % min  [0 0 ... 0 -1].|F2|
        %                     |¦Ë|
        % s.t. |Is 0  0|.|F1|    |C|
        %      |0  Ss 0| |F2| <= |R|  --> b
        %                |¦Ë|
        %
        %                |F1|
        %   [G1s G2s -ds]|F2|  = 0
        %                |¦Ë|
        %
        %                |F1|
        %                |F2| >= 0
        %                |¦Ë|
        f = [sparse(K*L,1); -1];
        As = [Is sparse(L1, K*L2+1);...
            sparse(Graph.NumberBS,K*L1) Ss sparse(Graph.NumberBS,1)];
        b = [Graph.WireCapacity; R];
        Aeqs = [G1s G2s -ds];
        O1s = sparse(K*N,1);
        O2s = sparse(K*L+1,1);
        % [x,fval,exitflag,output]=linprog(f,A, b,Aeq, beq,lb, ub)
        [F, ~, exitflag, output] = linprog(f,As,b,Aeqs,O1s,O2s,[]);
        if exitflag ~= 1
            fprintf('iteration number %d\n', output.iterations);
            error('error: linear programming does not converge.');
        end
        this.fobj.objective = F(end);
        fprintf('\tLinear programming optimal: ¦Ë = %.4f\n',fobj);
        t = 1:length(F)-1;
    case {Objective.MAX_MIN, Objective.MAX_MIN_WEIGHT, Objective.PROPORTION_FAIRNESS}
        %% objective 3/5: max-min or weighted max-min
        % min  [0 0 ... 0 1] * [F1; F2; d; d_min]
        % s.t. |Is 0 0  0| |F1|      |C|
        %      |0 Ss 0  0|.|F2|   <= |R| --> b
        %      |0  0 -I 1| |d|       |0|
        %                  |d_min|
        %                  |F|
        % |G1s G2s -Ids 0|.|d|     = 0   --> O1s
        %                  |d_min|
        %            [F;d;d_min] >= 0    -->O2s
        % Note: weighted max-min is equal to max-concurrent
        f = [sparse(K*L+K,1); -1];
        if OBJ == 3
            U1 = ones(K,1);
        elseif OBJ == 5
            U1 = ones(K,1).*this.flow_table(:,3)/min(this.flow_table(:,3));
        end
        As = [Is sparse(L1, K*L2+K+1);...
            sparse(Graph.NumberBS,K*L1) Ss sparse(Graph.NumberBS,K+1);...
            sparse(K,K*L), -sparse(eye(K)), U1];
        b = [Graph.WireCapacity; R; zeros(K,1)];
        Id = zeros(K*N,K);   % indicator of source and desination
        for k=1:K
            Id((k-1)*N+this.flow_table(k,1),k) = -1;   % indicator of source of flow i
            Id((k-1)*N+this.flow_table(k,2),k) = 1;
        end
        Ids=sparse(Id);
        clear Id;
        Aeqs = [G1s G2s -Ids zeros(K*N,1)];
        O1s = sparse(K*N,1);
        O2s = sparse(K*L+K+1,1);
        % [x,~,exitflag,output]  = linprog(f,A, b, Aeq,  beq, lb, ub)
        [F, ~, exitflag, output] = linprog(f,As,b, Aeqs, O1s, O2s,[]);
        if exitflag ~= 1
            fprintf('iteration number %d\n', output.iterations);
            error('error: linear programming does not converge.');
        end
        this.fobj.rate = F(end-K:end);
        t = 1:length(F)-K-1;
    case {Objective.MIN_RESOURCE, Objective.MIN_RADIO_RESOURCE}
        %% objective 2: min resource
        % min  [1 1 ... 1].|F1|
        %                  |F2|
        % s.t. |Is 0 |.|F1| <= |C|  --> b
        %      |0  Ss| |F2|    |R|
        %
        %        |G1 G2||F1| = ds
        %               |F2|
        %
        %           [F1 F2] >= 0
        % Note: this problem may be infeasible, when demand is large.
        f = ones(K*L,1);
        As = [Is sparse(L1, K*L2);...
            sparse(Graph.NumberBS,K*L1) Ss];
        b = [Graph.WireCapacity; R];
        Aeqs = [G1s G2s];
        O2s = sparse(K*L,1);
        % [x,~,exitflag,output]  = linprog(f,A, b,Aeq, beq,lb, ub)
        [F, ~, exitflag, output] = linprog(f,As,b,Aeqs,ds, O2s,[]);
        if exitflag ~= 1
            fprintf('iteration number %d\n', output.iterations);
            error('error: linear programming does not converge.');
        end
        this.fobj.objective = f'*F;
        F(abs(F)<10^-5)=0;
        this.fobj.flow_variables = [reshape(F(1:L1*K), L1, K); reshape(F(L1*K+(1:L2*K)), L2, K)];
end
if bitand(OBJ,64) == 0 && bitand(OBJ,32) == 0
    %% step 2: eliminate loop
    % min  [1 1 ... 1]*F
    % s.t. |G1s G2s|*|F1| = ¦Ëd
    %                |F2|
    %
    %     0 <= F <= F0 
    f = ones(K*L,1);
    O2s = sparse(K*L,1);
    Gs = [G1s G2s];
    %                                            Gs*F(t)=¦Ëd
    [F, ~, exitflag, output] = linprog(f,[],[],Gs,Gs*F(t),O2s,F(t));
    if exitflag ~= 1
        fprintf('iteration number %d\n', output.iterations);
        error('error: linear programming does not converge.');
    end
    F(abs(F)<10^-5)=0;
    this.fobj.flow_variables = [reshape(F(1:L1*K), L1, K); reshape(F(L1*K+(1:L2*K)), L2, K)];
end
end