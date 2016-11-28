%% Minimize Maximum Link Utilization Ratio and Minimize Resource Useage
% The problem is solved by Linear Programming with objective of minimizing maximum link
% utilization ratio and minimizing resource useage.
%
% *Note*: Using sparse matrix to relax the memory limit.
%% Method Declarion
%   [ theta, flow_variables ] = MinMaxLinkUtilizationMinResource( this, traffic_matrix_id )
%
% |flow_variables|: (K*L) matrix, one row represent the flow variables of one flow.
function [ theta, flow_variables ] = MinMaxLinkUtilizationMinResource( this, traffic_matrix_id )
% Graph parameters;
graph = this.graph;
flow_list = this.TrafficProfiles{traffic_matrix_id};
L = length(graph.Capacity);
N = size(graph.Adjacent, 1);
% Flow parameters
K = size(flow_list, 1);
% incident matrix of graph, convert to index-tuple for spasre matrix
n = [graph.Head; graph.Tail];  % link e from head(e) to tail(e)
e = [(1:L)'; (1:L)'];
b = [-ones(L,1); ones(L,1)];   % flow-out is negative, flow-in is positive
                               % this setting should be consistent with demand (demand at
                               % source is positive, at sink is negative.)                               
VN = zeros(K*L*2,1);
VE = zeros(K*L*2,1);
VB = zeros(K*L*2,1);
% fill each diagonal block
for t=0:2:2*(K-1)
    VN(t*L+1:(t+2)*L) = n+t/2*N;    % index in A-sparse, offset by (t/2)*N from A
    VE(t*L+1:(t+2)*L) = e+t/2*L;    % index in A-sparse, offset by (t/2)*L from A
    VB(t*L+1:(t+2)*L) = b;          % value in A-sparse, each block has the same value.
end
As = sparse(VN,VE,VB,K*N,K*L+1);

ds = zeros(K*N,1);
for i=1:K
%     src = flow_list(i,1);
%     dest = flow_list(i,2);
%     demand = flow_list(i,3);
    ds((i-1)*N+flow_list(i,1)) = -flow_list(i,3); % net flow at source is negative
    ds((i-1)*N+flow_list(i,2)) = flow_list(i,3);  % net flow at destination is positive
end
ds=sparse(ds);

Is = [repmat(sparse(eye(L)),1,K) -graph.Capacity];
O1s = sparse(L,1);
O2s = [sparse(K*L,1); 0];
%% Step 1: maximizing concurrent flow
% $$
% \begin{array}{rl}
% \min &~ \theta \\
% {\rm s.t.} &\sum\limits_{p\in\mathcal{P}_e}{y(p)}\le\theta c(e), \forall e\in\mathcal{L},\\
% &\sum\limits_{p\in\mathcal{P}_k}{y(p)\ge d(k)},\\
% &y(p)\ge 0, ~\forall p\in\mathcal{P},\\
% & 0\le\theta\le 1,
% \end{array}
% $$
%
% min  [0 0 ... 0 1]*|F|
%                    |0|
% s.t. [A 0]|F| = d
%           |¦È|
%      [I -C].|F| <= 0
%             |¦È|
%      |F| >= 0
%      |¦È|
fs = [sparse(1,K*L) 1];
opt = optimoptions(@linprog, 'Diagnostics', 'on', 'Algorithm', 'interior-point');
[F0, theta, exitflag, output] = linprog(fs,Is,O1s,As,ds,O2s,[],[],opt);
% [~, theta, exitflag, output] = linprog(fs,Is,O1s,As,ds,O2s,[],[],opt);
switch exitflag
    case 1
    case 0
        fprintf('\titeration number %d\n', output.iterations);
        error('Error: linear programming does not converge.');
    case -2
        error('Error: No feasible point exists for the problem.');    
    otherwise
        warning('Linear programming exited without results.');
end
%% step 3: min-resource to eliminate loop
% min  [1 1 ... 1]*F
% s.t. AF = d
%      IF <= ¦¨C
%      0 <= F <= F0 (F >= 0)
%     
% Since the solver has limited precision, the solution of the first step (F0)
% might not fully satisfy the given constraints. Specifically, For AF = d, some equations
% in the constraint do not hold for F = F0. If we give the constraint F <= F0, the problem
% may become infeasible, since the component of the precise solution maight large than the
% correspionding component in F0. As a result, we update ds = As*F0(1:end-1). Thus the
% problem is always feasible.
%
% The _interior-point_ or _dual-simplex_ method cannot reach the initial opoint (the
% solution of the *Step 1* problem). Thus we make the upper-bound of the variables a bit
% larger than F0.
% 
% *NOTE*: The second constraint IF <= ¦¨C might also not be fully satisfied. In this case,
% the calue of ¦¨ should also be updated.
f = ones(K*L,1);
Is = repmat(sparse(eye(L)),1,K);
C = graph.Capacity*theta;
As = sparse(VN,VE,VB,K*N,K*L);
ds = As*F0(1:end-1);            
O2s = sparse(K*L,1);
opt = optimoptions(@linprog, 'Diagnostics', 'on', 'Algorithm', 'interior-point');
[F, ~, exitflag, output] = linprog(f,Is,C,As,ds,O2s,F0(1:end-1)+10^-5,[],opt);
% [F, ~, exitflag, output] = linprog(f,Is,C,As,ds,O2s,[],[],opt);
switch exitflag
    case 1
    case 0
        fprintf('iteration number %d\n', output.iterations);
        error('Error: linear programming does not converge.');
    case -2
        warning('Infeasible demand constraints:');
        v = Is*F0(1:end-1);
        k = find(v>C);
        if isempty(k)
            fprintf('\tNo infeasible constraint.\n');
        else
            fprintf('\t%d, left = %f, right = %f',[k; v(k); C(k)]');
        end
        error('Error: No feasible point exists for the problem.');
    otherwise
        warning('Linear programming exited without results.');
end
F(abs(F)<10^-5)=0;
%% Reshape the output argument flow_variables
% F is (K*L,1) array, every L elements is the flow variables associate with one flow.
% while the variable is stored by column manner by the reshap function, to make one
% row of the array[flow_variables] represent one flow's flow variables, the reshape 
% operation is performed as follows:
%
% * It first reshap as a $L\times K$ matrix;
% * It is transposed as a $K\times L$ matrix, one row having $L$ variables of the flow.
flow_variables = (reshape(F, L, K))';
fprintf('\tLinear programming optimal: ¦È = %.4f(¦Ë = %.4f)\n',theta, 1/theta);

end

