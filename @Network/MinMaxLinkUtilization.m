%% Minimizing Maximum Link Utilization Ratio
% The problem is solved by Linear Programming with objective of minimizing maximum link
% utilization ratio.
% *Note*: Using sparse matrix to relax the memory limit.
%% Function Declarion
%   function [theta, flow_variables] = MinMaxUtilizationRatio( this, traffic_matrix_id )
%
% |flow_variables|: (K*L) matrix, one row represent the flow variables of one flow.
function [ theta, flow_variables ] = MinMaxLinkUtilization( this, traffic_matrix_id )
% Graph parameters;
graph = this.graph;
flow_list = this.TrafficProfiles{traffic_matrix_id};
L = length(graph.Capacity);
N = size(graph.Adjacent, 1);
% Flow parameters
K = size(flow_list, 1);
% incident matrxi of graph, convert to index-tuple for spasre matrix
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
%% Linear programming
% min  [0 0 ... 0 1]*|F|
%                    |0|
% s.t. [A 0]|F| = d
%           |θ|
%      [I -C].|F| <= 0
%             |θ|
%      |F| >= 0
%      |θ|
fs = [sparse(1,K*L) 1];
opt = optimoptions(@linprog, 'Diagnostics', 'on', 'Algorithm', 'interior-point');
[F, theta, exitflag, output] = linprog(fs,Is,O1s,As,ds,O2s,[],[],opt);
if exitflag ~= 1
    fprintf('iteration number %d\n', output.iterations);
    error('error: linear programming does not converge.');
end
F(abs(F)<10^-5)=0;
%% Reshape the output argument flow_variables
%     F is (K*L,1) array, every L elements is the flow variables associate with one flow.
%     while the variable is stored by column manner by the reshap function, to make one
%     row of the array[flow_variables] represent one flow's flow variables, the reshape 
%     operation is performed as follows:
%         it first reshap as a (L,K) matrix;
%         it is transposed as a (K,L) matrix, one row having L variables of the flow.
flow_variables = (reshape(F(1:end-1), L, K))';
fprintf('\tLinear programming optimal: θ = %.4f(λ = %.4f)\n',theta, 1/theta);

end

