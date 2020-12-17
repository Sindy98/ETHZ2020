function [ J_opt, u_opt_ind ] = LinearProgramming(P, G)
%LINEARPROGRAMMING Linear Programming
%   Solve a stochastic shortest path problem by Linear Programming.
%
%   [J_opt, u_opt_ind] = LinearProgramming(P, G) computes the optimal cost
%   and the optimal control input for each state of the state space.
%
%   Input arguments:
%       P:
%           A (K x K x L)-matrix containing the transition probabilities
%           between all states in the state space for all control inputs.
%           The entry P(i, j, l) represents the transition probability
%           from state i to state j if control input l is applied.
%
%       G:
%           A (K x L)-matrix containing the stage costs of all states in
%           the state space for all control inputs. The entry G(i, l)
%           represents the cost if we are in state i and apply control
%           input l.
%
%   Output arguments:
%       J_opt:
%       	A (K x 1)-matrix containing the optimal cost-to-go for each
%       	element of the state space.
%
%       u_opt_ind:
%       	A (K x 1)-matrix containing the index of the optimal control
%       	input for each element of the state space. Mapping of the
%       	terminal state is arbitrary (for example: HOVER).

global K HOVER

%% Handle terminal state
% Do yo need to do something with the teminal state before starting policy
% iteration ?
global TERMINAL_STATE_INDEX
% IMPORTANT: You can use the global variable TERMINAL_STATE_INDEX computed
% in the ComputeTerminalStateIndex.m file (see main.m)
P(TERMINAL_STATE_INDEX,:,:) = 0;
u_c = ones(K,1)*5;
J_last = valuePolicy(G,P,u_c);
delta = Inf;
while(delta >0.0001)
    for i = 1:K
        u_c(i) = next_best_a_v(i,P,G,J_last);
    end
    tmp = valuePolicy(G,P,u_c);
    delta = max(abs(tmp-J_last));
    J_last = tmp;
end
u_opt_ind =Index2Input(u_c);
J_opt =J_last;
end

function J = valuePolicy(G,P,u)
global K
P_policy = zeros(K,K);
G_policy = zeros(K,1);
for i = 1:K
    G_policy(i,1) = G(i,u(i));
    P_policy(i,:) = P(i,:,u(i));
end
J = LinearCaculate(P_policy,G_policy,K);

end

function J_policy = LinearCaculate(P,G,K)
J_last = zeros(K,1);
threP = 0.00001;
delta = 1;
i = 0;
t = 100;
while(delta>threP&&i<t)
    J_policy =P*J_last+G;
    delta = max(abs(J_policy-J_last));
    J_last = J_policy;
    i = i+1;
    
end
end
function [a_opt,v_opt] = next_best_a_v(k,P,G,J_last)
global K
vs = zeros(5,1);
for i=1:5
    vs(i)=G(k,i);
    for j = 1:K
        vs(i,1) = vs(i,1)+P(k,j,i)*J_last(j,1);
    end
end
[v_opt, a_opt] = min(vs);

end
function u = Index2Input(u_c)
u = zeros(size(u_c,1),1);
global NORTH SOUTH EAST WEST HOVER
action = [NORTH SOUTH EAST WEST HOVER];

for i = 1:size(u_c,1)
    u(i) = action(u_c(i));
    
end
end


