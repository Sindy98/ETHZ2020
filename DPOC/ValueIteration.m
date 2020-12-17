function [ J_opt, u_opt_ind ] = ValueIteration(P, G)
%VALUEITERATION Value iteration
%   Solve a stochastic shortest path problem by Value Iteration.
%
%   [J_opt, u_opt_ind] = ValueIteration(P, G) computes the optimal cost and
%   the optimal control input for each state of the state space.
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
J_p = zeros(K,1);
J_p(TERMINAL_STATE_INDEX,1) = 0;
u_p = zeros(K,1);
d_min = Inf;
d_thres = 0.0001;
while(d_min>d_thres)
    d_min = 0;
    for i = 1:K
        [u_p(i,1),v_c] = next_best_a_v(i,P,G,J_p);
        d_min = max(d_min,abs(v_c-J_p(i,1)));
        J_p(i,1)=v_c;        
    end 
end
u_p(TERMINAL_STATE_INDEX,1) = HOVER;
J_opt = J_p;
u_opt_ind = Index2Input(u_p);

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
