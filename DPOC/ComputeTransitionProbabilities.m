function P = ComputeTransitionProbabilities(stateSpace, map)
%COMPUTETRANSITIONPROBABILITIES Compute transition probabilities.
% 	Compute the transition probabilities between all states in the state
%   space for all control inputs.
%
%   P = ComputeTransitionProbabilities(stateSpace, map) 
%   computes the transition probabilities between all states in the state 
%   space for all control inputs.
%
%   Input arguments:
%       stateSpace:
%           A (K x 3)-matrix, where the i-th row represents the i-th
%           element of the state space.
%
%       map:
%           A (M x N)-matrix describing the world. With
%           values: FREE TREE SHOOTER PICK_UP DROP_OFF BASE
%
%   Output arguments:
%       P:
%           A (K x K x L)-matrix containing the transition probabilities
%           between all states in the state space for all control inputs.
%           The entry P(i, j, l) represents the transition probability
%           from state i to state j if control input l is applied.

global GAMMA R P_WIND
global FREE TREE SHOOTER PICK_UP DROP_OFF BASE
global NORTH SOUTH EAST WEST HOVER
global K TERMINAL_STATE_INDEX

[pickM, pickN] = find( map == PICK_UP );
if isempty(pickM)
    error('Error: Invalid Map (No Pick_up)');
end
action = 1:5;
P=zeros ([K,K,5]);
for i = 1:K
    for input_i = action
        [allow,position] = Move(stateSpace(i,:),input_i,map);
        if allow
            states = WindShot(position,map);
            tmp_size = size(states);
            for j = 1:tmp_size
                if(states(j,4)~=0)
                    [~,index] = ismember(states(j,1:3),stateSpace,'rows');
                    P(i,index,input_i) = states(j,4);
                end
            end
        end
    end 
end
P = NebShot(map,P,stateSpace);
%P_(pickup,0)2(pickup,1) = 1;
[x,y] = find(map==PICK_UP);
[a_p,index_Pickup] = ismember([x,y,0],stateSpace,'rows');
if(a_p)

    % % pick_up condition _1 +1 timestep for pickup
    % P(index_Pickup,:,HOVER)=0;
    % P(index_Pickup,index_Pickup+1,HOVER) =1; 

    % pick_up condition _2 +0 for pickup
    P(:,index_Pickup+1,:) = P(:,index_Pickup+1,:)+ P(:,index_Pickup,:);
    P(:,index_Pickup,:) = 0;
    %P(index_Pickup,index_Pickup+1,:)=1
end
%probability to base without package + = the probability of crash
[x_b,y_b] = find(map==BASE);
[a_b,index_Base] = ismember([x_b,y_b,0],stateSpace,'rows');
if(a_b)
    for input_i = 1:size(action)
        for i = 1:K
            p_c = sum(P(K,:,input_i));
            P(K,index_Base,input_i) =  P(K,index_Base,input_i)+1-p_c;
        end
    end
end


%P_end2other = 0, P_end2end=1;
P(TERMINAL_STATE_INDEX,:,:) = 0;
P(TERMINAL_STATE_INDEX,TERMINAL_STATE_INDEX,:) = 1;
end

% allowable input for possible next state from current state
function [allowable,position] = Move(state,input,map)
global TREE
sizeM = size(map);
position = state;
switch(input)
    case 1
        position(2)= state(2)+1;
    case 2
        position(2) = state(2)-1;
    case 3
        position(1) = state(1)+1;
    case 4
        position(1) = state(1)-1;      
end
if (position(1)*position(2)==0||position(1)>sizeM(1)||position(2)>sizeM(2)||map(position(1),position(2))==TREE) %leave bound || to the tree
    allowable = false;
else
    allowable=true;
end
end
function P = NebShot(map,P,stateSpace)
global SHOOTER R GAMMA
[x_Neb,y_Neb] = find(map==SHOOTER);

for i = 1:size(x_Neb)
    state = [x_Neb(i),y_Neb(i),0];
    state_tmp = state;
    for r = -R:R
        for j = abs(r)-R:R-abs(r)
            state_tmp(1) = state(1)+r;
            state_tmp(2) = state(2)+j;
            for k = 0:1
                state_tmp(3) = k;
                [a,index] = ismember(state_tmp,stateSpace,'rows');
                if(a)
                    P(:,index,:) = P(:,index,:)*(1-GAMMA/(abs(r)+abs(j)+1));
                end
            end
            
        end
        
    end
    
end
end
function flyWind= WindShot(state,map) % crash by wind
global P_WIND;
flyWind = zeros(5,4);
for i = 1:4
    [allowable,tmp]=Move(state,i,map);
    if(allowable)
        flyWind(i,:)=[tmp,P_WIND/4];
    end
end
flyWind(5,:) = [state,1-P_WIND];
end







