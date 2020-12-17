function G = ComputeStageCosts(stateSpace, map)
%COMPUTESTAGECOSTS Compute stage costs.
% 	Compute the stage costs for all states in the state space for all
%   control inputs.
%
%   G = ComputeStageCosts(stateSpace, map) 
%   computes the stage costs for all states in the state space for all
%   control inputs.
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
%       G:
%           A (K x L)-matrix containing the stage costs of all states in
%           the state space for all control inputs. The entry G(i, l)
%           represents the expected stage cost if we are in state i and 
%           apply control input l.

    global GAMMA R P_WIND Nc
    global FREE TREE SHOOTER PICK_UP DROP_OFF BASE
    global NORTH SOUTH EAST WEST HOVER
    global K
    global TERMINAL_STATE_INDEX
%% find base state index
[baseM, baseN] = find( map == BASE );
if isempty(baseM)
    error('Error: Invalid Map (No Base)');
elseif ( size(baseM, 1) > 1 )
    error('Error: Invalid Map (Multiple Bases)');
end
[~, baseIndex] = ismember([baseM, baseN, 0], stateSpace, 'row');
if ~baseIndex
    error('Error: State Space Constructed Wrongly')
end
%% initial G as inf  
G = ones(K,5);
G(:,:) =Inf;
%% probability not being shot by neighbor
cost_shot = ones(1,K);
cost_shot = NebShot(map,cost_shot,stateSpace);
%% update G by i and input
for i = 1:K
    for j = 1:5
        [allow,position] = Move(stateSpace(i,:),j,map); % whether this input is allowable. If so update G as 1 timestep
        if allow
            G(i,j)=1;
            % no wind blow
            [~,index_tmp] = ismember(position,stateSpace,'rows');
            G(i,j) = G(i,j)+(1-P_WIND)*(Nc-1)*(1-cost_shot(1,index_tmp)); % cost += P_currentCrash * P_not blow * extra Timestep
            for wind_input = 1:4
                [w_allow,tmp_state]=Move(position,wind_input,map); % whether this would be crashed during wind blow
                if(~w_allow)
                    G(i,j)= G(i,j)+P_WIND*(Nc-1)/4; % if not allow, crash as P_Wind/4 
                else
                   [~,index_tmp] = ismember(tmp_state,stateSpace,'rows'); % index of tmp_state
                   G(i,j) = G(i,j)+(1-cost_shot(1,index_tmp))*P_WIND*(Nc-1)/4; % get probability of crash in the tmp_state 
                end
            end
           
        end
    end 
end
G(TERMINAL_STATE_INDEX,:) =0;

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

