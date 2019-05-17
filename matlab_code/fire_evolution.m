fireMap = [0 1 1 0; 0 0 0 1; 0 0 1 0; 1 0 0 0];
% fireMap = fireMap_plus;

n = size(fireMap, 1);
aug_fireMap = zeros(n+2);
aug_fireMap(2:end-1,2:end-1) = fireMap;

v_original = reshape(fireMap', numel(fireMap), 1);
v = reshape(aug_fireMap', numel(aug_fireMap), 1); % reshape to be a vector
V = repmat(v, [1, numel(aug_fireMap)]);

neighborMat = neighbor_matrix(n+2);
NT = neighborMat';

ind = find(sum(NT,1)==8); 
NT = NT(:,ind);
V = V(:,ind);
selectedV = reshape(V(NT==1), 8, n^2);

% construct pf profile
pf = 0.087;
pfProfile = repmat(pf * [1/sqrt(2);1;1/sqrt(2);1;1;1/sqrt(2);1;1/sqrt(2)],[1,n^2]);


% final step
v_plus = eye(n^2) * v_original + ( ones(n^2,1) - v_original ) .* ( 8*ones(1,n^2) - ones(1,8) * (ones(8,n^2) - pfProfile .* selectedV ) )';

fireMap_plus = reshape(v_plus, n, n)'