function NeighborMat1 = neighbor_matrix1(n)
    m = n + 2;
    NeighborMat = zeros(m^2, m^2);
    for i = 2:n-1
        for j = 2:n-1
            NeighborMat(j+(i-1)*n,j+(i-1)*n-1) = 1; % left
            NeighborMat(j+(i-1)*n,j+(i-1)*n+1) = 1; % right
            NeighborMat(j+(i-1)*n,j+(i-2)*n) = 1; % up
            NeighborMat(j+(i-1)*n,j+i*n) =1; % down
            NeighborMat(j+(i-1)*n,j+(i-2)*n-1) = 1; % up-left
            NeighborMat(j+(i-1)*n,j+(i-2)*n+1) = 1; % up-right
            NeighborMat(j+(i-1)*n,j+i*n-1) =1; % down-left
            NeighborMat(j+(i-1)*n,j+i*n+1) =1; % down-right
        end
    end
    NeighborMat1 = NeighborMat(2:n^2-1,2:n^2-1);
end

