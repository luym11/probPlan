function NeighborMat = neighbor_matrix(n)
% create a matrix whose indices indicate the neighbor of each entry
    NeighborMat = zeros(n^2, n^2);
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
    
    for j = 1:n % upper line, i=1
        if j == 1
            NeighborMat(1,2) = 1;NeighborMat(1,n+1) = 1;NeighborMat(1,n+2) = 1;
        elseif j == n
            NeighborMat(n,n+n) = 1;NeighborMat(n,n-1) = 1;NeighborMat(n,n+n-1) = 1;
        else
            NeighborMat(j,j-1) = 1; % left
            NeighborMat(j,j+1) = 1; % right
            NeighborMat(j,j+n-1) = 1; % down-left
            NeighborMat(j,j+n) = 1; % down
            NeighborMat(j,j+n+1) = 1; % down-right
        end
    end
    
    for j = 1:n % bottom line, i=n
        if j == 1
            NeighborMat((n-1)*n+j,(n-2)*n+j) = 1;NeighborMat((n-1)*n+j,(n-1)*n+j+1) = 1;NeighborMat((n-1)*n+j,(n-2)*n+j+1) = 1;
        elseif j == n
            NeighborMat((n-1)*n+j,(n-2)*n+j) = 1;NeighborMat((n-1)*n+j,(n-1)*n+j-1) = 1;NeighborMat((n-1)*n+j,(n-2)*n+j-1) = 1;
        else
            NeighborMat((n-1)*n+j,(n-1)*n+j-1) = 1; % left
            NeighborMat((n-1)*n+j,(n-1)*n+j+1) = 1; % right
            NeighborMat((n-1)*n+j,(n-2)*n+j-1) = 1; % up-left
            NeighborMat((n-1)*n+j,(n-2)*n+j) = 1; % up
            NeighborMat((n-1)*n+j,(n-2)*n+j+1) = 1; % up-right
        end
    end
    
    for i = 2:n-1 % left line, j=1
        NeighborMat((i-1)*n+1,(i-2)*n+1)=1;
        NeighborMat((i-1)*n+1,i*n+1)=1;
        NeighborMat((i-1)*n+1,(i-1)*n+2)=1;
        NeighborMat((i-1)*n+1,(i-2)*n+2)=1;
        NeighborMat((i-1)*n+1,i*n+2)=1;
    end
    
    for i = 2:n-1 % right line, j=n
        NeighborMat((i-1)*n+n,(i-2)*n+n)=1;
        NeighborMat((i-1)*n+n,i*n+n)=1;
        NeighborMat((i-1)*n+n,(i-1)*n+n-1)=1;
        NeighborMat((i-1)*n+n,(i-2)*n+n-1)=1;
        NeighborMat((i-1)*n+n,i*n+n-1)=1;
    end
end