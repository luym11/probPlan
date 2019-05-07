function [Gc,Dist] = MeshGraphLeaders(n)

[X,Y] = meshgrid(1:n,1:n);

Gc = zeros(n^2,n^2);
for i = 2:n-1
        for j = 2:n-1
                Gc(i+(j-1)*n,i+(j-1)*n+1) = 1; Gc(i+(j-1)*n+1,i+(j-1)*n) = 1;
                Gc(i+(j-1)*n,i+(j-1)*n-1) = 1; Gc(i+(j-1)*n-1,i+(j-1)*n) = 1;
                Gc(i+(j-1)*n,i+j*n) =1; Gc(i+j*n,i+(j-1)*n) =1;
        end
end
for j = 1:n  %% bottom line
        if j == 1
                Gc(1,2) = 1;Gc(2,1) = 1;
                Gc(1,n+1) = 1;Gc(n+1,1) = 1;
        elseif j == n
                Gc((j-1)*n+1,(j-2)*n+1) = 1; Gc((j-2)*n+1,(j-1)*n+1) = 1;
                Gc((j-1)*n+1,(j-1)*n+2) = 1; Gc((j-1)*n+2,(j-1)*n+1) = 1;
        else
                Gc((j-1)*n+1,(j-2)*n+1) = 1; Gc((j-2)*n+1,(j-1)*n+1) = 1;
        end
end

for j = 1:n  %% upper line
        if j == 1
                Gc(n,n+n) = 1;Gc(n+n,n) = 1;
                Gc(n,n-1) = 1;Gc(n-1,n) = 1;
        elseif j == n
                Gc((j-1)*n+n,(j-2)*n+n) = 1; Gc((j-2)*n+n,(j-1)*n+n) = 1;
                Gc((j-1)*n+n,(j-1)*n+n-1) = 1; Gc((j-1)*n+n-1,(j-1)*n+n) = 1;
        else
                Gc((j-1)*n+n,(j-2)*n+n) = 1; Gc((j-2)*n+n,(j-1)*n+n) = 1;
        end
end

for i = 2:n-1 %% left line
        Gc(i,i+1) = 1;Gc(i+1,i) = 1;
        Gc(i,i + n) = 1; Gc(i+n,i) = 1;
end
for i = 2:n-1 %% right line
        Gc(i+(n-1)*n,i+(n-1)*n+1) = 1;Gc(i+(n-1)*n+1,i) = 1;
end

Gc = triu(Gc);
Gc = Gc + Gc';
Gc = full(spones(Gc));


%{
%leaders added in order
for(i=1:L)
        Gc(i,:) = ones(1,n*n);
end


%%Add leaders randomly
r=zeros(1,L);
for(i=1:L)
        different=0;
        while(different==0)
                different=1;
                r(i)=randi([1,n^2]);
                for(j=1:i-1)
                        if(r(i)==r(j))
                                different=0;
                        end
                end
        end
        Gc(r(i),:) = ones(1,n*n);
end
%}


end

