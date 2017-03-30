function getDiffusion(To,Tn,t)
    m = numel(Tn);
    cfl = 0.01;
    B = [cfl*ones(m,1) cfl*vertcat(-1,-2*ones(m-2,1),-1)+1 cfl*ones(m,1)];
    A = zeros(m);
    A = spdiags(B,[-1 0 1],A);
    [P,D] = eig(full(A));
    Pinv = P^-1;
    a = Pinv*To;
    b = Pinv*Tn;
    % Remove odd modes and modes that have flipped sign because small
    % magnitude
    idx = (abs(a)>1E-6)&(abs(b)>1E-6);
    idx(end)=0;
    a=a(idx);
    b=b(idx);
    d=diag(D);
    d=d(idx);
    N = log(abs(b./a))./log(d);
    %plot(a)
    %plot(b)
    %plot(diag(D))
    plot(N)
    n = mean(N);
    alpha = cfl*n/t
end