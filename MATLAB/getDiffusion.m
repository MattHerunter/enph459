% To is initial temp. distribution, Tn is temp. distribution after time t
function getDiffusion(To,Tn,t)
    m = numel(Tn);
    % cfl number is alpha*dt/dx^2
    cfl = 0.001;
    
    % Set up the diffusion matrix for second order centered finite
    % difference
    B = [cfl*ones(m,1) cfl*vertcat(-1,-2*ones(m-2,1),-1)+1 cfl*ones(m,1)];
    A = zeros(m);
    A = spdiags(B,[-1 0 1],A);
    
    % Get the eigenvector decomposition of A and invert the eigenvector
    % matrix (P's cols are A's evects, D's diag are the corresponding
    % evals, so that P*D*Pinv = A
    [P,D] = eig(full(A));
    Pinv = P^-1;
    
    % Magnitudes of modes (last col is DC, first col is high frequency) 
    a = Pinv*To;
    b = Pinv*Tn;
    
    % Remove:
    % Zero/small mag. modes
    % Modes that have flipped sign (likely because of small relative magnitude)
    % Modes that have grown after diffusion
    minRelMag = 1E-3;
    idx = (abs(a)./max(abs(a))>minRelMag)&(abs(b)./max(abs(b))>minRelMag) & (abs(a)>abs(b)) & (a./b > 0);
    
    % Remove DC component
    idx(end)=0;
    a = a(idx);
    b = b(idx);
    
    % Eigenvalues in d are amount of attenutation a mode sees per timestep
    % (1 for DC as diffusion does nothing to DC)
    d = diag(D);
    d = d(idx);
    
    % Solution to the eignevalue decomposition of b_i = (d_i)^n*a_i
    N = log(abs(b./a))./log(d);
    
    % Do we need dx^2 in here?
    alpha_mean = cfl*mean(N)/t
    alpha_max = cfl*max(N)/t
    alpha_end = cfl*N(end)/t
    
    % Plotting
    subplot(2,2,1);
    plot(N);
    title('N');
    
    subplot(2,2,2);
    plot(a);
    title('a');
    
    subplot(2,2,3);
    plot(b);
    title('b');
    
    subplot(2,2,4);
    plot(d);
    title('d');
end