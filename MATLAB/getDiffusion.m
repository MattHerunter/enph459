% To is initial temp. distribution, Tn is temp. distribution after time t
function getDiffusion(To,Tn,t)
   figure
   idx = findDiffusionWindow(To,Tn);
   To = To(idx(1):idx(2));
   Tn = Tn(idx(1):idx(2));
   calculateDiffusion(To,Tn,t)
end

function calculateDiffusion(To,Tn,t)
    % Length of the data (so dx = 1)
    m = numel(Tn);
    
    % Assume a cfl number (alpha*dt/dx^2) so that the matrix is known.
    % Assumes dt to be some unknown number, as we don't know alpha
    cfl = 1E-5;
    
    % Set up the diffusion matrix for second order centered finite
    % difference
    % Uses Txx(i) = (T(i-1) - 2T(i) + T(i+1))/dx^2, Tt(i) = alpha*Txx(i)
    %B = [cfl*ones(m,1) cfl*vertcat(-1,-2*ones(m-2,1),-1)+1 cfl*ones(m,1)];
    B = [cfl*ones(m,1) cfl*vertcat(1,-2*ones(m-2,1),1)+1 cfl*ones(m,1)];
    A = zeros(m);
    A = spdiags(B,[-1 0 1],A);
    
    A(1,1:2) = cfl*[-2 2]+[1,0];
    
    A(end,(end-1):end) = cfl*[2 -2] + [0 1];
    
    % Get the eigenvector decomposition of A and invert the eigenvector
    % matrix (P's cols are A's evects, D's diag are the corresponding
    % evals, so that P*D*Pinv = A
    [P,D] = eig(full(A));
    
    if P ~= real(P)
        warning('Complex eigenvectors! >:(')
    end
    if D ~= real(D)
        warning('Complex eigenvalues! >:(')
    end
    P = real(P);
    D = real(D);
    
    [~,ord]= sort(sum(abs(diff(sign(P),1)),1));
    P = P(:,ord);
    D = D(ord,ord);
    
    Pinv = P^-1;
    
    % Magnitudes of modes (last col is DC, first col is high frequency) 
    a = Pinv*To;
    b = Pinv*Tn;
    
    % Remove:
    % Zero/small mag. modes
    % Modes that have flipped sign (likely because of small relative magnitude)
    % Modes that have grown after diffusion
    minRelMag = 1E-5;
    idx = (abs(a)./max(abs(a)) > minRelMag) & (abs(b)./max(abs(b)) > minRelMag) & (abs(a) > abs(b)) & (a./b > 0);
    
    % Remove DC component
    idx(abs(sum(sign(P),1))==size(P,1)) = 0;
    
    a = a(idx);
    b = b(idx);
    
    % Eigenvalues in d are amount of attenutation a mode sees per timestep
    % (1 for DC as diffusion does nothing to DC)
    d = diag(D);
    d = d(idx);
    
    % Attempt at PDE craftiness
    n = (1:numel(idx))';
    n = n(idx);
    lambda = n.^2*pi^2/m^2;
    
    % Solution to the eignevalue decomposition of b_i = (d_i)^n*a_i
    N = log(b./a)./log(d);
    
    % Do we need dx^2 in here?
    alpha_mean = cfl*mean(N)/t
    alpha_max = cfl*max(N)/t
    alpha_end = cfl*N(end)/t
    alpha_best = cfl*N(abs(a)==max(abs(a)))/t
    
    % Plotting
    subplot(2,3,1);
    plot(N);
    title('N');
    
    subplot(2,3,2);
    plot(a);
    title('a');
    
    subplot(2,3,3);
    plot(b);
    title('b');
    
    subplot(2,3,4);
    plot(d);
    title('d');
    
    subplot(2,3,5);
    plot(cfl*N/t);
    title('\alpha');
    
    subplot(2,3,6);
    plot(a./b);
    title('a/b');
end

function idx = findDiffusionWindow(To,Tn)
    % Chop off ends of signals to avoid false zero derivative readings
    m = numel(To);
    b = ceil(m/100);
    To = To(b:end-b);
    Tn = Tn(b:end-b);
    m = numel(To);
    
    % Sum of the slope magnitudes (only zero when both slopes are zero)
    d = abs(diff(To)) + abs(diff(Tn));
    
    % Split the signal into the first and second half. May be a better way
    % of splitting it up (around maximum or something like that)
    d1 = d(1:floor(end/2));
    d2 = d(ceil(end/2)+1:end);
    
    % Get the indices of minimum slope sum
    idx = [0 0];
    [~,idx(1)] = min(d1);
    [~,idx(2)] = min(d2);
    idx(2) = idx(2) + numel(d2);
    
    idx = sort(idx);
    
    x = (1:m)';
    plot(x, To, x, Tn, idx, To(idx), 'xr')
end