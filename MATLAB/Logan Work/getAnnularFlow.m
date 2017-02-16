% Get the average velocity through an annular sector [a,b,phi]
function averageVelocity = getAnnularFlow(a,b,phi)
    
    % Variable parameters
    k = b/a;
    
    % Discretization.
    resolution = 500;
    R = linspace(a,b,resolution+1);
    Theta = linspace(0,phi,resolution+1);
    
    [r,theta] = meshgrid(R,Theta);
    alpha = @(k,x) sinh(k*pi/phi*log(a./x));
    beta = @(k,x) sinh(k*pi/phi*log(b./x));
    
    % Compute series to N terms.
    N = 100;
    
    %     s = zeros(resolution+1,resolution+1);
    %     for n=1:N
    %         % This condition only necessary to avoid dividing 0/0. Otherwise these
    %         % terms are zero anyway because of (1-(-1)^n).
    %         if mod(n,2)==1
    %             s = s + 2*phi^2*(1-(-1)^n)/(n*pi*(n^2*pi^2-4*phi^2)).*(b^2*alpha(n,r)/alpha(n,b)+a^2*beta(n,r)/beta(n,a)).*sin(n*pi*theta/phi);
    %         end
    %     end
    %
    %     % Compute PDE solution.
    %     u = (1/4)*r.^2.*(1-cos(2*theta-phi)/cos(phi)) + s;
    
    % Integral of velocity over area.
    S = 0;
    for n=1:N
        % This condition only necessary to avoid dividing 0/0. Otherwise these
        % terms are zero anyway because of (1-(-1)^n).
        if mod(n,2)==1
            S = S + 2*phi^5*(1-(-1)^n)^2*a^4/(n*pi*(n^2*pi^2-4*phi^2))^2*((1+k^4)*n*pi/phi*coth(n*pi/phi*log(k))+2*(1-k^4)-2*k^2*n*pi/phi*csch(n*pi/phi*log(k)));
        end
    end
    Q = (1/16)*a^4*(k^4-1)*(phi-tan(phi)) + S;
    
    % Divide by area.
    A = (phi/2)*a^2*(k^2-1);
    averageVelocity = -Q/A;
end