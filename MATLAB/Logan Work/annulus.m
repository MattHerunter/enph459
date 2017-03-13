%Variable parameters.
a = 2;
k = 3;
b = k*a;
phi = pi;

%Discretization.
resolution = 100;
R = linspace(a,b,resolution+1);
Theta = linspace(0,phi,resolution+1);

[r,theta] = meshgrid(R,Theta);
alpha = @(k,x) sinh(k*pi/phi*log(a./x));
beta = @(k,x) sinh(k*pi/phi*log(b./x));

%Compute series to N terms.
N = 50;
s = zeros(resolution+1,resolution+1);
for n=1:N
    %This condition only necessary to avoid dividing 0/0. Otherwise these
    %terms are zero anyway because of (1-(-1)^n).
    if mod(n,2)==1
        s = s + 2*phi^2*(1-(-1)^n)/(n*pi*(n^2*pi^2-4*phi^2)).*(b^2*alpha(n,r)/alpha(n,b)+a^2*beta(n,r)/beta(n,a)).*sin(n*pi*theta/phi);
    end
end

%Compute PDE solution.
u = (1/4)*r.^2.*(1-cos(2*theta-phi)/cos(phi)) + s;

%Verify PDE by finite difference.
dr = (b-a)/resolution;
dt = phi/resolution;
u_r = diff(u,1,2)/dr;
u_r = u_r(2:end-1,2:end);
u_rr = diff(u,2,2)/dr^2;
u_rr = u_rr(2:end-1,:);
u_tt = diff(u,2,1)/dt^2;
u_tt = u_tt(:,2:end-1);
r_pde = r(2:end-1,2:end-1);
laplacian = u_rr + (1./r_pde).*u_r + (1./r_pde.^2).*u_tt;

%Integral of velocity over area.
S = 0;
for n=1:N
    %This condition only necessary to avoid dividing 0/0. Otherwise these
    %terms are zero anyway because of (1-(-1)^n).
    if mod(n,2)==1
        S = S + 2*phi^5*(1-(-1)^n)^2*a^4/(n*pi*(n^2*pi^2-4*phi^2))^2*((1+k^4)*n*pi/phi*coth(n*pi/phi*log(k))+2*(1-k^4)-2*k^2*n*pi/phi*csch(n*pi/phi*log(k)));
    end
end
Q = (1/16)*a^4*(k^4-1)*(phi-tan(phi)) + S;
%Verify integral numerically.
Q2 = trapz(Theta,trapz(R,u.*r,2));

%Divide by area.
A = (phi/2)*a^2*(k^2-1);
u_avg = Q/A;



