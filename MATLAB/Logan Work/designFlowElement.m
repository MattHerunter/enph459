function designFlowElement(R,N,t)    
    eps = R*1.e-5;

    f = @(x) flowElementGuess(R,N,t,x)-R;
    f_x = @(x) (f(x+eps)-f(x))/eps;
    
    guess(1) = t;
    R_guess(1) = flowElementGuess(R,N,t,guess(1));
    
    for i = 1:10
        delta = f(guess(i))/f_x(guess(i));
        guess(i+1) = guess(i) - delta;
        if abs(delta) < R*1.e-5
            break 
        end
    end
    
    real(guess)
    A = guess(end);
    b_vals = flowElement(R,N,t,guess(end));

%     guess = 0.0255:0.0001:0.0265;
%     for i = 1:numel(guess)
%         guessr(i) = flowElementGuess(R,N,t,guess(i));
%     end
%     
%     figure
%     scatter(guess,guessr)
    
%     A = 0.02602;
%     b_vals = flowElement(R,N,t,A);
   
    x = linspace(0,b_vals(1),100);
    y = getCircleFlow(b_vals(1))*ones(1,100);
    x = [x linspace(b_vals(1),b_vals(1)+t,100)];
    y = [y zeros(1,100)];
    for i = 2:N
        x = [x linspace(b_vals(i-1)+t,b_vals(i),100)];
        y = [y getAnnularFlow(b_vals(i-1)+t,b_vals(i),pi)*ones(1,100)];
        x = [x linspace(b_vals(i),b_vals(i)+t,100)];
        y = [y zeros(1,100)];
    end
    
    plot(x,y./max(y));
    hold on
    plot(x,1-(x./R).^2);
    ylim([0,1]);
end

function R_guess = flowElementGuess(R,N,t,A)
    scale = getCircleFlow(A)/idealCircleFlow(R,A,t);
    
    b_vals(1) = A;
    for n = 2:N
        a = b_vals(n-1)+t;
        
        eps = R*1.e-6;
        f = @(b)real(getAnnularFlow(a,b,pi)-scale*idealAnnularFlow(R,a,b,t));
        f_x = @(b) (f(b+eps)-f(b))/eps;
        b_vals(n) = R-t;
        for i = 1:20
            delta = f(b_vals(n))/f_x(b_vals(n));
            b_vals(n) = b_vals(n) - delta;
            if abs(delta) < R*1.e-6
               break 
            end
        end
            
        if b_vals(n) >= R-t
           break
        end
    end
    
    if n == N
        R_guess = b_vals(N)+t;
    else
        R_guess = b_vals(n)+t;
    end
end

function b_vals = flowElement(R,N,t,A)
    scale = getCircleFlow(A)/idealCircleFlow(R,A,t);
    
    b_vals(1) = A;
    for n = 2:N
        a = b_vals(n-1)+t;
        
        eps = R*1.e-6;
        f = @(b)real(getAnnularFlow(a,b,pi)-scale*idealAnnularFlow(R,a,b,t));
        f_x = @(b) (f(b+eps)-f(b))/eps;
        b_vals(n) = R-t;
        for i = 1:20
            delta = f(b_vals(n))/f_x(b_vals(n));
            b_vals(n) = b_vals(n) - delta;
            if abs(delta) < 1.e-10
               break 
            end
        end
        
        if b_vals(n) >= R-t
           break
        end
    end
    
    if n == N
        R_guess = b_vals(N)+t;
    else
        R_guess = b_vals(n)+t;
    end
end

function u_avg = idealAnnularFlow(R,a,b,t)
    u_avg = 2/((b+t/2)^2-(a-t/2)^2)*(((b+t/2)^2-(a-t/2)^2)/2 - ((b+t/2)^4-(a-t/2)^4)/(4*R^2));
end

function u_avg = idealCircleFlow(R,A,t)
    u_avg = 2/(A+t/2)^2*((A+t/2)^2/2-(A+t/2)^4/(4*R^2));
end

function y = annularFilterModelEquation(A,r)
    phi = pi;
    t = 1;
    y = zeros(size(r));
    for ii = 1:numel(A)
        a = A(ii);
        if ii ~= numel(A)
            b = A(ii+1)-t;
        else
            b = r(end) - t;
        end
        [~,a_idx] = min(abs(r-a));
        [~,b_idx] = min(abs(r-b));
        u = getAnnularFlow(a, b, phi);
        y(a_idx:b_idx) = -u;
    end
    y = y/max(y);
end
    