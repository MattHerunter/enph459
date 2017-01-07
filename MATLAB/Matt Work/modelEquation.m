function y = modelEquation(x, alpha, n)
    y=0*x;
    y(n:end)=x(1:(end-n+1));
    for jj=1:n
        z=diff(y,2);
        z(3:(end+2))=z;
        z(1:2)=0;
        y=y+alpha*z;
    end
end