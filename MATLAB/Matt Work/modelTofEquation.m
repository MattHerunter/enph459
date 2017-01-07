% Should add problem parameter dt
function y = modelTofEquation(x,dt)
    y = x;
    y(n:end) = x(1:(end-n+1));
    y(1:(n-1)) = y(n);
end