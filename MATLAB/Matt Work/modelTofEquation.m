% Should add problem parameter dt
function y = modelTofEquation(x,dt)
    y = x;
    idx_shift = floor(dt);
    interp_shift = dt - idx_shift;
    
    % Shift indices
    y(idx_shift:end) = x(1:(end-idx_shift+1));
    y(1:(idx_shift-1)) = y(idx_shift);
    
    % Interp for shifts smaller than one index
end