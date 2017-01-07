% Physical model for temperature diffusion, T_t = alpha*T_xx
% y1 : Initial waveform
% alpha : diffusion coefficient
% n : number of timesteps
% y2 : Waveform after diffusion
function y2 = modelDiffusionEquation(y1, alpha, n)
    y2 = y1;
    dt=0.01;
    for ii = 1:n/dt
        % Gets the second derivative of y in x
        z = diff(y2,2);
        
        % Center and apply 0 gradient BC's
        z(2:(end+1)) = z;
        z(1) = z(2);
        z(end+1) = z(end);
        
        % Update y2
        y2 = y2 + alpha*z*dt;
    end
end