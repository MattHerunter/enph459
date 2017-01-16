function [shift, alpha] = fitFlowAndDiffusion(y1,y2,varargin)
    % y1 is the signal at the first sensor, y2 should lag y1 and have more
    % diffusion applied. alpha is the diffusion coefficient such that 
    % T_t = alpha*T_xx
    if(nargin == 4)
        shiftStartingPoint = varargin(1);
        alphaStartingPoint = varargin(2);
    else
        shiftStartingPoint = 0;
        alphaStartingPoint = 0;
    end
    shiftGradient = 0;
    alphaGradient = 0;
    fitting = true;
    while(fitting)
        
    end
end

function yShift = shiftArray(y,n)
   if(n > 0)
      yShift(1:n) = y(1);
      yShift((n+1):end) = y(1:(end-n-1));
   elseif(n < 0)
       
   else
       yShift = y;
   end
end
