function designFDFElement()
    % Number of channels
    n = 5;
    
    % Minimum material thickness in mm
    t = 1;
    
    % Minimum cut width in mm
    w = 0.25;
    
    % Inner radius of pipe in mm
    R = 2.5*25.4;
    
    resolution = 500;
    r = linspace(0,R,resolution)';
    r_n = r/R;
    u_n = 1 - r_n.^2;
    
    if n*(t + w) > R
        warning('Too many channels, impossible to fit into given pipe radius.');
        return
    end
    
    A = [2, linspace(r(end)/n,r(end)*(1-1/n),n-1)];
    y = annularFilterModelEquation(A,r);
    plot(r,u_n,r,y);
    pause
    A_f = lsqcurvefit(@annularFilterModelEquation,A,r,u_n);
    plot(r,u_n,r,annularFilterModelEquation(A_f,r));
    
end