% aml2: We modified the MAML technique by dividing each
% detail function into two segments. Different
% coefficients for each segment are computed.
%
% SYNTAX: y=aml2(xn,yn,delay)
%
% INPUT: xn = Received signal from first receiver
% yn = Received signal from second receiver
% delay = True TDOA between xn and yn
% OUTPUT: y = Error between true TDOA and estimated TDOA
%
% SUB_FUNC: None
% Written by Unal Aktas

function y = aml2(xn,yn,delay)

    xyn = xcorr(xn,yn,'biased');
    [sigmas b] = max(xyn);
    rx = xcorr(xn,'biased');
    maxx = rx(length(xn));
    ry = xcorr(yn,'biased');
    maxy = ry(length(yn));
    sigman1 = maxx-sigmas;
    sigman2 = maxy-sigmas;
    
    nx = floor(log2(length(xn)));
    ny = floor(log2(length(yn)));
    
    [cx,lx] = wavedec(xn,nx,'db4');
    [cy,ly] = wavedec(yn,ny,'db4');
    
    dxc = [];
    for i = 1:nx
        d = detcoef(cx,lx,i);
        dl = length(d);
        Ns1 = 128/2^(i-1);     % THE LENGTH OF THE SUBBLOCK
        if Ns1 <= 1
            Ns = dl;
        else
            Ns = Ns1;
        end
        
        D = ceil(dl/Ns);
        if dl < Ns*D
            dm = [d zeros(1,D*Ns-dl)];
        end
        for k = 1:D
            p = (k-1)*Ns+1:k*Ns;
            sigmad = (1/Ns)*sum(dm(p).^2);
            sigmasd = sigmad - sigman1;
            if sigmasd <= 0
                wd = 0;
            else
                wd = sigmasd/(sigman1*sigman2+sigmasd*(sigman1+sigman2));
            end
            dc(p) = wd*dm(p);
        end
        dxc = [dc(1:dl) dxc];
    end
    
    a = appcoef(cx,lx,'db4',nx);
    al = length(a);
    sigmaa = (1/al)*sum(a.^2);
    sigmasa = sigmaa - sigman1;
    if sigmasa <= 0
        wa = 0;
    else
        wa = sigmasa/(sigman1*sigman2+sigmasa*(sigman1+sigman2));
    end
    ac = wa*a;
    dxc = [ac dxc];
    
    xd = waverec(dxc,lx,'db4');
    
    dyc = [];
    for i = 1:ny
        dy = detcoef(cy,ly,i);
        dyl = length(dy);
        Ns1 = 128/2^(i-1);
        if Ns1 <= 1
            Ns = dyl;
        else
            Ns = Ns1;
        end
        
        D = ceil(dyl/Ns);
        if dyl < Ns*D
            dmy = [dy zeros(1,D*Ns)];
        end
        for k = 1:D
            p = (k-1)*Ns+1:k*Ns;
            sigmady = (1/Ns)*sum(dmy(p).^2);
            sigmasdy = sigmady - sigman1;
            if sigmasdy <= 0
                wdy = 0;
            else
                wdy = sigmasdy/(sigman1*sigman2+sigmasdy*(sigman1+sigman2));
            end
            dcy(p) = wdy*dmy(p);
        end
        
        dyc = [dcy(1:dyl) dyc];
    end
    
    ay = appcoef(cy,ly,'db4',ny);
    ayl = length(ay);
    sigmaay = (1/ayl)*sum(ay.^2);
    sigmasay = sigmaay-sigman1;
    if sigmasay <= 0
        way = 0;
    else
        way = sigmasay/(sigman1*sigman2+sigmasay*(sigman1+sigman2));
    end
    acy = way*ay;
    dyc = [acy dyc];
    
    yd = waverec(dyc,ly,'db4');
    rxyd = xcorr(xd,yd);
    
    %figure
    %hold on
    %plot(rxyd);
    
    [a5,b5] = max(rxyd);
    y = numel(xn) - b5;
    %er5 = delay - (b5-1024);
    %y = er5;
    