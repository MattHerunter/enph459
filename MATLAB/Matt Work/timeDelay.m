function td=timeDelay(data)
    Fs = 2000;
    d = designfilt('lowpassfir','PassbandFrequency',100, ...
        'StopbandFrequency',101, ...
        'DesignMethod','kaiserwin','SampleRate',Fs);
    for jj=1:length(data)
        x1=data{jj,4};
        x2=data{jj,5};
        x1=x1-mean(x1);
        x2=x2-mean(x2);
        x1=x1/max(x1);
        x2=x2/max(x2);
        x1=filtfilt(d,x1);
        x2=filtfilt(d,x2);
        td(jj)=finddelay(x1,x2);
        if(td(jj)<1)
            t=1:numel(x1);
            plot(t,x1,t,x2);
        end
    end
end