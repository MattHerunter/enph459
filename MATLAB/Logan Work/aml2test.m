total = 2000;
window = 1500;

[fulla,fullb,fullc] = fake_data(total,25,5);

%figure
%hold on
%plot(fulla)
%plot(fullb)

for i = 1:total-window
    a = fulla(i:i+window);
    b = fullb(i:i+window);
    c = fullc(i:i+window);
    
    %Fs = 1000;
    %filterd = designfilt('lowpassfir','PassbandFrequency',15, ...
    %    'StopbandFrequency',30, ...
    %    'DesignMethod','kaiserwin','SampleRate',Fs);
    %a = filtfilt(filterd,a);
    %b = filtfilt(filterd,b);
    
    %Fs = 1000;
    %cutoff = 20;
    %[filterb, filtera] = butter(4,cutoff/(Fs/2));
    %a = filter(filterb,filtera,a);
    %b = filter(filterb,filtera,b);
    
    %a = wden(a,'sqtwolog','s','mln',4,'sym4');
    %b = wden(b,'sqtwolog','s','mln',4,'sym4');
    
    a = (diff(a));
    b = (diff(b));
    c = sign(diff(c));

    a=a-mean(a);
    a=a/std(a);
    b=b-mean(b);
    b=b/std(b);
    
    x(i) = finddelay(a,b);
    y(i) = finddelay(c,b)-finddelay(c,a);
end

%figure
%hold on
%plot(fulla)
%plot(fullb)
%plot(fullc)

figure
hold on
plot(x)
plot(y)
ylim([0 50])