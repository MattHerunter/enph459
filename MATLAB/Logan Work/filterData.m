data = importData('C:/Users/Logan Numerow/Desktop/ENPH459/git/Python/enph459daq/Data/2017-01-18/Set3');

test = data{1,5};

Fs = 500;
filt1 = test;
for i=2:numel(filt1)
    filt1(i) = 0.9*filt1(i-1)+0.1*filt1(i);
end

d = designfilt('lowpassfir','PassbandFrequency',20, ...
    'StopbandFrequency',21, ...
    'DesignMethod','kaiserwin','SampleRate',Fs);
filt2 = filtfilt(d,test);


plot(test);
hold on;
plot(filt1);
plot(filt2);