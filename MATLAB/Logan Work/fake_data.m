function [data1, data2, data3] = fake_data(len, delay, noise)
    data = cumsum(randi([-1,1],len+2*delay,1).*randi([-1,1],len+2*delay,1).*randi([-1,1],len+2*delay,1));
    data1 = data(2*delay+1:2*delay+len)+randi([-noise,noise],len,1);
    data2 = data(delay+1:delay+len)+randi([-noise,noise],len,1);
    data3 = data(1:len);
    data1 = data1';
    data2 = data2';
    data3 = data3';