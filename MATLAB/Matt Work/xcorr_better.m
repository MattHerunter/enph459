function [R,lag]=xcorr_better(data1,data2)
    n=numel(data1);
    m=numel(data2);
    k=n+m-1;
    lag=nan(k,1);
    for ii=1:k
       arr1 = data1(max(1,n-ii+1):min(n,n+m-ii));
       arr2 = data2(max(1,1-n+ii):min(m,ii));
       R(ii)= dot(arr1,arr2);
       R(ii)=R(ii)/numel(arr1);
       lag(ii)=ii-n;
    end
end
