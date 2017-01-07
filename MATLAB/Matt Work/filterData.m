function filterdata=filterData(data)
   filterdata = data-mean(data);
   filterdata=filterdata/max(abs(filterdata));
   filterDatafft=fft(filterdata);
   plot(abs(filterDatafft));
end