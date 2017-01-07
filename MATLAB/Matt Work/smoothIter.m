function smthdata = smoothIter(data,n)
   smthdata=data;
   for ii=1:n
      smthdata=smooth(smthdata); 
   end
end