function y = smin( a,  b,  k )


    r = exp(-a/k) + exp(-b/k);
    y = -k*log(r);
    
end
