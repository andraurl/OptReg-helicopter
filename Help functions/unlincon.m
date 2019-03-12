function [c, c_eq] = unlincon(z)
%     global N mx nx alpha beta lambda_t
    N = 40;
    mx = 6;
    nx = N*mx;
    alpha = 0.2;
    beta = 20;
    lambda_t = 2.0944;
     c = zeros(N,1); 
%      for n = 1:N-1
%          c(n) = alpha*exp(-beta*(z(1+(n-1)*mx)-lambda_t)^2) - z(5:mx:nx);
%      end
%      length(c)
    c(1:N,1) = alpha*exp(-beta*(z(1:mx:nx)-lambda_t).^2) - z(5:mx:nx);
    
    c_eq = [];
end