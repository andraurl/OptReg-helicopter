%% Visualization of hill constranint
alpha = 0.2;
beta = 20;
PI = 3.14159265358;
lambda_t = 2*PI/3;
    
lambda = 0:0.01:3.2;
e = zeros(length(lambda),1);
i = 1;
for l = lambda
    e(i) = alpha*exp(-beta*(l-lambda_t)^2);
    i = i+1;
end
figure
plot(lambda,e);