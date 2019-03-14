%% Visualization of hill constranint
alpha = 0.2;
beta = 20;
PI = 3.14159265358;
lambda_t = 2*PI/3;
N = 40;

lambda = 0:pi/1000:pi;
e = zeros(length(lambda),1);
i = 1;
for l = lambda
    e(i) = alpha*exp(-beta*(l-lambda_t)^2);
    i = i+1;
end

figure(1)
plot(lambda,e, 'LineWidth', 1.2)
legend('Real hill', 'Optimal path', 'location', 'northeast')
handles(1) = xlabel('$angle$ ($rad$)'); % xLabel
handles(2) = ylabel('$angle$ ($rad$)'); %yLabel
set(handles, 'Interpreter' , 'Latex'); % Making them in latex
set(handles, 'Fontsize' , 20); % Fontsize
set(gcf, 'PaperPositionMode', 'auto');
print -depsc2 real_hill.eps %Sets the filename for export

figure(2)

sv = alpha*exp(-beta*(z(1:mx:nx)-lambda_t).^2);
rv = e;

plot(sv, 'LineWidth', 1.2);
legend('Hill constraint', 'Optimal path', 'location', 'northeast')
handles(1) = xlabel('$time$ ($s$)'); % xLabel
handles(2) = ylabel('$angle$ ($rad$)'); %yLabel
set(handles, 'Interpreter' , 'Latex'); % Making them in latex
set(handles, 'Fontsize' , 20); % Fontsize

set(gcf, 'PaperPositionMode', 'auto');
print -depsc2 hill_constraint.eps %Sets the filename for export
%close;
