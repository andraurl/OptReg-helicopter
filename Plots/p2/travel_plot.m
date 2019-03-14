figure(1)
hold off;

t = x_p2.time;
sv = x_p2.signals.values(:,1);

t2 = 0:35/length(x1):35-35/length(x1);
rv = x1;

a = 1;
%b = [1/8 1/8 1/8 1/8 1/8 1/8 1/8 1/8];
b = [1/16 1/16 1/16 1/16 1/16 1/16 1/16 1/16 1/16 1/16 1/16 1/16 1/16 1/16 1/16 1/16];
y = filter(b,a,sv);

% plot(t,y,t,rv,'--' , 'LineWidth' , 1.2); %choose between filtered and
                                          %unfiltered data
plot(t,sv,t2,rv,'--', 'LineWidth', 1.2);

legend('Travel', 'Optimal path', 'location', 'northeast') % Up rigth corner legends
handles(1) = xlabel('$time$ ($s$)'); % xLabel
handles(2) = ylabel('$angle$ ($rad$)'); %yLabel
set(handles, 'Interpreter' , 'Latex'); % Making them in latex
set(handles, 'Fontsize' , 20); % Fontsize
%set(get(gca,'ylabel'),'rotation',0) % % Rotates text on ylabel

set(gcf, 'PaperPositionMode', 'auto');
print -depsc2 p10p2_10_travel.eps %Sets the filename for export
%close;

