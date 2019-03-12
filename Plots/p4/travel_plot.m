figure(1)
hold off;

t = x_p4_no_feedback.time;
sv = x_p4_no_feedback.signals.values(:,1);
rv = x_star_p4_no_feedback.signals.values(:,1);

a = 1;
%b = [1/8 1/8 1/8 1/8 1/8 1/8 1/8 1/8];
b = [1/16 1/16 1/16 1/16 1/16 1/16 1/16 1/16 1/16 1/16 1/16 1/16 1/16 1/16 1/16 1/16];
y = filter(b,a,sv);

% plot(t,y,t,rv,'--' , 'LineWidth' , 1.2); %choose between filtered and
                                          %unfiltered data
plot(t,sv,t,rv,'--', 'LineWidth', 1.2);

legend('Travel', 'location', 'northeast') % Up rigth corner legends
handles(1) = xlabel('$time$ ($s$)'); % xLabel
handles(2) = ylabel('$angle$ ($rad$)'); %yLabel
set(handles, 'Interpreter' , 'Latex'); % Making them in latex
set(handles, 'Fontsize' , 20); % Fontsize
%set(get(gca,'ylabel'),'rotation',0) % % Rotates text on ylabel

set(gcf, 'PaperPositionMode', 'auto');
print -depsc2 p10p4_travel_no_feedback.eps %Sets the filename for export
%close;
