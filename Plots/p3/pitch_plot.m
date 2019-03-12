figure(1)
hold off;

t = x_p3.time;
sv = x_p3.signals.values(:,3);
rv = pc_star_p3.signals.values;

a = 1;
%b = [1/8 1/8 1/8 1/8 1/8 1/8 1/8 1/8];
b = [1/16 1/16 1/16 1/16 1/16 1/16 1/16 1/16 1/16 1/16 1/16 1/16 1/16 1/16 1/16 1/16];
y = filter(b,a,sv);

% plot(t,y,t,rv,'--' , 'LineWidth' , 1.2); %choose between filtered and
                                          %unfiltered data
plot(t,sv,t,rv,'--', 'LineWidth', 1.2);

legend('Pitch', 'p_c', 'location', 'northeast') % Up rigth corner legends
handles(1) = xlabel('$time$ ($s$)'); % xLabel
handles(2) = ylabel('$angle$ ($rad$)'); %yLabel
set(handles, 'Interpreter' , 'Latex'); % Making them in latex
set(handles, 'Fontsize' , 20); % Fontsize
%set(get(gca,'ylabel'),'rotation',0) % % Rotates text on ylabel

set(gcf, 'PaperPositionMode', 'auto');
print -depsc2 p10p3_pitch.eps %Sets the filename for export
%close;




