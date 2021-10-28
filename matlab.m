clc; clear;
double Y;
figure(1);
v=0;
xlabel('Tiempo');
ylabel('Velocidad RPM');

g = animatedline;
g.Color = 'red';  %EKG
ax = gca;
s=0;
x_axis_sec = datenum(seconds(8));
t_num = 0;

startTime = datetime('now');
t_start_num = datenum(startTime);  % Convert to num for faster calculation

bt = bluetooth("HC-06",1);

configureTerminator(bt,"CR/LF")
bt.Terminator

%write(bt,"7y","string")
%bt.NumBytesWritten
while 1
    write(bt,"S","string");
    bt.NumBytesWritten;

    bt.NumBytesAvailable;
    name = read(bt,3,"string")
    %split_temp = strsplit(name);   % Split incoming string
    %X = str2double(convertCharsToStrings(split_temp)); 
    Y = str2double(name);

    t_num = datenum(datetime('now')) - t_start_num;
    addpoints(g, t_num, Y);   % PPG infrared LED
    ax.XLim = [t_num - x_axis_sec, t_num];
    drawnow


end
