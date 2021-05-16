figure(1);
    plot(time/2,BODY_Variables.Data(:,7)*180/pi)      %7
        grid on,hold on;
    xlabel('Time [sec]');
    ylabel(ETICHETTA(7));
hold on
plot(time/2,zeros(length(time),1),'k--');
figure(2);plot(time/2,BODY_Variables.Data(:,8)*180/pi-20)      %8
    grid on,hold on;
    xlabel('Time [sec]');
    ylabel(ETICHETTA(8));
hold on
plot(time/2,zeros(length(time),1),'k--');
figure(3)
    plot(time/2,BODY_Variables.Data(:,9)*180/pi+1)      %9
        grid on,hold on;
    xlabel('Time [sec]');
    ylabel(ETICHETTA(9));
hold on
plot(time/2,(-0)*ones(length(time),1),'k--');