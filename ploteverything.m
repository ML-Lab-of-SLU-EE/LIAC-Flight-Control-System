function  ploteverything(BODY_Variables_RUNs,MAX_RUN,PARAMETRI_SIMULATI_SUCCESS,PARAMETRI_SIMULATI_SUCCESS_INDEX,PARAMETRI_SIMULATI_FAIL,PARAMETRI_SIMULATI_FAIL_INDEX)
close all;
ETICHETTA = {'V [m/sec]','\alpha [\circ]','\beta [\circ]','p [\circ/sec]','q[\circ/sec]','r[\circ/sec]','phi[\circ]','theta[\circ]','psi[\circ]','x [m]','y [m]','h [m]','de [\circ]','da [\circ]','dr [\circ]','dt [\circ]'};
success_index = find(PARAMETRI_SIMULATI_SUCCESS_INDEX);
fail_index = find(PARAMETRI_SIMULATI_FAIL_INDEX);
num_zero = length(success_index) ;
for i=1:length(success_index) 
%     time(:,i) = BODY_Variables_RUNs(index_zero(i)).Time;
    time(:,i) = linspace(0,30,30001);
    u(:,i) = BODY_Variables_RUNs(success_index(i)).Data(:,1);
    v(:,i) = BODY_Variables_RUNs(success_index(i)).Data(:,2);
    w(:,i) = BODY_Variables_RUNs(success_index(i)).Data(:,3);
    p(:,i) = BODY_Variables_RUNs(success_index(i)).Data(:,4);
    q(:,i) = BODY_Variables_RUNs(success_index(i)).Data(:,5);
    r(:,i) = BODY_Variables_RUNs(success_index(i)).Data(:,6);
    phi(:,i) = BODY_Variables_RUNs(success_index(i)).Data(:,7)*180/pi;
    theta(:,i) = BODY_Variables_RUNs(success_index(i)).Data(:,8)*180/pi;
    psi(:,i) = BODY_Variables_RUNs(success_index(i)).Data(:,9)*180/pi;
    x(:,i) = BODY_Variables_RUNs(success_index(i)).Data(:,10);
    y(:,i) = BODY_Variables_RUNs(success_index(i)).Data(:,11);
    h(:,i) = BODY_Variables_RUNs(success_index(i)).Data(:,12);
    V(:,i) = sqrt(u(:,i).^2+v(:,i).^2+w(:,i).^2);
    alpha(:,i) = atan(w(:,i)./V(:,i))*180/pi;
    beta(:,i) = asin(v(:,i)./V(:,i)).*180/pi;
end
    figure(2);
    for i=1:num_zero 
        plot(time(:,i),V(:,i));%1
        grid on,hold on;    
        xlabel('Time [sec]');
        ylabel(ETICHETTA(1));
    end
    
    figure(3);
    for i=1:num_zero 
        plot(time(:,i),alpha(:,i));%2
        grid on,hold on;    
        xlabel('Time [sec]');
        ylabel(ETICHETTA(2));
    end 
    figure(4);
    for i=1:num_zero 
    plot(time(:,i),beta(:,i));%3
        grid on,hold on;
    xlabel('Time [sec]');
    ylabel(ETICHETTA(3));
    end
    
    figure(5);
    for i=1:num_zero 
        plot(time(:,i),p(:,i));     %4
        grid on,hold on;
        xlabel('Time [sec]');
        ylabel(ETICHETTA(4));
    end
    
    figure(6);
    for i=1:num_zero 
        plot(time(:,i),q(:,i));     %5
        grid on,hold on;
        xlabel('Time [sec]');
        ylabel(ETICHETTA(5));
    end
    
    figure(7);
    for i=1:num_zero 
        plot(time(:,i),r(:,i));     %6
        grid on,hold on;
        xlabel('Time [sec]');
        ylabel(ETICHETTA(6));
    end
    
    figure(8);
    for i=1:num_zero 
        plot(time(:,i),phi(:,i)*180/pi/1000)      %7
        grid on,hold on;
        xlabel('Time [sec]');
        ylabel(ETICHETTA(7));
    end
    
    figure(9);
    for i=1:num_zero 
        plot(time(:,i),theta(:,i)*180/pi/1000);      %8
        grid on,hold on;
        xlabel('Time [sec]');
        ylabel(ETICHETTA(8));
    end
    
    figure(10);
    for i=1:num_zero 
        plot(time(:,i),psi(:,i)*180/pi/1000);      %9
        grid on,hold on;
        xlabel('Time [sec]');
        ylabel(ETICHETTA(9));
    end
    
    figure(11);
    for i=1:num_zero 
        plot(time(:,i),x(:,i));      %10
        grid on,hold on;
        xlabel('Time [sec]');
        ylabel(ETICHETTA(10));
    end
    
    figure(12);
    for i=1:num_zero 
        plot(time(:,i),y(:,i));      %11
        grid on,hold on;
        xlabel('Time [sec]');
        ylabel(ETICHETTA(11));
    end
    
    figure(13);
    for i=1:num_zero 
        plot(time(:,i),h(:,i));      %12
        grid on,hold on;
        xlabel('Time [sec]');
        ylabel(ETICHETTA(12));
    end
    figure(14);
    for i=1:num_zero 
        plot3(x(:,i),y(:,i),h(:,i));      %13
        grid on,hold on;
        xlabel(ETICHETTA(10));
        ylabel(ETICHETTA(11));
        zlabel(ETICHETTA(12))
    end
end
