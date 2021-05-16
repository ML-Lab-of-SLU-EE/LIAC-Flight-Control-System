clear; close all;clc;
restoredefaultpath
warning off all;
addpath( genpath( pwd ))                    % Add current Folder to the path, otherwise would not find some function and files
MONTECARLO_EXE = 1;                         % Only simulate the Nominal parameter set if MONTECARLO_EXE = 0, otherwise execute the MONTECARLO simulations
OUTPUT_DISTURBANCE = 0;                     % Add sinusoidal to input/output
INPUT_DISTURBANCE =  0;                     % Add sinusoidal to input/output 3deg ==> 0.5
VARIANCE = diag( [ones( 12 , 1 )] )*1e-9;   % if you want to test it againt noisy measurements increase it adequately 1e-8 is low enough to be No Noise Case
MAX_RUN =500;                               % How many runs do you want to perform
UncertantiesParameters = 25/100;            % Parameter variation in percentage for all the dynamic parameters except  gravity!!!
InputCrossCoupling =  10/100;               % CrossCoupling in the inputs, usually very small, it can be for example Thrust effect on the yaw/roll due to propeller torque effect 5% is an extreme value
ThrustReduction = 20/100;                   % Max Thrust Reduction in percentage
CmAlphaIncert = 0.8;                        % If == 0 takes the uncertanties of the parameters, otherwise custom one defined
montecarlo_n = 0.01;                          % 随机布设范围
Ts = 0.01;                                  % Controller frequency 100Hz=0.01s, Simulation step is Ts/10 (CTRL+E in Simulink file)
% Set the simulation time Programmatically
model_name = dir('*.slx');
model_name = model_name.name ; model_name = strrep( model_name , '.slx' , '' );
open(model_name)
set_param( model_name , 'FixedStep' , 'Ts/10');
pause( .1 )
blockName = [ model_name , '/Reference Generator Variation/Step' ];
% DeltaSpeed = 10;
DeltaSpeed =  1;
V_trim = 4;
% Set the delta Speed Programmatically
set_param( blockName , 'after' , num2str( DeltaSpeed ) );
pause(1)
save_system
close_system
tic
INIT % set parameters of the A/C: Navion
% Define some condition where you want the a/c to be trimmed at (V_trim)
alpha_trim = 0;
beta_trim = 0;
TRIM % trim the aircraft
%% Define the control laws: Lateral and Longitudinal separately
POLE_PLACEMENT % define the nominal closed loop by pole  placement

TEST_NAVION_LONG  % AC LONGITUDINAL L1 AC design
TEST_NAVION_LAT   % AC LATERAL L1 AC design

%% Setup some initial condition X0 and target conditions
x0 = stato_trim_body;   %% Initial condition = Trim Conditions
x0(12) = 10;            %% initial Altitude
x0(11) = 0;             %% initial East Component (position)
x0(9)=-0.0*pi/180;      %% Initial Psi

target_point = stato_trim_body;     %% where we want to trim the A/C (can be different than x0)
target_point(12) = 10;               %% altitude we want to reach
% target_point(9) = 5*pi/180;       %% Psi Target: target coincides with psi of the reference path
target_point(8) = 0*pi/180; 
target_point(7) = 0*pi/180; 
target_point(9) = -15*pi/180; 

try
    target_point(1) = target_speed; % If it is defined elsewhere , inside MONTECARLO.m  use that one otherwise go on with default/ as initial/trim condition to reach (see the simulink model)
end

%%

Ts_delay = 1*Ts; %Ts = 0.01 Nominally .It used in simulation in ...


%% NOMINAL CASE

omega_true = eye(4);        % can add for example mismatches representing crosscoupling +0.05.*rand(4,4)
PARAMETERS_AC(2) = rho*7/8; %  nominal density 7/8 1.225
PARAMETERS_AC(2) = rho*8/8; %  test 1st may
PARAMETERS_AC_old = PARAMETERS_AC;
PARAMETERS_AC_init = PARAMETERS_AC;
BODY_Variables.Time = 0;
[t,x1,y1] = sim(model_name); %Name of the file that run the simulation

    time = BODY_Variables.Time;
    u = BODY_Variables.Data(:,1);
    v = BODY_Variables.Data(:,2);
    w = BODY_Variables.Data(:,3);
    p = BODY_Variables.Data(:,4);
    q = BODY_Variables.Data(:,5);
    r = BODY_Variables.Data(:,6);
    phi = BODY_Variables.Data(:,7)*180/pi;
    theta = BODY_Variables.Data(:,8)*180/pi;
    psi = BODY_Variables.Data(:,9)*180/pi;
    x = BODY_Variables.Data(:,10);
    y = BODY_Variables.Data(:,11);
    h = BODY_Variables.Data(:,12);

    de = ACTUATOR_TRUE.Data(:,1)*180/pi;
    da = ACTUATOR_TRUE.Data(:,2)*180/pi;
    dr = ACTUATOR_TRUE.Data(:,3)*180/pi;
    dt = ACTUATOR_TRUE.Data(:,4);

    %% Data are saved in body coordinates
    V = sqrt(u.^2+v.^2+w.^2);
    alpha = atan(w./V)*180/pi;
    beta = asin(v./V)*180/pi;
    %% plot everything
    ETICHETTA = {'V [m/sec]','\alpha [\circ]','\beta [\circ]','p [\circ/sec]','q[\circ/sec]','r[\circ/sec]','phi[\circ]','theta[\circ]','psi[\circ]','x [m]','y [m]','h [m]','de [\circ]','da [\circ]','dr [\circ]','dt [\circ]'};
    figure;
    plot(time,BODY_Variables.Data(:,1));%1
        grid on,hold on;
    xlabel('Time [sec]');
    ylabel(ETICHETTA(1));
    figure;
    plot(time,alpha);%2
        grid on,hold on;
    xlabel('Time [sec]');
    ylabel(ETICHETTA(2));
    figure;
    plot(time,beta);%3
        grid on,hold on;
    xlabel('Time [sec]');
    ylabel(ETICHETTA(3));
    figure;
    plot(time,BODY_Variables.Data(:,4));     %4
        grid on,hold on;
    xlabel('Time [sec]');
    ylabel(ETICHETTA(4));
    figure;
    plot(time,BODY_Variables.Data(:,5));     %5
        grid on,hold on;
    xlabel('Time [sec]');
    ylabel(ETICHETTA(5));
    figure;
    plot(time,BODY_Variables.Data(:,6));     %6
        grid on,hold on;
    xlabel('Time [sec]');
    ylabel(ETICHETTA(6));
    figure;
    plot(time,BODY_Variables.Data(:,7)*180/pi)      %7
        grid on,hold on;
    xlabel('Time [sec]');
    ylabel(ETICHETTA(7));
    figure;
    plot(time,BODY_Variables.Data(:,8)*180/pi-20)      %8
    grid on,hold on;
    xlabel('Time [sec]');
    ylabel(ETICHETTA(8));
    figure;
    plot(time,BODY_Variables.Data(:,9)*180/pi)      %9
        grid on,hold on;
    xlabel('Time [sec]');
    ylabel(ETICHETTA(9));
    figure;
    plot(time,BODY_Variables.Data(:,10))      %10
        grid on,hold on;
    xlabel('Time [sec]');
    ylabel(ETICHETTA(10));
    figure;
    plot(time,BODY_Variables.Data(:,11))      %11
        grid on,hold on;
    xlabel('Time [sec]');
    ylabel(ETICHETTA(11));
    figure;
    plot(time,BODY_Variables.Data(:,12))      %12
        grid on,hold on;
    xlabel('Time [sec]');
    ylabel(ETICHETTA(12));
toc
try
    if MONTECARLO_EXE==1
        ACTUATOR_NOMINAL = ACTUATOR_TRUE;               % save nominal input history [de da dr dt]
        BODY_Variables_NOMINAL =  BODY_Variables;       % save nominal output history [u v w p q r phi theta psi x y z]
        pause(.1)
        warning off;                                     % stop annoying warning error messages
        PARAMETERS_AC_old = PARAMETERS_AC;              % old aircraft parameter, NOMINAL stored an on top add the parameter uncertanties
        omega_true = eye(4);                              % omega_NOMINAL
        kk = 0;                                           % index of the failed parameter
        gg = 0;
        PARAMETRI_SIMULATI = zeros(33 , MAX_RUN);       % Preallocate the parameter
        PARAMETRI_SIMULATI_FAIL_INDEX = zeros(MAX_RUN , 1);
        PARAMETRI_SIMULATI_SUCCESS_INDEX = zeros(MAX_RUN , 1);
        % initialize the RUNs where the variable are stored
        ACTUATOR_RUNs( MAX_RUN ) = ACTUATOR_TRUE;
        BODY_Variables_RUNs( MAX_RUN ) =  BODY_Variables;
        cc1 = waitbar( 0 / (MAX_RUN), ' simulation running ' ); % show a progressingbar saying the simulation is running
        target_point_AC_int = stato_trim_body;
        for i = 1:MAX_RUN
%             PARAMETERS_AC_init_7(i) = random('Normal',PARAMETERS_AC_init(7),montecarlo_n,1);
%               PARAMETERS_AC_init_8(i) = random('Normal',PARAMETERS_AC_init(8),montecarlo_n,1);
%             PARAMETERS_AC_init_9(i) = random('Normal',PARAMETERS_AC_init(9),montecarlo_n,1);
%             PARAMETERS_AC_init_17(i) = random('Normal',PARAMETERS_AC_init(17),montecarlo_n,1);
%             PARAMETERS_AC_init_18(i) = random('Normal',PARAMETERS_AC_init(18),montecarlo_n,1);
%             PARAMETERS_AC_init_19(i) = random('Normal',PARAMETERS_AC_init(19),montecarlo_n,1);
%             PARAMETERS_AC_init_20(i) = random('Normal',PARAMETERS_AC_init(20),montecarlo_n,1);
%             PARAMETERS_AC_init_21(i) = random('Normal',PARAMETERS_AC_init(21),montecarlo_n,1);
%               PARAMETERS_AC_init_23(i) = random('Normal',PARAMETERS_AC_init(23),montecarlo_n,1);
%             PARAMETERS_AC_init_24(i) = random('Normal',PARAMETERS_AC_init(24),montecarlo_n,1);
%             PARAMETERS_AC_init_26(i) = random('Normal',PARAMETERS_AC_init(26),montecarlo_n,1);
%             PARAMETERS_AC_init_27(i) = random('Normal',PARAMETERS_AC_init(27),montecarlo_n,1);
%             PARAMETERS_AC_init_28(i) = random('Normal',PARAMETERS_AC_init(28),montecarlo_n,1);
%             PARAMETERS_AC_init_29(i) = random('Normal',PARAMETERS_AC_init(29),montecarlo_n,1);
              target_point_AC_int_7(i)= random('Normal',target_point_AC_int(7),1.5,1);
              target_point_AC_int_8(i)= random('Normal',target_point_AC_int(8),1.5,1);
        end
        
        for ii = 1 : MAX_RUN % Run <MAX_RUN> simulation with different PARAMETERS_AC +-30% of the nominal , Cmalpha can be even unstable!! and it recovers
            omega_true = eye(4);
            % uncertainties in the Parameter describing the aircraft dynamic
%             PARAMETERS_AC_MONTECARLO = PARAMETERS_AC_old + UncertantiesParameters * PARAMETERS_AC_old .*rand(size(PARAMETERS_AC)).*sign(randn(size(PARAMETERS_AC)));
            PARAMETERS_AC_MONTECARLO = PARAMETERS_AC_old;
%             PARAMETERS_AC_MONTECARLO(7) = PARAMETERS_AC_init_7(ii); 
%             PARAMETERS_AC_MONTECARLO(8) = PARAMETERS_AC_init_8(ii) ;
%             PARAMETERS_AC_MONTECARLO(9) = PARAMETERS_AC_init_9(ii) ;
%             PARAMETERS_AC_MONTECARLO(17) = PARAMETERS_AC_init_17(ii) ;
%             PARAMETERS_AC_MONTECARLO(18) = PARAMETERS_AC_init_18(ii);
%             PARAMETERS_AC_MONTECARLO(19) = PARAMETERS_AC_init_19(ii);
%             PARAMETERS_AC_MONTECARLO(20) = PARAMETERS_AC_init_20(ii);
%             PARAMETERS_AC_MONTECARLO(21) = PARAMETERS_AC_init_21(ii) ;
%               PARAMETERS_AC_MONTECARLO(23) = PARAMETERS_AC_init_23(ii);
%             PARAMETERS_AC_MONTECARLO(24) = PARAMETERS_AC_init_24(ii) ;
%             PARAMETERS_AC_MONTECARLO(26) = PARAMETERS_AC_init_26(ii) ;
%             PARAMETERS_AC_MONTECARLO(27) = PARAMETERS_AC_init_27(ii) ;
%             PARAMETERS_AC_MONTECARLO(28) = PARAMETERS_AC_init_28(ii) ;
%             PARAMETERS_AC_MONTECARLO(29) = PARAMETERS_AC_init_29(ii) ;
            
%     CrossCoupling is added to de da dr --------dt influence not considered
            Omega_inputCrossCoupling = zeros(size(omega_true));
            Omega_inputCrossCoupling(1:end-1,1:end-1) = randn(3,3) * InputCrossCoupling;
%             Omega_inputCrossCoupling(1:end-1,end) =  InputCrossCoupling/10 * randn(size(Omega_inputCrossCoupling(1:end-1,end)));
%             Omega_inputCrossCoupling(end,1:end-1) =  InputCrossCoupling/10 * randn(size(Omega_inputCrossCoupling(end,1:end-1)));
%             Omega_inputCrossCoupling(end,end) = randn(1,1) * InputCrossCoupling;
            % OMEGA_MONTECARLO = omega_true+randn(4,4)*InputCrossCoupling;              % add cross coupling up to  <<InputCrossCoupling % >> per control channel
            OMEGA_MONTECARLO = omega_true+Omega_inputCrossCoupling; % Add cross coupling to omega_true
            % correct/override the thrust to be at minimum equal to =
            % = T_W_ratio*(1-ThrustReduction)~ 70% MAX THRUST
            mass = random('Normal',PARAMETERS_AC_MONTECARLO(30),montecarlo_n,1,1);
            Tmax = mass * g * T_W_ratio;  % TMAX  T_W_ratio of the mass
%            PARAMETERS_AC_MONTECARLO(15) = Tmax * (1 - ThrustReduction * abs(rand(1)));
           target_point(7)=target_point_AC_int_7(ii);
           target_point(8)=target_point_AC_int_8(ii);
            SETUP_MONTECARLO;   % RUN the simulation simulation with the new data/parameter
% ~~~~~~~~~~~~Store all the actuator and body history~~~~~~~~~~~~~~~~~%
            ACTUATOR_RUNs(ii) = ACTUATOR_TRUE;
            BODY_Variables_RUNs(ii) =  BODY_Variables;
            PARAMETRI_SIMULATI(:,ii)=PARAMETERS_AC_MONTECARLO ; % store all the parameters
            ii
            if max(BODY_Variables.Time)<29                                                % if the simulation aborted store the parameter that triggered the singularities
                kk=kk+1                                                     % add the counter
                PARAMETRI_SIMULATI_FAIL(:,kk) = PARAMETERS_AC_MONTECARLO;   % store the failure one separately
                PARAMETRI_SIMULATI_FAIL_INDEX(ii) = 1;
                target_point_8_FAIL(kk)=target_point(8);
                target_point_7_FAIL(kk)=target_point(7);
            else
                gg=gg+1;
                PARAMETRI_SIMULATI_SUCCESS(:,gg) = PARAMETERS_AC_MONTECARLO; 
                PARAMETRI_SIMULATI_SUCCESS_INDEX(ii) = 1;
                target_point_8_SUCCESS(gg)=target_point(8);
                target_point_7_SUCCESS(gg)=target_point(7);
            end
            waitbar(ii/MAX_RUN,cc1);                                        % update the waitbar
        end        
        close(cc1);                                                         % close the waitbar

%%%%~~~~~~~~~~~~~~  Part 4 : Save the simulations data ~~~~~~~~~~~~~~~~~~~~~~~%%%%%

        BODY_Variables_RUNs(MAX_RUN+1)=BODY_Variables_NOMINAL; % store at the end the body variable NOMINAL
        ACTUATOR_RUNs(MAX_RUN+1) = ACTUATOR_NOMINAL;
        PARAMETRI_SIMULATI(:,MAX_RUN+1) = PARAMETERS_AC_old;
        %% save all the time hystory in v7.3 because otherwise could crash!!! Do not know why
        NOME = datestr( now ) ;
        NOME = strrep( NOME , ':' , '_' );
%         save( NOME ,'ThrustReduction','InputCrossCoupling','UncertantiesParameters','BODY_Variables_RUNs','ACTUATOR_RUNs','PARAMETRI_SIMULATI','Reference_Altitude','CmAlphaIncert','-v7.3');
        %% Clear and Re Load the latest file


%% %%~~~~~~~~~~~~~~  Part 5 : Plot all simulations data and parameters ~~~~~~~~~~~~~~~~~~~~~~~%%%%%

        % Label of all parameters
        LABEL_NEXT = {'g', 'rho', 'c', 'b', 'S', 'CL_0', 'CL_{alpha}', 'CL_{eq}', 'CL_{de}', 'CD_0', 'AR', 'e', 'CY_{beta}', 'CY_{dr}', ...
            'T_{max}', 'Cl_{beta}', 'Cl_p', 'Cl_r', 'Cl_{da}', 'Cl_{dr}', 'Cm_0', 'Cm_{alpha}', 'Cm_{eq}', 'Cm_{de}' ...
            ,'Cn_{beta}',  'Cn_p', 'Cn_r' ,'Cn_{da}' ,'Cn_{dr}' ,'m', 'Ixx', 'Iyy', 'Izz' };

        MAX_RUN = max( size ( BODY_Variables_RUNs )) - 1; % BODY_Variables_RUNs is a time series of 1x(MAX_RUN -1) variable run



        % Plot all the parameters !!! No need for link them all
        MAX_PARAM = 33; % Total number of parameters


    end
end
%% Print back the value used for the simulation
prompt1='是否画收敛状态的图:\n 1:画图 \n 2:不画图仅储存值\n 3: 蒙特卡洛参数图\n';
a=input(prompt1);
if a==1
    ploteverything(BODY_Variables_RUNs,MAX_RUN,PARAMETRI_SIMULATI_SUCCESS,PARAMETRI_SIMULATI_SUCCESS_INDEX,PARAMETRI_SIMULATI_FAIL,PARAMETRI_SIMULATI_FAIL_INDEX);
    save( NOME ,'BODY_Variables_RUNs','ACTUATOR_RUNs','PARAMETRI_SIMULATI','PARAMETRI_SIMULATI_FAIL','PARAMETRI_SIMULATI_FAIL_INDEX','PARAMETRI_SIMULATI_SUCCESS','PARAMETRI_SIMULATI_SUCCESS_INDEX','Reference_Altitude','CmAlphaIncert','-v7.3');
end
if a==2
    save( NOME ,'BODY_Variables_RUNs','ACTUATOR_RUNs','PARAMETRI_SIMULATI','PARAMETRI_SIMULATI_FAIL','PARAMETRI_SIMULATI_FAIL_INDEX','PARAMETRI_SIMULATI_SUCCESS','PARAMETRI_SIMULATI_SUCCESS_INDEX','Reference_Altitude','CmAlphaIncert','-v7.3');
end
if a==3
    ploteveryMONTECARLO(PARAMETRI_SIMULATI,PARAMETRI_SIMULATI_SUCCESS,PARAMETRI_SIMULATI_FAIL)
end
if a==0
    close all;
    figure;
%     plot(target_point_8_SUCCESS,'O');hold on;plot(target_point_8_FAIL,'r.');
%     figure(2);
%     plot(target_point_9_SUCCESS,'O');hold on;plot(target_point_9_FAIL,'r.');
    for i=1:length(target_point_8_SUCCESS)
    plot(target_point_8_SUCCESS(i),target_point_7_SUCCESS(i),'bO');hold on;
    end
    hold on;
    for i=1:length(target_point_8_FAIL)
    plot(target_point_8_FAIL(i),target_point_7_FAIL(i),'rO');hold on;
    end
    fprintf("再见");
    SVM_8_9; 
%     plot3(target_point_8_SUCCESS,target_point_9_SUCCESS,linspace(1,length(target_point_8_SUCCESS),length(target_point_8_SUCCESS)),'O')
%     hold on
%     plot3(target_point_8_FAIL,target_point_9_FAIL,linspace(1,length(target_point_9_FAIL),length(target_point_9_FAIL)),'rO')
%     gride on
end
% Finished: play music
toc
load handel                 % play aleluia song to wake me up
sound( y , Fs )


