% Initialize data for trim and simulation.
% This script setup the nominal parameter of the aircraft dynamics
% state_dot = f(state,input)

global  g rho c b S CL_0 CL_alpha CL_eq CL_de CD_0 AR e CY_beta CY_dr ...
    Tmax Cl_beta Cl_p Cl_r Cl_da Cl_dr Cm_0 Cm_alpha Cm_eq Cm_de ...
    Cn_beta  Cn_p Cn_r Cn_da Cn_dr m Ixx Iyy Izz

rho = 1.225;
g = 9.81;
m = 0.4;      %[kg]
Ixx = 915.06*(1e-5); %kg*m2
Iyy = 422.48*(1e-5); %kg*m2
Izz = 1337.5*(1e-5); %kg*m2
Ixy = -166.07*(1e-5); %kg*m2
Ixz = -0.0001*(1e-5); %kg*m2
Iyz = 0;

S = 0.138;     %[m^2]
b = 1.22;        %[m]
c = 0.29;         %[m]
T_W_ratio = 1.2;
Tmax = m * g * T_W_ratio;  %we guess TMAX 33 %  OF the mass

% LONGITUDINAL

CL_alpha = 4.44;
CDalpha = 0.33;
Cm_alpha = - 0.683;
CLalpha_dot = 0;
Cmalpha_dot = - 4.36;
CLq = 3.8;
Cmq = - 9.96;
CL_de = 0.355;
Cm_de = - 0.923;

CL_eq = CLalpha_dot + CLq;
Cm_eq = Cmalpha_dot + Cmq;

% CL = 0.41;    %pag 400 Aircraft control and simulation Brian L.Stevin
% CD = 0.05;
CL = 1;    %pag 400 Aircraft control and simulation Brian L.Stevin
CD = 0.1;
% To inverse engineering some parameters
% CL=CL0+CLalpha*alpha+CLde*de
% Cm=Cm0+Cmalpha*alpha+Cmde*de
% CD=CD0+CD_alpha*alpha;

% x0=zeros(5,1)
% x=fsolve(@equilibriumULTRALIGHT,x0)   %           if ran gives us line 40;
% alpha=x(1),de=x(2),CL0=x(3),CD0=x(4),Cm0=x(5),
%Cm0=0.1325,de=0.0053,CL0=0.5411,CD0=0.0363;
alpha = 0.0889;de = - 0.0256;
CL_0 = 0.0242; CD_0 = 0.0206; Cm_0 = 0.0371; %reversed engineered parameters
%%LATERAL
CY_beta = - .564;
Cl_beta = - 0.074;
Cn_beta = 0.071;
Cl_p = - .41;
Cn_p = - .0575;
Cl_r = 0.107;
Cn_r = - .125;
Cl_da = 0.1342; % -0.134;    %%TO BE CHECKED OUT
Cn_da = - 0.00346;    %%TO BE CHECKED OUT
CY_dr = 0.157;        %%TO BE CHECKED OUT
Cl_dr = 0.118;        %%TO BE CHECKED OUT
Cn_dr = - .0717;      %%TO BE CHECKED OUT

%%
AR = 6.04;   e = 0.8;
%% Store all the aircraft parameters
PARAMETERS_AC = [g rho*8/8 c b S CL_0 CL_alpha CL_eq CL_de CD_0 AR e CY_beta CY_dr ...
    Tmax Cl_beta Cl_p Cl_r Cl_da Cl_dr Cm_0 Cm_alpha Cm_eq Cm_de ...
    Cn_beta  Cn_p Cn_r Cn_da Cn_dr m Ixx Iyy Izz];
