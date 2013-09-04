% init-file for quadrotor
rho     = 1.225;                                                    % [kg/m³]   Dichte der Luft
g       = 9.81;                                                     % [m/s²]    Erdbeschleunigung
w_e     = 7.2921151467e-5;                                          % [rad/s]   Erddrehrate
c_w     = 0.9228*2/3;                                               % [1]       Widerstandsbeiwert bei translatorischer Bewegung
c_M     = 0.9228*4/5;                                               % [1]       Widerstandsbeiwert bei rotatorischer Bewegung
R_p     = 0.1269;                                                   % [m]       Rotorradius
A_xy    = 2*0.023*0.2 + .11*0.12;                                   % [m²]      Fläche in x-/y-Achse
A_z     = 4*0.023*0.2 + .12^2 + 0*R_p^2*pi*4;                       % [m²]      Fläche in z-Achse, ohne Rotoren
l_m     = 0.275;                                                    % [m]       Abstand Motor zu Kern
z_m     = -0.03;                                                    % [m]       z-Abstand Propeller-Schwerpunkt
m       = 1.477;                                                    % [kg]      Gesamtmasse des Quadrokopters, ohne Cam, +40g mit Cam
m_M     = 0.072;                                                    % [kg]      Masse Motor + Propeller 
m_R     = 0.044;                                                    % [kg]      Masse Rohr
m_C     = m - 4*m_M - 2*m_R;                                        % [kg]      Masse Kern
I_x     = 2*m_M*l_m^2 + 1/12*m_C*(2*.12^2) + 1/12*m_R*(2*l_m)^2;    % [kg m²]   Massenträgheitsmoment um x-Achse
I_y     = I_x;                                                      % [kg m²]   Massenträgheitsmoment um y-Achse
I_z     = 4*m_M*l_m^2 + 1/12*m_C*(2*.12^2) + 2*1/12*m_R*(2*l_m)^2;  % [kg m²]   Massenträgheitsmoment um z-Achse
inertia = diag([I_x I_y I_z]);                                      % [kg m²]   Massenträgheitstensor des Quadrokopters
U_max   = 14.8;                                                     % [V]       Maximale Eingangsspannung für Motor

% Identification of Roxxy2827-34 motor with 10x4.5 propeller
J_M     = 2.5730480633e-5;                                           % [kgm^2]

% identification from 27.11.2012 with old data, best data
R_A     = 0.201084219222241;
Psi     = 0.007242179827506;

% new controller model: u_motor = U_in*(alpha_m/i + beta_m)
alpha_m = 0.104863758313889;
beta_m  = 0.549262344777900;

%% Thrust T = CT*rho*pi*R^4*omega^2
% J  = v_z_Motor*60/(UPM*2*R)       = v_z*pi/(R_p*w_M)          % Fortschrittsgrad
% CT = CT3*J^3 + CT2*J^2 + CT1*J + CT0

% witz normalized rotational speed of motor --> increased matrix condition
CT2s = -1.3077e-2;
CT1s = -2.5224e-4;
CT0s = 1.5393579917e-5;  % identification from 15.11.2012
% torque constant, i.e. Torque = k_t*Thrust
k_t =  0.018228626171312;
k_m = -7.011631909766668e-5;

%%  Drag coefficients

C_wxy   = 1/2*A_xy*rho*c_w;                                 % bezogene Widerstandsgröße udot, vdot
C_wz    = 1/2*A_z*rho*c_w;                                  % bezogene Widerstandsgröße wdot
C_mxy   = 1/2*A_z*rho*c_M;                                  % bezogene Widerstandsgröße pdot, qdot
C_mz    = 1/2*A_xy*rho*c_M;                                 % bezogene Widerstandsgröße rdot

%% Initial conditions

F0 = m*g;
w0 = sqrt(m*g/4/CT0s);
U0 =  R_A/Psi*k_t*m*g/4 + Psi*w0;
