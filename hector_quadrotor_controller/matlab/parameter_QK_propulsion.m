% Geometrical data for quadrotor
rho     = 1.225;                                                    % [kg/m³]   Dichte der Luft
g       = 9.81;                                                     % [m/s²]    Erdbeschleunigung
w_e     = 7.2921151467e-5;                                          % [rad/s]   Erddrehrate
c_w     = 0.9228*10/3;                                              % [1]       Widerstandsbeiwert bei translatorischer Bewegung
c_M     = 0.9228*4;                                                 % [1]       Widerstandsbeiwert bei rotatorischer Bewegung
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

% Identification of Roxxy2827-34 motor with 10x4.5 propeller
Psi     = 6.7140e-003;                                              % [Nm/A], new identification 01.06.2012
J_M     = 5.2553e-5;                                                % [kgm^2]
R_A     = 2.2708e-001;                                              % [Ohm], new identification 01.06.2012

% Thrust T = CT0s*omega^2 + CT1s*omega*v_1 +/- CT2s*v_1^2
CT2s    = -1.3077e-2;
CT1s    = -2.5224e-4;
CT0s    =  1.5278e-5;

k_t     = 3.1130e-002;                                              % [m] torque constant, i.e. M = k_t*T

%  Drag coefficients
C_wxy   = 1/2*A_xy*rho*c_w;                                         % bezogene Widerstandsgröße udot, vdot
C_wz    = 1/2*A_z*rho*c_w;                                          % bezogene Widerstandsgröße wdot
C_mxy   = 1/2*A_z*rho*c_M;                                          % bezogene Widerstandsgröße pdot, qdot
C_mz    = 1/2*A_xy*rho*c_M;                                         % bezogene Widerstandsgröße rdot
