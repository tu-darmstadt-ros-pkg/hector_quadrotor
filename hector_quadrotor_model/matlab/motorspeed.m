function [M_e,I,xpred] = motorspeed(xin, uin, parameter, dt)
%#eml

assert(isa(xin,'double'));
assert(all(size(xin) == 1));

assert(isa(uin,'double'));
assert(all(size(uin) == [2 1]));

assert(isa(parameter,'struct'));
assert(isa(parameter.k_m,'double'));
assert(isa(parameter.k_t,'double'));

assert(isa(parameter.CT2s,'double'));
assert(isa(parameter.CT1s,'double'));
assert(isa(parameter.CT0s,'double'));

assert(isa(parameter.Psi,'double'));
assert(isa(parameter.J_M,'double'));
assert(isa(parameter.R_A,'double'));

assert(isa(parameter.alpha_m,'double'));
assert(isa(parameter.beta_m,'double'));

assert(isa(parameter.l_m,'double'));

assert(isa(dt,'double'));
assert(all(size(dt) == 1));

eml.cstructname(parameter,'PropulsionParameters');

% Identification of Roxxy2827-34 motor with 10x4.5 propeller
Psi     =  parameter.Psi;
J_M     =  parameter.J_M;
R_A     =  parameter.R_A;
k_m     = parameter.k_m;
k_t     = parameter.k_t;
alpha_m = parameter.alpha_m;
beta_m  = parameter.beta_m;

% temporarily used Expressions
U       = uin(1);
F_m     = uin(2);
omega_m = xin(1);

M_m   = k_t*F_m + k_m*omega_m;

temp  = (U*beta_m-Psi*omega_m)./(2*R_A);
I     = temp + sqrt(temp.^2 + (U*alpha_m./R_A));
M_e   = Psi*I;                  % electrical torque motor 1-4

% new version
fx   = 1/J_M.*(M_e - M_m);

% old version
% fx   = (Psi/R_A*(U-Psi*omega_m) - M_m)/J_M;
% A    = -(Psi^2/R_A)/J_M;
% B(1) =  Psi/(J_M*R_A);
% B(2) = -1/J_M;

% system outputs. Use euler solver to predict next time step
% predicted motor speed
xpred   = xin + dt*fx;
% electric torque
%y = [M_e I];

% system jacobian
% A       = 1 + dt*A;
% input jacobian
% B       = A*B*dt;

