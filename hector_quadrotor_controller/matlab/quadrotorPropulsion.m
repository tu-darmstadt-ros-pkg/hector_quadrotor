function [y,xpred] = quadrotorPropulsion(xin, uin, parameter, dt)
%#eml

assert(isa(xin,'double'));
assert(all(size(xin) == [4 1]));

assert(isa(uin,'double'));
assert(all(size(uin) == [10 1]));

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

% initialize vectors
xpred   = xin;      % motorspeed
v_1     = zeros(4,1);
y       = zeros(14,1);
F_m     = zeros(4,1);
U       = zeros(4,1);
M_e     = zeros(4,1);
I       = zeros(4,1);
F       = zeros(3,1);

% Input variables
u     = uin(1);
v     = uin(2);
w     = uin(3);
p     = uin(4);
q     = uin(5);
r     = uin(6);
U(1)  = uin(7);
U(2)  = uin(8);
U(3)  = uin(9);
U(4)  = uin(10);

% Constants
CT0s = parameter.CT0s;
CT1s = parameter.CT1s;
CT2s = parameter.CT2s;
k_t  = parameter.k_t;
l_m  = parameter.l_m;
Psi  = parameter.Psi;

v_1(1) = - w + l_m*q;
v_1(2) = - w - l_m*p;
v_1(3) = - w - l_m*q;
v_1(4) = - w + l_m*p;

% calculate thrust for all 4 rotors
for i = 1:4
    % if the flow speed at infinity is negative
    if v_1(i) < 0
        F_m(i) = CT2s*v_1(i).^2 + CT1s*v_1(i).*xin(i) + CT0s*xin(i).^2;
        % if the flow speed at infinity is positive
    else
        F_m(i) = -CT2s*v_1(i).^2 + CT1s*v_1(i).*xin(i) + CT0s*xin(i).^2;
    end
    % sum up all rotor forces
    F(3) = F(3) + F_m(i) ;
    
    [M_e(i),I(i),xpred(i)] = motorspeed(xin(i), [U(i) F_m(i)], parameter, dt);
end

% System output, i.e. force and torque of quadrotor
y(1) = F(1);
y(2) = F(2);
y(3) = F(3);

% torque for rotating quadrocopter around x-axis is the mechanical torque
y(4) = (F_m(4)-F_m(2))*l_m;
% torque for rotating quadrocopter around y-axis is the mechanical torque
y(5) = (F_m(1)-F_m(3))*l_m;
% torque for rotating quadrocopter around z-axis is the electrical torque
y(6) = (-M_e(1)-M_e(3)+M_e(2)+M_e(4));

% motor speeds (rad/s)
y(7:10) = xpred(1:4);

% motor current (A)
y(11:14) = I(1:4); % M_e(1:4) / Psi;
