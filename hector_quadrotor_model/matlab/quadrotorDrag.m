function y = quadrotorDrag(uin, parameter, dt)
%#eml

assert(isa(uin,'double'));
assert(all(size(uin) == [6 1]));

assert(isa(parameter,'struct'));
assert(isa(parameter.C_wxy,'double'));
assert(isa(parameter.C_wz,'double'));

assert(isa(parameter.C_mxy,'double'));
assert(isa(parameter.C_mz,'double'));


assert(isa(dt,'double'));
assert(all(size(dt) == 1));

eml.cstructname(parameter,'DragParameters');

% initialize vectors
y     = zeros(6,1);

% Input variables
u     = uin(1);
v     = uin(2);
w     = uin(3);
p     = uin(4);
q     = uin(5);
r     = uin(6);

% Constants
C_wxy = parameter.C_wxy;
C_wz  = parameter.C_wz;
C_mxy = parameter.C_mxy;
C_mz  = parameter.C_mz;

% temporarily used vector
absoluteVelocity        = sqrt(u^2 + v^2 + w^2);
absoluteAngularVelocity = sqrt(p^2 + q^2 + r^2);

% system outputs
% calculate drag force
y(1) = C_wxy* absoluteVelocity*u;
y(2) = C_wxy* absoluteVelocity*v;
y(3) = C_wz * absoluteVelocity*w;

% calculate draq torque
y(4) = C_mxy* absoluteAngularVelocity*p;
y(5) = C_mxy* absoluteAngularVelocity*q;
y(6) = C_mz * absoluteAngularVelocity*r;
