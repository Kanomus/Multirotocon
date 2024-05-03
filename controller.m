function [ u1, u2 ] = controller(~, state, des_state, params)
%CONTROLLER  Controller for the planar quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [y; z], state.vel = [y_dot; z_dot], state.rot = [phi],
%   state.omega = [phi_dot]
%
%   des_state: The desired states are:
%   des_state.pos = [y; z], des_state.vel = [y_dot; z_dot], des_state.acc =
%   [y_ddot; z_ddot]
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls
%   u1 = F, u2 = M

k.p = 1;
k.i = 1;
k.d = 1;
k.rp = 1;
k.ri = 0;
k.rd = 0.4;

vector = des_state.pos - state.pos;

if vector(1) ~= 0
    req_theta = atan(vector(2) / vector(1));
else
    % Handle the case where vector(1) is zero
    req_theta = state.rot + (pi/2);
end
% req_theta = atan(vector(2)/vector(1));
theta = state.rot + (pi/2);
theta_error = req_theta - theta;

if(theta_error == 0)
    u1 = k.p * sqrt((vector(1))^2 + (vector(2))^2);
    u2 = 0;
else
    u2 = k.rp * theta_error;
    multiplied = vector .* state.vel;
    u1 = k.p * norm(multiplied,2);
end

persistent int_pos_error;
persistent int_theta_error;

if(isempty(int_pos_error))
    int_pos_error = 0;
end
if(isempty(int_theta_error))
    int_theta_error = 0;
end

int_pos_error = int_pos_error + vector;
int_theta_error = int_theta_error + theta_error;

u1 = u1 + (k.i * norm(int_pos_error));
u2 = u2 + (k.ri * int_theta_error);

u1 = u1 + (k.d * sqrt((state.vel(1))^2 + (state.vel(2))^2));
u2 = u2 + (k.rd * state.omega);

end

