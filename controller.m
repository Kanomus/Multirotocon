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

dt = 1/100;

k.p = 30;
k.i = 0.18;
k.d = 1800;
k.rp = 70;
k.ri = 0.001;
k.rd = 0.5;

pos_error = des_state.pos - state.pos;
y_error = pos_error(1);
z_error = pos_error(2);

persistent int_y_error;
persistent int_z_error;
persistent prev_z_error;
persistent prev_y_error;

if(isempty(int_y_error))
    int_y_error = 0;
end
if(isempty(int_z_error))
    int_z_error = 0;
end
if(isempty(prev_y_error))
    prev_y_error = y_error;
end
if(isempty(prev_z_error))
    prev_z_error = z_error;
end


%---------------working in the z direction--------------

Pz = k.p * z_error;

int_z_error = int_z_error + (z_error*dt);
diff_z_error = (z_error - prev_z_error)/dt;

Iz = k.i * int_z_error;
Dz = k.d * diff_z_error;

%----------------working in the y direction---------------

phi_max = pi/6;
%mapping y_error [0,inf]) to [0, pi/2)
%des_theta = - atan(2 * y_error);
%changing limit from pi/2 to pi/6
%des_theta = (1/3) * des_theta;

des_theta = - y_error;
if(des_theta>0)
    des_theta = min(phi_max, des_theta);
else
    des_theta = max(-phi_max, des_theta);
end

theta_error = des_theta - state.rot;
Pr = k.rp * theta_error;

Dr = - (k.rd * state.omega);

int_y_error = int_y_error + (y_error * dt);
Ir =  - (k.ri * int_y_error);

%------------------final calculations--------------

u1 = Pz + Iz + Dz;
u2 = Pr + Ir + Dr;

prev_z_error = z_error;
prev_y_error = y_error;

end

