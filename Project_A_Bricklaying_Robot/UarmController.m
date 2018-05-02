% MECH 598 Deng Yang
% Controller output:tau_dot
function [ tau_dot ] = UarmController(trajectory,Theta,Theta_dot,t)
   
% Trajectory interpolation
Theta_ref = zeros(4,1);
Theta_d_ref = zeros(4,1);
Theta_dd_ref = zeros(4,1);
for i = 1:4
    Theta_ref(i) = interp1(trajectory(1,:),trajectory(i+1,:),t);
    Theta_d_ref(i) = interp1(trajectory(1,:),trajectory(i+5,:),t);
    Theta_dd_ref(i) = interp1(trajectory(1,:),trajectory(i+9,:),t);
end

% Trajectory-following Control - PD Control
K_p = 50; % Proportional gain matrix containing gains K_p1 to K_p4
K_v = 14.14; % Derivative gain matrix containing gains K_v1 to K_v4

tau_dot = Theta_dd_ref-K_p.*(Theta-Theta_ref)-K_v.*(Theta_dot-Theta_d_ref); % control input (torque)

end
