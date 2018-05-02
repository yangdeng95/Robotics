% MECH 598 Deng Yang
% output:13*n matrix
function [trajectory,T] = createUarmTrajectory( via, uarm )

t_f = 80; % final time (do not change) [s]

dt = 0.1;
t = 0:dt:t_f;

trajectory(1,:) = t; %Time

d1 = abs(norm(via(:,1) - via(:,2)));
d2 = abs(norm(via(:,2) - via(:,3)));
d3 = abs(norm(via(:,3) - via(:,4)));
d4 = abs(norm(via(:,4) - via(:,5)));
d5 = abs(norm(via(:,5) - via(:,6)));
d6 = abs(norm(via(:,6) - via(:,7)));
d7 = abs(norm(via(:,7) - via(:,8)));
d8 = abs(norm(via(:,8) - via(:,9)));
d9 = abs(norm(via(:,9) - via(:,10)));
d10 = abs(norm(via(:,10) - via(:,11)));
d11 = abs(norm(via(:,11) - via(:,12)));
d12 = abs(norm(via(:,12) - via(:,13)));
d13 = abs(norm(via(:,13) - via(:,14)));
totalD = d1+d2+d3+d4+d5+d6+d7+d8+d9+d10+d11+d12+d13;
t1 = d1/totalD*t_f;        % Travelling time between first and second point
t2 = t1 + d2/totalD*t_f;   % Travelling time between first and third point
t3 = t2 + d3/totalD*t_f;   % Travelling time between first and forth point
t4 = t3 + d4/totalD*t_f;   % Travelling time between first and fifth point
t5 = t4 + d5/totalD*t_f;   % Travelling time between first and sixth point
t6 = t5 + d6/totalD*t_f;   % Travelling time between first and seventh point
t7 = t6 + d7/totalD*t_f;   % Travelling time between first and eighth point
t8 = t7 + d8/totalD*t_f;   % Travelling time between first and nineth point
t9 = t8 + d9/totalD*t_f;   % Travelling time between first and tenth point
t10 = t9 + d10/totalD*t_f;   % Travelling time between first and 11 point
t11 = t10 + d11/totalD*t_f;   % Travelling time between first and 12 point
t12 = t11 + d12/totalD*t_f;   % Travelling time between first and 13 point

discTime = [0, t1, t2, t3, t4, t5, t6, t7, t8, t9, t10, t11, t12, t_f];

x_values = interp1(discTime, via(1,:),t);
y_values = interp1(discTime, via(2,:),t);
z_values = interp1(discTime, via(3,:),t);
pos = [x_values; y_values; z_values];

prev_joint_angles = zeros(3,1);
for ii = 1:length(t)
    [~, trajectory(2:5,ii)] = UarmIK(pos(:,ii),prev_joint_angles, uarm);
    prev_joint_angles = trajectory(2:5,ii);
end
T = discTime;
% Velocity
% Forwards difference
for ii = 1:length(t)-1
    trajectory(6:9,ii) = (trajectory(2:5,ii+1) - trajectory(2:5,ii))/dt;
end
% Backward difference
trajectory(6:9,end) = (trajectory(2:5,end) - trajectory(2:5,end-1))/dt;
% Acceleration
% Forward difference
for ii = 1:length(t)-1
    trajectory(10:13,ii) = (trajectory(6:9,ii+1) - trajectory(6:9,ii))/dt;
end
% Backward difference
trajectory(10:13,end) = (trajectory(6:9,end)-trajectory(6:9,end-1))/dt;

end