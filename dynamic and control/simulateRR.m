function [  ] = simulateRR(  )
% MECH 498 - Intro to Robotics - Spring 2016
% Lab 3
% Solutions by Craig McDonald
%
%    DESCRIPTION - This function should run a dynamic simulation of the RR
%    robot for this assignment, and then play a video of the resulting
%    robot motion.
%
%
%    ADDITIONAL CODE NEEDED: lots
%    

close all;

% Initialize robot
robot = RRInit();
m_1 = robot.m_1;
m_2 = robot.m_2;
m_r1 = robot.m_r1;
m_r2 = robot.m_r2;
l_1 = robot.l_1;
l_2 = robot.l_2;
g = robot.g;

M_1 = m_1+m_r1;
M_2 = m_2+m_r2;
l_c1 = ((m_1+0.5*m_r1)*l_1)/M_1;
l_c2 = ((m_2+0.5*m_r2)*l_2)/M_2;
I1 = (1/12)*m_r1*l_1^2+m_1*(l_1/2)^2;
I2 = (1/12)*m_r2*l_2^2+m_2*(l_2/2)^2;

% Joint Torque Limit
tau_max = 20; % [N-m] (Scalar)

% Time
dt = 0.01; % [s]
t_f = 10; % [s]

% Initial Conditions
X_0 = [pi/3;0;pi/2;0];    % theta1 theta1_dot theta2 theta2_dot

% Control Gains (Scalar)
K_p = 100;
K_v = 200;

% Numerical Integration
t = 0:dt:t_f;
X = zeros(4,length(t)); % initialize variable to hold state vector
X_dot = zeros(4,length(t)); % initialize variable to hold state vector derivatives
thetad = [0;pi/2];
KE = zeros(2,length(t));
PE = zeros(2,length(t));

for i = 1:length(t)
    if i == 1
        X(:,i) = X_0;
    else
    % Control torques
    
    tau = [-K_p*(X(1,i-1)-thetad(1))-K_v*X(2,i-1);-K_p*(X(3,i-1)-thetad(2))-K_v*X(4,i-1)];
    
    % Apply joint torque limits
    tau(tau>tau_max) = tau_max;
    tau(tau<-tau_max) = -tau_max;
    
    % Dynamic Model
    M = [M_1*l_c1^2+M_2*(l_1+l_c2*cos(X(3,i-1)))^2+I1+I2, 0 ; 0, M_2*l_c2^2+I2];
    C = [-2*M_2*l_c2*sin(X(3,i-1))*(l_1+l_c2*cos(X(3,i-1)))*X(2,i-1)*X(4,i-1);...
            M_2*l_c2*sin(X(3,i-1))*(l_1+l_c2*cos(X(3,i-1)))*X(2,i-1)^2];
    G = [0;M_2*g*l_c2*cos(X(3,i-1))];

    Theta = M\(tau-C-G);
    X_dot(2,i) = Theta(1);
    X_dot(4,i) = Theta(2);
    
    % Trapezoidal Integration
    if i > 1
        X(2,i) = X(2,i-1)+0.5*(X_dot(2,i-1)+X_dot(2,i))*dt;
        X(4,i) = X(4,i-1)+0.5*(X_dot(4,i-1)+X_dot(4,i))*dt;
        X_dot(1,i) = X(2,i);
        X_dot(3,i) = X(4,i);
        X(1,i) = X(1,i-1)+0.5*(X(2,i-1)+X(2,i))*dt;
        X(3,i) = X(3,i-1)+0.5*(X(4,i-1)+X(4,i))*dt;
    end
    
    % Plot Energy
        KE(:,i) = 0.5*X([2,4],i)'*M*X([2,4],i);
        PE(2,i) = M_2*g*l_c2*sin(X(3,i));
    end
end

% Graphical Simulation
robot.handles = drawRR([X_0(1),X_0(3)],robot);
for i = 2:length(t)
    setRR([X(1,i),X(3,i)],robot);
    pause(1e-6); % adjustable pause in seconds
end

% Plot Output
figure;
plot(t,X(1,:))
xlabel('time(s)')
ylabel('theta 1 position')

figure;
plot(t,X(3,:))
xlabel('time(s)')
ylabel('theta 2 position')

figure
title('energy figures')
subplot(2,2,1)
plot(t,KE(1,:))
ylabel('kinetic energy theta1')
xlabel('time')
subplot(2,2,2)
plot(t,KE(2,:))
ylabel('kinetic energy theta2')
xlabel('time')
subplot(2,2,3)
plot(t,PE(2,:))
ylabel('potential energy theta2')
xlabel('time')
subplot(2,2,4)
plot(t,KE(1,:)+KE(2,:)+PE(2,:))
ylabel('total energy')
xlabel('time')
end

