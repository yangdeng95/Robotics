% MECH 598 Deng Yang
% Simulate Uarm
function [] = SimulateUarm( )
close all;

% Initialize
uarm = UarmInit();
%      start  sc  1   sc   2   sc   3   sc    4    sc   5   sc   6   end
via = [160, 0,  200,  0,   200, 0, 200, 0,  200,  0,  200, 0, 200,  160;
       0,  -200,-200,-200,-100,-200, 0,-200,-200,-200,-100,-200, 0,  0;
       190,  0,   0,  0,   0,   0,   0,  0,  100,  0,  100, 0, 100, 190];
size = 20; % brick sizes

% Time
dt = 0.1; % [s]
t_f = 80; % [s]
t = 0:dt:t_f;

% Initial Conditions
% theta1 theta1_dot theta2 theta2_dot theta3 theta3_dot
X_0 = zeros(8,1);
[trajectory,Time]= createUarmTrajectory(via,uarm);
X_0(1) = trajectory(2,1);
X_0(2) = trajectory(6,1);
X_0(3) = trajectory(3,1);
X_0(4) = trajectory(7,1);
X_0(5) = trajectory(4,1);
X_0(6) = trajectory(8,1);
X_0(7) = trajectory(5,1);
X_0(8) = trajectory(9,1);
% Numerical Integration
X = zeros(8,length(t)); % initialize variable to hold state vector
X_dot = zeros(8,length(t)); % initialize variable to hold state vector derivatives
tau_matrix = zeros(4,length(t)); % collect tau
Theta = zeros(4,1);
Thetad = zeros(4,1);

for ii = 1:length(t)-1
    if ii == 1
        X(:,ii) = X_0;
        X_dot(1,ii) = X(2,ii);
        X_dot(3,ii) = X(4,ii);
        X_dot(5,ii) = X(6,ii);
        X_dot(7,ii) = X(8,ii);
    end
    
    % new theta and thetad
    Theta(1) = X(1,ii);Theta(2) = X(3,ii);Theta(3) = X(5,ii);Theta(4) = X(7,ii);
    Thetad(1) = X(2,ii);Thetad(2) = X(4,ii);Thetad(3) = X(6,ii);Thetad(4) = X(8,ii);
    
    % Control torques
    tau_dot = UarmController(trajectory,Theta,Thetad,t(ii));
    Thetadd = tau_dot;
    X_dot(2,ii) = Thetadd(1);
    X_dot(4,ii) = Thetadd(2);
    X_dot(6,ii) = Thetadd(3);
    X_dot(8,ii) = Thetadd(4);

    % Integration
    X(2,ii+1) = X(2,ii) + X_dot(2,ii)*dt;
    X(4,ii+1) = X(4,ii) + X_dot(4,ii)*dt;
    X(6,ii+1) = X(6,ii) + X_dot(6,ii)*dt;
    X(8,ii+1) = X(8,ii) + X_dot(8,ii)*dt;
    X_dot(1,ii+1) = X(2,ii+1);
    X_dot(3,ii+1) = X(4,ii+1);
    X_dot(5,ii+1) = X(6,ii+1);
    X_dot(7,ii+1) = X(8,ii+1);
    X(1,ii+1) = X(1,ii) + X(2,ii)*dt;
    X(3,ii+1) = X(3,ii) + X(4,ii)*dt;
    X(5,ii+1) = X(5,ii) + X(6,ii)*dt;
    X(7,ii+1) = X(7,ii) + X(8,ii)*dt;    
    % Dynamic Model
    if  Time(2)<=ii<=Time(3) || Time(4)<=ii<=Time(5) || Time(6)<=ii<=Time(7) ||  ...,
            Time(8)<=ii<=Time(9)|| Time(10)<=ii<=Time(11) || Time(12)<=ii<=Time(13)
        tau = UarmDynamics_load(Theta,Thetad,Thetadd,uarm);
    else
        tau = UarmDynamics_noload(Theta,Thetad,Thetadd,uarm);
    end
    tau_matrix(:,ii) = tau;
end

% Graphical Simulation
uarm.handles = drawUarm([X_0(1,1),X_0(3,1),X_0(5,1),X_0(7,1)],uarm);
hold on
for ii = 2:length(t)
    setUarm([X(1,ii),X(3,ii),X(5,ii),X(7,ii)],uarm);
    % Bricklaying process 6 bricks
    if ii >= Time(3)/dt
       plot3(200,-200,0,'Marker','s','MarkerSize',size,'MarkerEdgeColor',[1,0,0], 'MarkerFaceColor',[1,0,0])
    end
    if ii >= Time(5)/dt
       plot3(200,-100,0,'Marker','s','MarkerSize',size,'MarkerEdgeColor',[1,0,0], 'MarkerFaceColor',[1,0,0])
    end
    if ii >= Time(7)/dt
       plot3(200,0,0,'Marker','s','MarkerSize',size,'MarkerEdgeColor',[1,0,0], 'MarkerFaceColor',[1,0,0])
    end
    if ii >= Time(9)/dt
       plot3(200,-200,100,'Marker','s','MarkerSize',size,'MarkerEdgeColor',[1,0,0], 'MarkerFaceColor',[1,0,0])
    end
    if ii >= Time(11)/dt
       plot3(200,-100,100,'Marker','s','MarkerSize',size,'MarkerEdgeColor',[1,0,0], 'MarkerFaceColor',[1,0,0])
    end
    if ii >= Time(13)/dt
        plot3(200,0,100,'Marker','s','MarkerSize',size,'MarkerEdgeColor',[1,0,0], 'MarkerFaceColor',[1,0,0])
    end
end
% plot torque
figure
title('Torque')
subplot(2,2,1)
plot(t,tau_matrix(1,:))
ylabel('Joint1 Torque N/m')
xlabel('time')
subplot(2,2,2)
plot(t,tau_matrix(2,:))
ylabel('Joint2 Torque N/m')
xlabel('time')
subplot(2,2,3)
plot(t,tau_matrix(3,:))
ylabel('Joint3 Torque N/m')
xlabel('time')
subplot(2,2,4)
plot(t,tau_matrix(4,:))
ylabel('Joint4 Torque N/m')
xlabel('time')
end

