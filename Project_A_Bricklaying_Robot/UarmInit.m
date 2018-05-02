% MECH 598 Deng Yang

function [ uarm_struct ] = UarmInit()

% Fill in uarm parameters and other necessary parameters 
g = 9.81; % gravity constant [kg/m^2]
m1 = 0.4; % mass of link 1 [kg]
m2 = 0.6; % mass of link 2 [kg]
m3 = 0.6; % mass of link 3 [kg]
m4 = 0.3; % mass of link 4 [kg]
G = 5; % Gravitation of a brick
l1 = 100; % length of link 1 [mm]
l2 = 150; % length of link 2 [mm]
l3 = 160; % length of link 3 [mm]
l4 = 60; %length of link 4 [mm]
I1 = m1*(l1*1e-3)^2/11; % inertia of link 1 [kg-m^2]
I2 = m2*(l2*1e-3)^2/12; % inertia of link 2 [kg-m^2]
I3 = m3*(l3*1e-3)^2/13; % inertia of link 3 [kg-m^2]
I4 = m4*(l4*1e-3)^2/14; % inertia of link 4 [kg-m^2]
J1 = I1/2; % inertia of actuator 1 [kg-m^2]
J2 = I2/4; % inertia of actuator 2 [kg-m^2]
J3 = I3/4; % inertia of actuator 3 [kg-m^2]
J4 = I4/4; % inertia of actuator 4 [kg-m^2]

uarm_struct.parameters.g = g;
uarm_struct.parameters.m1 = m1;
uarm_struct.parameters.m2 = m2;
uarm_struct.parameters.m3 = m3;
uarm_struct.parameters.m4 = m4;
uarm_struct.parameters.G = G;
uarm_struct.parameters.l1 = l1;
uarm_struct.parameters.l2 = l2;
uarm_struct.parameters.l3 = l3;
uarm_struct.parameters.l4 = l4;
uarm_struct.parameters.I1 = I1;
uarm_struct.parameters.I2 = I2;
uarm_struct.parameters.I3 = I3;
uarm_struct.parameters.I4 = I4;
uarm_struct.parameters.J1 = J1;
uarm_struct.parameters.J2 = J2;
uarm_struct.parameters.J3 = J3;
uarm_struct.parameters.J4 = J4;

% Robot joint limits (deg)
deg2rad = pi/180;
uarm_struct.joint_limits{1} = [-180,180]*deg2rad;
uarm_struct.joint_limits{2} = [-90,90]*deg2rad;
uarm_struct.joint_limits{3} = [-180,180]*deg2rad;
uarm_struct.joint_limits{4} = [-180,180]*deg2rad;

% Set bounds on the cartesian workspace of the robot
r_xy = 310;
r_zu = 350;
r_zd = 100;
uarm_struct.workspace = [-r_xy,r_xy,-r_xy,r_xy,-r_zd,r_zu];

% Set colors to be drawn for each link and associated frame, including the
% tool
uarm_struct.colors = default_colors(5);


    function [ C ] = default_colors( n_colors )
    % Outputs the default colors

    C = {[     0,    0.0290,    0.9910];
         [0.0070,    0.8880,    0.9250];
         [0.5840,    0.5250,    0.4800];
         [0.8940,    0.0840,    0.9560];
         [0.9660,    0.0740,    0.0940];
         [0.3010,    0.7450,    0.9330];
         [0.6350,    0.0780,    0.8500]};
    C = C(1:n_colors);

    end


end

