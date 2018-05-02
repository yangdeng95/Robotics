% MECH Deng Yang 
% Kinematics Simulation 

function UarmTrajectoryDraw()

% Initialize the fanuc struct
uarm = UarmInit();
via = [160, 0,  200,  0,   200, 0, 200, 0,  200,  0,  200,  0, 200, 160;
       0,  -200,-200,-200,-100,-200, 0,-200,-200,-200,-100,-200, 0, 0;
       190,  0,   0,  0,   0,   0,   0,  0, 100,  0,  100,   0, 100,  190];
%      start  sc  1   sc   2   sc   3   sc   4    sc   5    sc   6    end
color = [0,0,0];   % black

prev_angles = zeros(1,4);
uarm.handles = drawUarm(prev_angles,uarm);
hold on;

% calcuate the position used for the draw
t_f = 40;
dt = 0.1;
time = 0:dt:t_f;
joint_angles = zeros(4,length(time));

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

x_values = interp1(discTime, via(1,:),time);
y_values = interp1(discTime, via(2,:),time);
z_values = interp1(discTime, via(3,:),time);
pos = [x_values; y_values; z_values];

prev_joint_angles = zeros(4,1);
for ii = 1:length(time)
    [~, joint_angles(:,ii)] = UarmIK(pos(:,ii),prev_joint_angles, uarm);
    prev_joint_angles = joint_angles(:,ii);
end

% Draw in 3D
for ii = 1:length(time)
    % Move robot using setFanuc() if solution exists
        setUarm(joint_angles(:,ii), uarm);
    % Plot a point
        plot3(pos(1,ii),pos(2,ii),pos(3,ii),'MarkerEdgeColor',color, 'Marker', '.', 'MarkerSize', 3)
    % Bricklaying process  6 bricks
     if ii >= t2/dt
        plot3(200,-200,0,'Marker','s','MarkerSize',4,'MarkerEdgeColor',[1,0,0], 'MarkerFaceColor',[1,0,0])
     end
     if ii >= t4/dt
        plot3(200,-100,0,'Marker','s','MarkerSize',4,'MarkerEdgeColor',[1,0,0], 'MarkerFaceColor',[1,0,0])
     end
     if ii >= t6/dt
        plot3(200,0,0,'Marker','s','MarkerSize',4,'MarkerEdgeColor',[1,0,0], 'MarkerFaceColor',[1,0,0])
     end
     if ii >= t8/dt
        plot3(200,-200,100,'Marker','s','MarkerSize',4,'MarkerEdgeColor',[1,0,0], 'MarkerFaceColor',[1,0,0])
     end
     if ii >= t10/dt
        plot3(200,-100,100,'Marker','s','MarkerSize',4,'MarkerEdgeColor',[1,0,0], 'MarkerFaceColor',[1,0,0])
     end
     if ii >= t12/dt
        plot3(200,0,100,'Marker','s','MarkerSize',4,'MarkerEdgeColor',[1,0,0], 'MarkerFaceColor',[1,0,0])
     end

end

end

