function [ trajectory ] = createRobTrajectory( via, rob )
% MECH 498/598 - Intro to Robotics - Spring 2016
% Lab 3
% Solutions by Craig McDonald and Deng Yang
%
%    DESCRIPTION - Generate a joint position and velocity trajectory to be
%    used by the controller.
%
%    The input via is a 3x4 matrix containing via points for the end
%    effector. Each column contains a point in 3D space. The trajectory
%    should move sequentially through each via point. The best approach is
%    to simply move in a straight line in cartesian space between the
%    points. Use robIK() to find the corresponding trajectory in joint
%    space.
%
%    The output trajectory is a 7xn matrix. The first row will be
%    equally-spaced time stamps starting at time zero and finishing at time
%    t_f given to you below. Rows 2 through 4 contain joint angles,
%    and rows 5 7 should contain joint velocities.

t_f = 30; % final time (do not change) [s]
det_t = 0.01;
t = 0:det_t:t_f;
joint_angle = zeros(3,length(t));
joint_velocity = zeros(3,length(t));

path1 = via(:,2)- via(:,1);
path2 = via(:,3)- via(:,2);
path3 = via(:,4)- via(:,3);
h1 = path1/1000;
h2 = path2/999;
h3 = path3/999;
position1 = zeros(3,1001);
position2 = zeros(3,1000);
position3 = zeros(3,1000);
position1(:,1) = via(:,1);
position2(:,1) = via(:,2);
position3(:,1) = via(:,3);
% give the position in Cartisian coordinates
for ii = 1:1000
    position1(:,ii+1) = position1(:,1)+h1*ii;
end   
for ii = 1:999
    position2(:,ii+1) = position2(:,1)+h2*ii;
    position3(:,ii+1) = position3(:,1)+h3*ii;
end

for ii = 1:length(t)
    if ii <= 1001
        pos = position1(:,ii);
    elseif ii > 1001 && ii <= 2001
        pos = position2(:,ii-1001);
    elseif ii > 2001 && ii <= 3001
        pos = position3(:,ii-2001);
    end
    
    if ii == 1
        [~,angle] = robIK(pos,[0,0,0],rob);
    else
        [~,angle] = robIK(pos,joint_angle(:,ii-1),rob);
    end
       joint_angle(:,ii) = angle';
end
joint_velocity(:,1) = (joint_angle(:,2)-joint_angle(:,1))/det_t;
joint_velocity(:,length(t)) = (joint_angle(:,length(t))-joint_angle(:,length(t)-1))/det_t;
for ii = 2:length(t)-1
   joint_velocity(:,ii) = (joint_angle(:,ii+1)-joint_angle(:,ii-1))/(2*det_t); % central difference
end

trajectory(1,:) = t; %Time
trajectory(2:4,:) = joint_angle; %Joint angles
trajectory(5:7,:) = joint_velocity; %Joint velocities

end

