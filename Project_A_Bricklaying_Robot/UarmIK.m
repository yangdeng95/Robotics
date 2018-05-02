% MECH 598 Deng Yang
function [ is_solution, joint_angles ] = UarmIK( pos, prev_joint_angles, uarm )
% Rename parameter variables for convenience
l1 = uarm.parameters.l1;
l2 = uarm.parameters.l2;
l3 = uarm.parameters.l3;
l4 = uarm.parameters.l4;

% link 4 position
p_x = pos(1);
p_y = pos(2);
p_z = pos(3)+l4;

% Arm IK solution
theta_1 = [ atan2(p_y,p_x);
            atan2(-p_y,-p_x)];        
p_x1 = p_x./cos(theta_1);
p_z1 = p_z-l1;
cp3 = (l2^2+l3^2-p_x1(1).^2-p_z1^2)/(2*l2*l3);   
sp3 = [sqrt(1-cp3^2),-sqrt(1-cp3^2)];
theta_3 = [ -asin(cp3);
            asin(-cp3)];
phi = [ pi/2 - atan2(p_z1,p_x1(1));
        pi/2 - atan2(p_z1,p_x1(2))];
beta = [ atan2(l3*sp3(1),l2+l3*(-cp3));
         atan2(l3*sp3(2),l2+l3*(-cp3))];
theta_1 = [theta_1(1);theta_1(1);theta_1(2);theta_1(2)];
theta_2 = [-phi(1)+beta(2);-phi(1)+beta(1);phi(2)+beta(2);phi(2)+beta(1)];
theta_3 = [theta_3(1);theta_3(2);theta_3(1);theta_3(2)];
theta_4 = -(theta_2+theta_3);
thetas = [theta_1,theta_2,theta_3,theta_4];

% Check joint limits
bad_sol = false(size(thetas,1),1);
for i = 1:size(thetas,1)
    for j = 1:size(thetas,2)
        while thetas(i,j) < uarm.joint_limits{j}(1)-1e-8
            thetas(i,j) = thetas(i,j) + 2*pi;
        end
        while thetas(i,j) > uarm.joint_limits{j}(2)+1e-8
            thetas(i,j) = thetas(i,j) - 2*pi;
        end
        if thetas(i,j) < uarm.joint_limits{j}(1)-1e-8
            bad_sol(i) = true;
        end
    end
end
thetas(bad_sol,:) = []; % remove solutions outside of joint ranges

% Check for solutions, and find "nearest" solution
if isempty(thetas)
    is_solution = false;
    joint_angles = [];
    warning('no solution');

else
    is_solution = true;
    i = 1;
    while size(thetas,1) > 1
        theta_diff = abs(prev_joint_angles(i)-thetas(:,i));
        thetas = thetas(theta_diff==min(theta_diff),:);
        i = i + 1;
        if i > size(thetas,2)
            thetas = thetas(1,:);
        end
    end
    joint_angles = thetas(1,:);
end       

end