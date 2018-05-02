% MECH 598 Deng Yang

function [T,uarm_T] = UarmFK(joint_angles,uarm)
    theta1 = joint_angles(1);
    theta2 = joint_angles(2);
    theta3 = joint_angles(3);
    theta4 = joint_angles(4);
    
    T1 = dhtf(0,0,uarm.parameters.l1,theta1);
    T2 = dhtf(pi/2,0,0,theta2+pi/2);
    T3 = dhtf(0,uarm.parameters.l2,0,theta3-pi/2);
    T4 = dhtf(0,uarm.parameters.l3,0,theta4);
    T5 = dhtf(pi/2,0,uarm.parameters.l4,0);
    T = T1*T2*T3*T4*T5;
    uarm_T = {T1,T2,T3,T4,T5};
end