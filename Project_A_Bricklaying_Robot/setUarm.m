function setUarm( angles, uarm )

[~,uarm_T] = UarmFK(angles,uarm);
set(uarm.handles(1),'Matrix',uarm_T{1});
set(uarm.handles(2),'Matrix',uarm_T{2});
set(uarm.handles(3),'Matrix',uarm_T{3});
set(uarm.handles(4),'Matrix',uarm_T{4});
drawnow;

end