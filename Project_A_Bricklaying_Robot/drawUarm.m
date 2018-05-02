% MECH 598 Deng Yang
function [ handles ] = drawUarm( joint_angles, uarm )
      

% Create structure of Rob forward kinematics transforms
[~,uarm_T] = UarmFK(joint_angles,uarm);

% Shorten variable names
L_1 = uarm.parameters.l1; % [mm]
L_2 = uarm.parameters.l2; % [mm]
L_3 = uarm.parameters.l3; % [mm]
L_4 = uarm.parameters.l4; % [mm]

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plot scaling properties
origin_size = 20;
marker_size = 10;
vector_size = 0.05*max(abs(diff(reshape(uarm.workspace,2,3))));

% Create figure window
figure('Color','w');

% Create axes object
ax = axes('XLim',uarm.workspace(1:2),'YLim',uarm.workspace(3:4),...
   'ZLim',uarm.workspace(5:6));
vw = [31.3,22.8];
set(gca,'View',vw);
grid on;
axis equal;
xlabel('X (mm)','FontSize',16);
ylabel('Y (mm)','FontSize',16);
zlabel('Z (mm)','FontSize',16);

% Create frames and links

% Outline Worskpace
ws = uarm.workspace;
line([ws(1:2),fliplr(ws(1:2)),ws(1)],[ws(3)*[1,1],ws(4)*[1,1],ws(3)],ws(5)*ones(1,5),'Color',[0,0,0]);
line([ws(1:2),fliplr(ws(1:2)),ws(1)],[ws(3)*[1,1],ws(4)*[1,1],ws(3)],ws(6)*ones(1,5),'Color',[0,0,0]);

% Draw object in starting and final position
Obj(1) = hggroup('Parent',ax);
O = line(0,-200,0,'Marker','.','MarkerSize',60,'MarkerFaceColor',[1,0,0]);
set(O,'Parent',Obj(1));
str = 'Bricks Source';
text(-50,-370,0,str)

% Base frame
h = drawRobotFrame([0,0,0]);
hg = hggroup('Parent',ax);
set(h,'Parent',hg);
circ = linspace(0,2*pi,50);
L_0 = line(20*cos(circ),20*sin(circ),...
     0*ones(length(circ)),...
    'Color','k','LineWidth',1.5);
set(L_0,'Parent',hg);
T_0 = hgtransform('Parent',ax,'Matrix',makehgtform('translate',[0,0,0]));
set(hg,'Parent',T_0);

% Create link 1 and frame 1
h = drawRobotFrame(uarm.colors{1});
hg = hggroup('Parent',ax);
set(h,'Parent',hg);
L_1 = line([0,0],[0,0],[-L_1,0],...
    'Color',uarm.colors{1},'LineWidth',1.5);
set(L_1,'Parent',hg);
T_1 = hgtransform('Parent',T_0,'Matrix',uarm_T{1});
set(hg,'Parent',T_1);

% Create link 2 and frame 2
h = drawRobotFrame(uarm.colors{2});
hg = hggroup('Parent',ax);
set(h,'Parent',hg);
L_2 = line([0,L_2],[0,0],[0,0],...
    'Color',uarm.colors{2},'LineWidth',1.5);
set(L_2,'Parent',hg);
T_2 = hgtransform('Parent',T_1,'Matrix',uarm_T{2});
set(hg,'Parent',T_2);

% Create link 3 and frame 3
h = drawRobotFrame(uarm.colors{3});
hg = hggroup('Parent',ax);
set(h,'Parent',hg);
L_3 = line([0,L_3],[0,0],[0,0],...
    'Color',uarm.colors{3},'LineWidth',1.5);
set(L_3,'Parent',hg);
T_3 = hgtransform('Parent',T_2,'Matrix',uarm_T{3});
set(hg,'Parent',T_3);

% Create link 4 and frame 4
h = drawRobotFrame(uarm.colors{4});
hg = hggroup('Parent',ax);
set(h,'Parent',hg);
L_4 = line([0,0],[-L_4,0],[0,0],...
    'Color',uarm.colors{4},'LineWidth',1.5);
set(L_4,'Parent',hg);
T_4 = hgtransform('Parent',T_3,'Matrix',uarm_T{4});
set(hg,'Parent',T_4);

h = drawRobotFrame(uarm.colors{5});
hg = hggroup('Parent',ax);
set(h,'Parent',hg);
Obj(2) = hggroup('Parent',ax);
O = line(0,0,0,'Color',[0,0,0],'Marker','s','MarkerSize',30);
set(O,'Parent',Obj(2));
set(Obj(2),'Visible','off');
T_5 = hgtransform('Parent',T_4,'Matrix',uarm_T{5});
set(hg,'Parent',T_5);
set(Obj(2),'Parent',T_4);

set(gcf,'Renderer','openGL');
drawnow;

% Return hgtransform handles
handles = [T_1,T_2,T_3,T_4,Obj];

    function h = drawRobotFrame( color )
         
        % Plot reference frame
        X_b = [vector_size,0,0,1]';
        Y_b = [0,vector_size,0,1]';
        Z_b = [0,0,vector_size,1]';
        h(1) = line(0,0,0,'Marker','.','MarkerSize',origin_size,'Color',color);
        h(2) = line([0,X_b(1)],[0,X_b(2)],[0,X_b(3)],'LineWidth',1.5,'Color',color);
        h(3) = line([0,Y_b(1)],[0,Y_b(2)],[0,Y_b(3)],'LineWidth',1.5,'Color',color);
        h(4) = line([0,Z_b(1)],[0,Z_b(2)],[0,Z_b(3)],'LineWidth',1.5,'Color',color);
        h(5) = line(X_b(1),X_b(2),X_b(3),'LineWidth',1.5,'Marker','x','MarkerSize',marker_size,'Color',color);
        h(6) = line(Y_b(1),Y_b(2),Y_b(3),'LineWidth',1.5,'Marker','o','MarkerSize',marker_size,'Color',color);
        h(7) = line(Z_b(1),Z_b(2),Z_b(3),'LineWidth',1.5,'Marker','d','MarkerSize',marker_size,'Color',color);
    end


end

