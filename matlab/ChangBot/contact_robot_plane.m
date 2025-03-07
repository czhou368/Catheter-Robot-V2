clear; clc

robot = creatRobotChang();

%% initial positions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% ------ 1. robot initial pose ---------

loads0.f1_body = [0; 0; 0];    %  N/mm, the distributed self-weight.
loads0.f4_body = [0; 0; 0];    %  N/mm, the distributed self-weight.
loads0.F_endPoint = [0, 0, 0]';   % N

q0 = [1 -1 1 -1];
ini_guess = zeros(10,1);
F0 = zeros(3,1);

[shape0, ini0, res0, fk_results] = robot_disp_fk(robot, q0, loads0, ini_guess);

state_ini.shape = shape0;
state_ini.loads = loads0;
state_ini.fk_results = fk_results;
state_ini.ini = ini0;
state_ini.q = q0;

% ------- 2. plane positin -----------
tip0 = shape0(:, end);
wall_dis = 1.02*tip0(3);
p0 = [0;0;wall_dis];
n_p = [0;0;-1];
mu = 0.1;

plane = objPlane(p0, n_p, mu);
% plane_fl = objPlane(p0, n, 1e-5);


% -------- 3. plane traj -----------

dalpha = deg2rad(1);   % rotate in 1 degree per step
dbeta = 1e-1;   % move in 1e-1 mm per step
angleLim = pi/4;
betaLim = - 0.05*tip0(3);
q1 = [1 -1 0 0];
dq = 1e-2;     % actuated in 1e-2 mm per step

% transl_traj1 = LinearInterplNdim([0, betaLim], dbeta);
% n1 = length(transl_traj1);
% angle_traj1 = zeros(1, n1);
% q1_traj = repmat(q0, [n1, 1]);

q1_traj = LinearInterplNdim([q0', q1'], dq)';
n1 = size(q1_traj, 1);
angle_traj1 = zeros(1, n1);
transl_traj1 = zeros(1, n1);
% q1_traj = repmat(q0, [n1, 1]);

angle_traj2 = LinearInterplNdim([0, angleLim, -angleLim], dalpha);
n2 = length(angle_traj2);
transl_traj2 = ones(1, n2)*transl_traj1(end);
q2_traj = repmat(q0, [n2, 1]);

q_traj = q1_traj;
angle_traj = angle_traj1;
transl_traj = transl_traj1;
n = n1;

% q_traj = [q1_traj; q2_traj];
% angle_traj = [angle_traj1, angle_traj2];
% transl_traj = [transl_traj1, transl_traj2];
% 
% n = n1 + n2;

%% 

tic

% initialize conditions

tipContact = false;
tip_traj = [];
% tip_traj_fl = [];

state_traj = [];
% state_traj_fl = [];

state_prev = state_ini;
maxIter = n;

for i = 1:maxIter

    % move plane & actuation input
    qi = q_traj(i, :);
    alphaI = angle_traj(i);
    betaI = transl_traj(i);

    plane.T_history(:,:,end+1) = [RotZ(alphaI), p0+[0;0;betaI]; 0 0 0 1];
    plane = plane.rebuild();

    % assume ONLY tip contact, NOT entire body.
    if tipContact

        [state_update,tipContact] = updateTipContact(robot, plane, qi, state_prev);

        state_prev = state_update;

        p_tip = state_update.shape(:, end);

    else
        % get initial guess from previous step.
        [shape, ini_sol, res0, fk_results] = robot_disp_fk(robot, qi, loads0, state_prev.ini);

        state_update.shape = shape;
        state_update.loads = loads0;
        state_update.fk_results = fk_results;
        state_update.ini = ini_sol;
        state_update.q = qi;

        state_prev = state_update;

        % check if tip contact
        p_tip = shape(:, end);

        if plane.computeDepth(p_tip) >= 0
            tipContact = true;
        else
            tipContact = false;
        end

    end

    tip_traj = [tip_traj, p_tip];
    state_traj = [state_traj, state_update];

    disp('Finished ' +string(i) + '/' + string(maxIter))

end

tolTime = toc;

disp(tolTime/n)

%% plot the traj

vid1 = VideoWriter('out/vid_contact1', 'MPEG-4');
% close(vid1)
open(vid1);

figure()
hold on
plotConfig3D([720 480])

% plot3Mat(tip_traj)

xlabel('x (mm)')
ylabel('y (mm)')
zlabel('z (mm)')
axis equal
xlim([-100, 100])
ylim([-10, 100])
zlim([-2, 300])

% view(0,20)
% view([150, -45])

view(-120, -20)
camup([0 1 0])

hp = [];
hs = []; ht = [];

% plane radius / clock radiu
Rp = 90;
Rc = 20;

ns = length(state_traj);

plane_i = plane;

for i = 1:ns

    statei = state_traj(i);

    delete([hp hs ht])

    % update plane location
    alphai = angle_traj(i);
    betai = transl_traj(i);

    plane_i.p0 = p0 + betai;
    hp = plane_i.plotPlaneClock(Rp, Rc, alphai, 12);   % fix the rotation?
    hs = plotRobot(robot, statei.fk_results.g);
    ht = plot3Mat(tip_traj(:,1:i), 'b');

    writeVideo(vid1, getframe(gcf));

    pause(0.01)
    
end

close(vid1)

