clear; clc

robot = creatRobotChang();

loads.f1_body = [0; 0; 0];    %  N/mm, the distributed self-weight.
loads.f4_body = [0; 0; 0];    %  N/mm, the distributed self-weight.
loads.F_endPoint = [0, 0, 0]';   % N

actuation = [1 -1 -0.5 0.5];

ini_guess = zeros(10,1);

tic 
[J, C, kf_ini_sol, fk_results] = computeJacobCompl(robot, actuation, loads, ini_guess);

toc

tic
[J_ivp, C_ivp, E, B, res0, ~] = computeJacobComplFast(robot, actuation, loads, kf_ini_sol);

toc

state_i.actuation = actuation;
state_i.sol_results = fk_results;
state_i.tension1 = kf_ini_sol(7:8);
state_i.tension2 = kf_ini_sol(9:10);

state_traj = state_i;

J
C
J_ivp
C_ivp

%% plot

figure()
hold on

xlabel('x(mm)'); ylabel('y(mm)'); zlabel('z(mm)');
axis equal
zlim([-10, robot.L*1.05]); xlim([-150, 150]); ylim([-50, 50]);
view(10,10)
grid on; 

n = length(state_traj);

for i=1:n
    state_i = state_traj(i);
    % state_i.tension1
    % state_i.tension2
    sol_results = state_i.sol_results;
    % plot3(shape_i(1,:), shape_i(2,:), shape_i(3,:),'b');
    plotRobot(robot, sol_results.g);
end




