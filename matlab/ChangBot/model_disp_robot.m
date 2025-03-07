clear; clc

robot = creatRobotChang([170, 50, 5, 40, 5, 5.3, 3, 2.8, 2.5], ...
    [60*1e3, 2*1e3, 60*1e3, 2*1e3, 60*1e3], ...
    [20*1e3, 1*1e3, 20*1e3, 1*1e3, 20*1e3]);

% loads.f_body = [-0.47; 0; 0];    % 0.47 N/m, the self-weight of distributed force.
% loads.F_endPoint = [-0.148, 0, 0]'; % [0, 0, 0]';  % 

loads.f1_body = [0; 0; 0];    %  N/mm, the distributed self-weight1
loads.f4_body = [0; 0; 0];    %  N/mm, the distributed self-weight.
loads.F_endPoint = [0, 0, 0]';   % N

% actuation = [5 -5 5 -5];

ini_guess = zeros(10,1);
ini_sol = ini_guess; 

state_traj = [];


% actuation_i =  [0 0 0.5 -0.5];
% actuation_i =  [-0.2247    0.2247   -0.1323    0.1323];
actuation_i =  [4 -4 1 -1];

[shape, ini_sol, res, results] = fk_shooting(robot, actuation_i, loads, ini_sol);

state_i.shape = shape;
state_i.actuation = actuation_i;
state_i.ini = ini_sol;
state_i.sol_results = results;
state_i.res = res;
state_i.tension1 = ini_sol(7:8);
state_i.tension2 = ini_sol(9:10);

state_traj = [state_traj, state_i];



%% plot

figure()
hold on

xlabel('x(mm)'); ylabel('y(mm)'); zlabel('z(mm)');
axis equal
zlim([-10, robot.L*1.05]); xlim([-150, 150]); ylim([-50, 50]);
% zlim([-0.2, 0.2]); xlim([-0.15, 0.15]); ylim([0, 0.25]);
view(10,10)
grid on; 

n = length(state_traj);

% tension_traj = zeros(1, n);

for i=1:n
    state_i = state_traj(i);
    shape_i = state_i.shape;
    state_i.res
    state_i.tension1
    state_i.tension2
    sol_results = state_i.sol_results;
    % plot3(shape_i(1,:), shape_i(2,:), shape_i(3,:),'b');
    plotRobot(robot, sol_results.g)
end

% plot for i = 2
% y_all = sol_results.y_sol;
% s_all = sol_results.s_sol;
% n_all = y_all(:, 16:18)';
% 
% figure()
% hold on
% plot(s_all, n_all(1,:))
% plot(s_all, n_all(2,:))
% plot(s_all, n_all(3,:))

%% functions

    

