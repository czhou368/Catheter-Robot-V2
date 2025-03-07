function [ini_sol, results, res] = forward_kinematics(actuations, robot_parameters, E, G,  ...
    external_loads, ini_guess)
robot = creatRobotChang(robot_parameters, E, G);

loads.f1_body = external_loads(:, 1);    %  N/mm, the distributed self-weight.
loads.f4_body = external_loads(:, 2);    %  N/mm, the distributed self-weight.
loads.F_endPoint = external_loads(:, 3);   % N

% ini_guess = zeros(10,1);

[shape, ini_sol, res, results] = robot_disp_fk(robot, actuations, loads, ini_guess);
end

