function [J, C, ini0, fk_results] = computeJacobCompl(robot, actuation, loads, ini_guess)
% actuation is 1x4 vector
% only 3d position Jacoiban:
%  J: 3x4,
%  C: 3x3.

J = zeros(3, 4);
C = zeros(3, 3);

[shape0, ini0, res0, fk_results] = robot_disp_fk(robot, actuation, loads, ini_guess);

p0 = shape0(:, end);
F_load0 = loads.F_endPoint;

da = 5e-2;
dF = 1e-3;

for i = [1, 3]
    actuation_i = actuation;
    actuation_i(i) = actuation_i(i) + da;   % perturbance
    actuation_i(i+1) = actuation_i(i+1) - da;   % perturbance
    [shape_i, ~, ~, ~] = robot_disp_fk(robot, actuation_i, loads, ini0);
    p_i = shape_i(:, end);

    J(:, i) = (p_i - p0) / da;
    J(:, i+1) = - (p_i - p0) / da;
end

for j = 1:3
    load_i = loads;
    load_i.F_endPoint(j) = F_load0(j) + dF;
    [shape_i, ~, ~, ~] = robot_disp_fk(robot, actuation, load_i, ini0);
    p_i = shape_i(:, end);

    C(:, j) = (p_i - p0) / dF;
end

end