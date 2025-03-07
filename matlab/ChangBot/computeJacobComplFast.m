function [J, C, E, B, res0, fk_results] = computeJacobComplFast(robot, actuation, loads, ini_sol)
% actuation is 1x4 vector, ini is 10x1
% only 3d position Jacoiban:
%  J: 3x4,
%  C: 3x3.

J = zeros(3, 4);
C = zeros(3, 3);

% E = [Eq, Ew, Eu]; 
Eq = zeros(3, 4);   
Ew = zeros(3, 3);   
Eu = zeros(3, 10);   

% B = [Bq, Bw, Bu];
Bq = zeros(10, 4);   
Bw = zeros(10, 3);   
Bu = zeros(10, 10);    

[shape0, ~, res0, fk_results] = robot_disp_fk(robot, actuation, loads, ini_sol, 'ivp');

p0 = shape0(:, end);
F_load0 = loads.F_endPoint;

% da = 5e-2;
% dF = 1e-3;
da = 1e-5;
dF = 1e-5;
du = 1e-5;

for i = [1, 3]
    actuation_i = actuation;
    actuation_i(i) = actuation_i(i) + da;   % perturbance
    actuation_i(i+1) = actuation_i(i+1) - da;   % perturbance
    [shape_i, ~, res_i, ~,] = robot_disp_fk(robot, actuation_i, loads, ini_sol, 'ivp');
    p_i = shape_i(:, end);

    Eq(:, i) = (p_i - p0) / da;
    Eq(:, i+1) = - (p_i - p0) / da;

    Bq(:, i) = (res_i - res0) / da;
    Bq(:, i+1) = - (res_i - res0) / da;

end

for j = 1:3
    load_i = loads;
    load_i.F_endPoint(j) = F_load0(j) + dF;
    [shape_i, ~, res_i, ~] = robot_disp_fk(robot, actuation, load_i, ini_sol, 'ivp');
    p_i = shape_i(:, end);

    Ew(:, j) = (p_i - p0) / dF;

    Bw(:, j) = (res_i - res0) / dF;
end

for k = 1:10
    ini_i = ini_sol;
    ini_i(k) = ini_i(k) + du;
    [shape_i, ~, res_i, ~] = robot_disp_fk(robot, actuation, loads, ini_i, 'ivp');
    p_i = shape_i(:, end);

    Eu(:, k) = (p_i - p0) / du;

    Bu(:, k) = (res_i - res0) / du;

end

% calculate final J, C, (Calib Rucker's paper)
E = [Eq, Ew, Eu]; 
B = [Bq, Bw, Bu];

% J = Eq - Eu * (Bu \ Bq);
% C = Ew - Eu * (Bu \ Bw);
sig_tol = 1e-5;
J = Eq - Eu * (pinv(Bu, sig_tol) * Bq);
C = Ew - Eu * (pinv(Bu, sig_tol) * Bw);


end





