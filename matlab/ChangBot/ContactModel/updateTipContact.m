function [state_update,contactFlag] = updateTipContact(robot, obs, q, state0)
% solve the updated shape if the tip is in contact with obstacle.

    shape_prev = state0.shape;
    loads_prev = state0.loads;
    ini_prev = state0.ini;
    q_prev = state0.q;

    F_prev = loads_prev.F_endPoint;

    % using the previous Jacobian
    % [J, C, ~, ~] = computeJacobCompl(robot, q_prev, loads_prev, ini_prev);
    % [~, ini_sol, ~, ~] = robot_disp_fk(robot, q_prev, loads_prev, ini_prev);
    [J, C, E, B, ~, ~] = computeJacobComplFast(robot, q_prev, loads_prev, ini_prev);

    Jc = J(1:3,:);
    Cc = C(1:3, 1:3);   % Cc = dp/df

    % geometry
    p_tip = shape_prev(:, end);
    n = obs.getNormal(p_tip);
    mu = obs.mu;

    d = 60;   % tunable
    theta = linspace(0, pi, round(d/2)+1);
    theta = theta(1:end-1);
    cs = [cos(theta); sin(theta)];
    nullVec = null(n');
    D = [nullVec*cs, -nullVec*cs];
    e = ones(d,1);

    % fixed the penetration/vibration
    p_proj = obs.project(p_tip);

    % account for environment move !!
    if size(obs.T_history, 3) >= 2
        T_curr = obs.T_history(:,:,end);
        T_prev = obs.T_history(:,:,end-1);
        % pc_prev_i = contacts_prev(ic).point;
        p_proj_move4x1 = T_curr * inv(T_prev) * [p_proj; 1];
        p_proj_move = p_proj_move4x1(1:3);
    else
        % no obstacle movement
        p_proj_move = p_proj;
    end

    % final b: obstacle move & penetration correction.
    b = p_tip - p_proj_move;

    h = Jc*(q'-q_prev') - Cc*F_prev+ b;

    % LCP
    M = [n'*Cc*n, n'*Cc*D, 0;
         D'*Cc*n, D'*Cc*D, e;
         mu,      -e',     0];

    g = [n'*h; D'*h; 0];
    
    % solve scaled LCP
    sv = svd(Cc);
    scale = max(sv);
    dimM = size(M,1);

    Dw = diag([ones(1, dimM-1), scale]);
    Dx = diag([ones(1, dimM-1)/scale, 1]);

    M2 = Dw * M * Dx;
    g2 = Dw * g;

    [w2, x2, retcode2] = LCPSolve(M2, g2, 1e-8, dimM^2);
    test_w2 = M2*x2+g2;
    x_sol = Dx * x2;

    fn = x_sol(1);
    beta = x_sol(2:d+1);

    % calculate the update state
    F_sol = n*fn + D*beta;

    loads_sol = loads_prev;
    loads_sol.F_endPoint = F_sol;

    % solve the bvp using new w
    % [~, p, yu0, b_res, shape] = three_tube_fk(ctr, q, w, yu0_prev);
    % [shape_sol, ini_sol, ~, fk_results_sol] = robot_disp_fk(robot, q, loads_sol, ini_prev);

    % solve the fk using ivp forward. Check out Calib Rucker's paper
    Bq = B(:, 1:4);  Bw = B(:, 5:7); Bu = B(:, 8:17);
    Bu_inv = pinv(Bu, 1e-5);
    du_sol = - Bu_inv * (Bq*(q'-q_prev') + Bw*(F_sol-F_prev));
    ini_sol = ini_prev + du_sol;
    [shape_sol, ~, ~, fk_results_sol] = robot_disp_fk(robot, q, loads_sol, ini_sol, 'ivp');
    
    state_update.shape = shape_sol;
    state_update.loads = loads_sol;
    state_update.fk_results = fk_results_sol;
    state_update.ini = ini_sol;
    state_update.q = q;

    % check the acc of compliance
    % dp = Jc*(q-q_prev) + Cc*(f-f_prev);
    % dp_num = p{3}(end,:)' - p_tip;

    % check if the contact is maintained.
    if fn > 1e-12
        contactFlag = true;
    else 
        contactFlag = false;
    end


end