clear; clc

robot = creatRobotChang();

loads.f1_body = [0; 0; 0];    %  N/mm, the distributed self-weight.
loads.f4_body = [0; 0; 0];    %  N/mm, the distributed self-weight.
loads.F_endPoint = [0, 0, 0]';   % N

% actuation = [5 -5 5 -5];

ini_guess = zeros(6,1);
ini_sol = ini_guess; 

state_traj = [];

for dT = 1

    % actuation_i =  [60 100 0 0];
    actuation_i =  [100 0 0 0];
    
    [shape, ini_sol, res, results] = fk_shooting(robot, actuation_i, loads, ini_sol);

    state_i.shape = shape;
    state_i.actuation = actuation_i;
    state_i.ini = ini_sol;
    state_i.sol_results = results;
    state_i.res = res;

    % calculate the integrated length

    state_traj = [state_traj, state_i];
end


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
    sol_results = state_i.sol_results;
    % plot3(shape_i(1,:), shape_i(2,:), shape_i(3,:),'b');

    % calc. the integrated length & shorten length
    [len_tendons, disp_tendons] = getTendonLen(robot, sol_results.p, sol_results.R);

    len_tendons
    disp_tendons

    % plot
    plotRobot(robot, sol_results.g)
end

% % plot for i = 2
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

    
function [shape, ini10_sol, res, results] = fk_shooting(robot, actuation, loads, ini_guess)
    function ys = ode_T(s, y, tension, f_body, ri, Ki)
        R = reshape(y(4:12), 3, 3);
        u = y(13:15);
        n = y(16:18);
        % tension = y(19:22);
        v = [0;0;1];

        A = 0; B = 0; G = 0; H = 0; a = 0; b = 0;
        Ai = cell(1,2);
        Bi = cell(1,2);
        ai = cell(1,2);
        bi = cell(1,2);
        dpi_b = cell(1,2);
        
        for i = 1:2
            dpi_b{i} = hat(u) * ri{i} + v;
            Ai{i} = - tension(i) * (hat(dpi_b{i}))^2 / ((norm(dpi_b{i}))^3);
            Bi{i} = hat(ri{i}) * Ai{i};
            A = A + Ai{i};
            B = B + Bi{i};
            G = G - Ai{i} * hat(ri{i});
            H = H - Bi{i} * hat(ri{i});
            ai{i} = Ai{i} * (hat(u)*dpi_b{i});
            bi{i} = hat(ri{i}) * ai{i};
            a = a + ai{i};
            b = b + bi{i};
        end
        
        % formulate the diff equation
        le = zeros(3,1);  % ?? DOUBLE CHECK!!!
        du = inv(H + Ki) * (- hat(u)*Ki*(u-robot.u_star) - ...
                              hat(v)*R'*n - R'*le - b);
        dn = - R * (a + G * du) - f_body ;
        ys = [R * v;
              reshape(R * hat(u), 9, 1);
              du;
              dn];
    end

    function [y2, m2, n2] = state_trans(y1, tension, ri, K1, K2)

        % everything below is at s=L (end of the rod)
        p1 = y1(end, 1:3)';
        R1 = reshape(y1(end, 4:12),3,3);
        u1 = reshape(y1(end, 13:15),3,1);
        n1 = reshape(y1(end, 16:18),3,1);
        v = [0;0;1];
        
        m1 = K1 * R1 * (u1 - robot.u_star);
        Ft = zeros(3,1); 
        Mt = zeros(3,1);
        dpi = cell(1,2);

        for i = 1:2           
           dpi{i} = R1 * (hat(u1)*ri{i} + v);
           Ft = Ft + tension(i) * dpi{i} / norm(dpi{i});
           Mt = Mt + tension(i) * hat(R1 * ri{i}) * dpi{i} / norm(dpi{i});
        end

        p2 = p1; R2 = R1;
        n2 = n1 + Ft;
        m2 = m1 + Mt;
        u2 = R2'*inv(K2)*m2 + robot.u_star;

        y2 = [p2; reshape(R2,9,1); u2; n2];

    end

    function [p, R, g] = getShape(y_cell)
        % y is nx12, get the shape
        segs = length(y_cell);

        if nargout == 1
            p = cell(1, segs);
            for i = 1:segs
                y = y_cell{i};
                p{i} = y(:,1:3)';
            end

        elseif nargout == 2
            p = cell(1, segs);
            R = cell(1, segs);
            for i = 1:segs
                y = y_cell{i};
                ny = size(y,1);
                p{i} = y(:,1:3)';
                R{i} = reshape(y(:,4:12)', 3, 3, ny);
            end

        elseif nargout == 3
            p = cell(1, segs);
            R = cell(1, segs);
            g = cell(1, segs);
            for i = 1:segs
                y = y_cell{i};
                ny = size(y,1);

                p{i} = y(:,1:3)';
                R{i} = reshape(y(:,4:12)', 3, 3, ny);
                pp = reshape(p{i}, 3, 1, ny);
                base = repmat([0 0 0 1], [1,1,ny]);
                g{i} = cat(1, cat(2, R{i}, pp), base);
            end
        end
    end


    function [res, shape, results] = residual_disp(ini6)
        ini_18 = [robot.p0; reshape(robot.R0, 9,1); ini6(1:6)];
        tension1 = actuation(1:2);
        tension2 = actuation(3:4);

        % define different parts
        ode_disp1 = @(s, y) ode_T(s, y, tension1+tension2, loads.f1_body, robot.ri1, robot.K1);
        ode_disp2 = @(s, y) ode_T(s, y, tension1+tension2, loads.f1_body, robot.ri1, robot.K2);
        ode_disp3 = @(s, y) ode_T(s, y, tension1+tension2, loads.f1_body, robot.ri1, robot.K3);
        ode_disp4 = @(s, y) ode_T(s, y, tension2, loads.f4_body, robot.ri4, robot.K4);
        ode_disp5 = @(s, y) ode_T(s, y, tension2, loads.f4_body, robot.ri4, robot.K5);

        % solve ivps
        s1_end = robot.l1;
        s2_end = robot.l1+robot.l2;
        s3_end = robot.l1+robot.l2+robot.l3;
        s4_end = robot.l1+robot.l2+robot.l3+robot.l4;
        s5_end = robot.l1+robot.l2+robot.l3+robot.l4+robot.l5;

        [s1_sol, y1_sol] = ode45(ode_disp1, [0, s1_end], ini_18); 

        y2_ini = state_trans(y1_sol(end, :), [0 0], robot.ri1, robot.K1, robot.K2);
        [s2_sol, y2_sol] = ode45(ode_disp2, [s1_end, s2_end], y2_ini);

        y3_ini = state_trans(y2_sol(end, :), [0 0], robot.ri1, robot.K2, robot.K3);
        [s3_sol, y3_sol] = ode45(ode_disp3, [s2_end, s3_end], y3_ini);

        y4_ini = state_trans(y3_sol(end, :), tension1, robot.ri1, robot.K3, robot.K4);
        [s4_sol, y4_sol] = ode45(ode_disp4, [s3_end, s4_end], y4_ini);

        y5_ini = state_trans(y4_sol(end, :), [0 0], robot.ri4, robot.K4, robot.K5);
        [s5_sol, y5_sol] = ode45(ode_disp5, [s4_end, s5_end], y5_ini);
        
        % force/moment at end of the rod
        [~,m_end,n_end] = state_trans(y5_sol(end, :), tension2, robot.ri4, robot.K5, robot.K5);
        
        res = [n_end - loads.F_endPoint; 
               m_end - 0];

        if nargout > 1
            cell_all = {y1_sol, y2_sol, y3_sol, y4_sol, y5_sol};
            [results.p, results.R, results.g] = getShape(cell_all);
            results.y_sol = [y1_sol; y2_sol; y3_sol; y4_sol; y5_sol];
            results.s_sol = [s1_sol; s2_sol; s3_sol; s4_sol; s5_sol];
            shape = results.y_sol(:,1:3)';
        end
    end


    options = optimoptions('fsolve', 'OptimalityTolerance', 1e-12, ...
        'Display', 'iter', ...
        'MaxFunctionEvaluations', 5e3, ...
        'Algorithm','levenberg-marquardt');
    ini10_sol = fsolve(@residual_disp, ini_guess, options);

    [res, shape, results] = residual_disp(ini10_sol);

end


function [len_tendons, disp_tendons] = getTendonLen(robot, p, R)

pc_total = zeros(1, 2);
pn_total = zeros(1, 2);

pc_c = [p{1}, p{2}, p{3}];
pc_n = [p{1}, p{2}, p{3}, p{4}, p{5}];
Rc_c = cat(3, R{1}, R{2}, R{3});
Rc_n = cat(3,R{1}, R{2}, R{3}, R{4}, R{5});

n_c = size(pc_c, 2);
n_n = size(pc_n, 2);

Lc = robot.l1 + robot.l2 + robot.l3;
Ln = robot.l1 + robot.l2 + robot.l3 + robot.l4 + robot.l5;

for j = 1:2
    pc_i = pc_c + reshape(pagemtimes(Rc_c, robot.ri1{j}), 3, n_c);
    pn_i = pc_n + reshape(pagemtimes(Rc_n, robot.ri4{j}), 3, n_n);

    pc_total(j) = sum(vecnorm(diff(pc_i,1,2), 2, 1));
    pn_total(j) = sum(vecnorm(diff(pn_i,1,2), 2, 1));
end

len_tendons = [pc_total, pn_total];
disp_tendons = [Lc - pc_total, Ln - pn_total];
% disp_tendons = [pc_total - Lc, pn_total - Ln];

end
