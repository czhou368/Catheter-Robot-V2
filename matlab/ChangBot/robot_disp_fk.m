function [shape, ini10_sol, res, results] = robot_disp_fk(robot, actuation, loads, ini_guess, type)
    function ys = ode_T(s, y, tension, f_body, ri, Ki)
        R = reshape(y(4:12), 3, 3);
        u = y(13:15);
        n = y(16:18);
        % tension = y(19:22);
        v = [0;0;1];

        nt = length(ri);  % number of tendons

        A = 0; B = 0; G = 0; H = 0; a = 0; b = 0;
        Ai = cell(1,nt);
        Bi = cell(1,nt);
        ai = cell(1,nt);
        bi = cell(1,nt);
        dpi_b = cell(1,nt);
        
        for i = 1:nt
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

        nt = length(ri);   % number of tendons
        
        m1 = K1 * R1 * (u1 - robot.u_star);
        Ft = zeros(3,1); 
        Mt = zeros(3,1);
        dpi = cell(1,nt);

        for i = 1:nt           
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

    function [res, shape, results] = residual_disp(ini10)
        ini_18 = [robot.p0; reshape(robot.R0, 9,1); ini10(1:6)];
        tension = ini10(7:10);
        tension1 = ini10(7:8);
        tension2 = ini10(9:10);

        % define different parts
        ode_disp1 = @(s, y) ode_T(s, y, tension, loads.f1_body, robot.ri14, robot.K1);
        ode_disp2 = @(s, y) ode_T(s, y, tension, loads.f1_body, robot.ri14, robot.K2);
        ode_disp3 = @(s, y) ode_T(s, y, tension, loads.f1_body, robot.ri14, robot.K3);
        ode_disp4 = @(s, y) ode_T(s, y, tension2, loads.f4_body, robot.ri4, robot.K4);
        ode_disp5 = @(s, y) ode_T(s, y, tension2, loads.f4_body, robot.ri4, robot.K5);

        % solve ivps
        s1_end = robot.l1;
        s2_end = robot.l1+robot.l2;
        s3_end = robot.l1+robot.l2+robot.l3;
        s4_end = robot.l1+robot.l2+robot.l3+robot.l4;
        s5_end = robot.l1+robot.l2+robot.l3+robot.l4+robot.l5;

        [s1_sol, y1_sol] = ode45(ode_disp1, [0, s1_end], ini_18); 

        y2_ini = state_trans(y1_sol(end, :), [0 0 0 0], robot.ri14, robot.K1, robot.K2);
        [s2_sol, y2_sol] = ode45(ode_disp2, [s1_end, s2_end], y2_ini);

        y3_ini = state_trans(y2_sol(end, :), [0 0 0 0], robot.ri14, robot.K2, robot.K3);
        [s3_sol, y3_sol] = ode45(ode_disp3, [s2_end, s3_end], y3_ini);

        y4_ini = state_trans(y3_sol(end, :), tension1, robot.ri1, robot.K3, robot.K4);
        [s4_sol, y4_sol] = ode45(ode_disp4, [s3_end, s4_end], y4_ini);

        y5_ini = state_trans(y4_sol(end, :), [0 0], robot.ri4, robot.K4, robot.K5);
        [s5_sol, y5_sol] = ode45(ode_disp5, [s4_end, s5_end], y5_ini);
        
        % force/moment at end of the rod
        [~,m_end,n_end] = state_trans(y5_sol(end, :), tension2, robot.ri4, robot.K5, robot.K5);
   
        % length agree
        pc_total = zeros(2, 1);
        pn_total = zeros(2, 1);

        y_sol_c = [y1_sol; y2_sol; y3_sol];
        % y_sol_n = [y4_sol; y5_sol];
        y_sol_n = [y1_sol; y2_sol; y3_sol; y4_sol; y5_sol];

        n_c = size(y_sol_c, 1);
        n_n = size(y_sol_n, 1);
        pc_c = y_sol_c(:, 1:3)';
        pc_n = y_sol_n(:, 1:3)';
        Rc_c = reshape(y_sol_c(:, 4:12)', 3, 3, n_c);
        Rc_n = reshape(y_sol_n(:, 4:12)', 3, 3, n_n);

        % original length
        Lc = robot.l1 + robot.l2 + robot.l3;
        % Ln = robot.l4 + robot.l5;
        Ln = robot.l1 + robot.l2 + robot.l3 + robot.l4 + robot.l5;

        for j = 1:2
            pc_i = pc_c + reshape(pagemtimes(Rc_c, robot.ri1{j}), 3, n_c);
            pn_i = pc_n + reshape(pagemtimes(Rc_n, robot.ri4{j}), 3, n_n);

            pc_total(j) = sum(vecnorm(diff(pc_i,1,2), 2, 1));
            pn_total(j) = sum(vecnorm(diff(pn_i,1,2), 2, 1));
        end

        res = [n_end - loads.F_endPoint; 
               m_end - 0;
               pc_total - (Lc - actuation(1:2)'); 
               pn_total - (Ln - actuation(3:4)')];

        if nargout > 1
            cell_all = {y1_sol, y2_sol, y3_sol, y4_sol, y5_sol};
            [results.p, results.R, results.g] = getShape(cell_all);
            results.y_sol = [y1_sol; y2_sol; y3_sol; y4_sol; y5_sol];
            results.s_sol = [s1_sol; s2_sol; s3_sol; s4_sol; s5_sol];
            shape = results.y_sol(:,1:3)';
        end
    end

if nargin == 4
    type = 'bvp';
end

if strcmp(type, 'bvp')
    options = optimoptions('fsolve', 'FunctionTolerance', 1e-4, ...
        'Display', 'iter', ...
        'MaxFunctionEvaluations', 5e3, ...
        'StepTolerance', 1e-18, ...
        'Algorithm','levenberg-marquardt');
    ini10_sol = fsolve(@residual_disp, ini_guess, options);
    
    [res, shape, results] = residual_disp(ini10_sol);
elseif strcmp(type, 'ivp')
    % just pass the ivp forward.
    [res, shape, results] = residual_disp(ini_guess);
    ini10_sol = [];
end


end