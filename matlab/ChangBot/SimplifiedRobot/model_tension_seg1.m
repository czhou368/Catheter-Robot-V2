clear; clc

%% define variables

% global ri u_star du_star p0 R0_vec L dri ddri
% global K_BT_p K_BT_c

robot.p0 = [0;0;0];
robot.R0 = eye(3);

r1 = 1e-3 * [8, 0, 0]';
r2 = 1e-3 * [0, 8, 0]';
r3 = 1e-3 * [-8, 0, 0]';
r4 = 1e-3 * [0, -8, 0]'; 

robot.ri = {r1, r2, r3, r4};   % ri{3} quote r3
robot.u_star = [0;0;0];   % straight line
robot.L = 242e-3;  %m, length
robot.d = 0.8e-3;  % m, diameter

I = pi / 64 * robot.d^4;
J = 2 * I;
nu = 0.3125;
E_coupled = 229.6e9;  % Gpa
G_coupled = 1/2 * E_coupled / (1 + nu);

robot.K_BT_c = diag([E_coupled * I, E_coupled * I, G_coupled * J]);

% robot.fe = [-0.47; 0; 0];    % 0.47 N/m, the self-weight of distributed force.

%% calc shape

loads.f_body = [-0.47; 0; 0];    % 0.47 N/m, the self-weight of distributed force.
loads.F_endPoint = [-0.148, 0, 0]'; % [0, 0, 0]';  % 
cancel = 7.25;
loads.tension = [0, cancel, 0, cancel+6.38];

ini_guess = zeros(6,1);

[shape, g_array, ini6_sol, res] = fk_shooting(robot, loads, ini_guess);

figure()
plot3(shape(:, 2), shape(:, 3), shape(:, 1),'b');

xlabel('y(m)'); ylabel('z(m)'); zlabel('x(m)');
zlim([-0.2, 0.0]); xlim([-0.15, 0.15]); ylim([0, 0.25]);
title('Figure 4.8')
view(155,10)
grid on; 

function [shape, g_array, ini6_sol, res] = fk_shooting(robot, loads, ini_guess)
    function ys = ode_straight_coupled(s, y)
        R = reshape(y(4:12), 3, 3);
        u = y(13:15);
        n = y(16:18);
        v = [0;0;1];

        A = 0; B = 0; G = 0; H = 0; a = 0; b = 0;
        Ai = cell(1,4);
        Bi = cell(1,4);
        ai = cell(1,4);
        bi = cell(1,4);
        dpi_b = cell(1,4);
        
        for i = [1, 2, 3, 4]
            dpi_b{i} = hat(u) * robot.ri{i} + v;
            Ai{i} = - loads.tension(i) * (hat(dpi_b{i}))^2 / ((norm(dpi_b{i}))^3);
            Bi{i} = hat(robot.ri{i}) * Ai{i};
            A = A + Ai{i};
            B = B + Bi{i};
            G = G - Ai{i} * hat(robot.ri{i});
            H = H - Bi{i} * hat(robot.ri{i});
            ai{i} = Ai{i} * (hat(u)*dpi_b{i});
            bi{i} = hat(robot.ri{i}) * ai{i};
            a = a + ai{i};
            b = b + bi{i};
        end
        
        % formulate the diff equation
        le = zeros(3,1);  % ?? DOUBLE CHECK!!!
        du = inv(H + robot.K_BT_c) * (- hat(u)*robot.K_BT_c*(u-robot.u_star) - ...
                              hat(v)*R'*n - R'*le - b);
        dn = - R * (a + G * du) - loads.f_body ;
        ys = [R * v;
              reshape(R * hat(u), 9, 1);
              du;
              dn];
    end
    
    function res = residual_straight_coupled(ini_6)
        ini_18 = [robot.p0; reshape(robot.R0, 9,1); reshape(ini_6, 6, 1)];
        [s_sol, y_sol] = ode45(@ode_straight_coupled, [0, robot.L], ini_18); 
        % everything below is at s=L (end of the rod)
        p = y_sol(end, 1:3);
        R = reshape(y_sol(end, 4:12),3,3);
        u = reshape(y_sol(end, 13:15),3,1);
        n = reshape(y_sol(end, 16:18),3,1);
        v = [0;0;1];
        
        m = robot.K_BT_c * R * (u - robot.u_star);
        F_ex = n; 
        M_ex = m;
        dpi = cell(1,4);
        for i = [1,2,3,4]
           dpi{i} = R * (hat(u)*robot.ri{i} + v);
           F_ex = F_ex + loads.tension(i) * dpi{i} / norm(dpi{i});
           M_ex = M_ex + loads.tension(i) * hat(R * robot.ri{i}) * dpi{i} / norm(dpi{i});
        end

        res = [F_ex - loads.F_endPoint; M_ex - 0];
    end

    [ini6_sol, res] = fsolve(@residual_straight_coupled, ini_guess);
    ini_18_c = [robot.p0; reshape(robot.R0,9,1); reshape(ini6_sol,6,1)];
    [s_sol, y_sol] = ode45(@ode_straight_coupled, [0, robot.L], ini_18_c);
    shape = y_sol(:, 1:3);
    g_array = [];    % to be changed !!

end

