clear; clc

%% define variables

% global ri u_star du_star p0 R0_vec L dri ddri
% global K_BT_p K_BT_c

robot.p0 = [0;0;0];
robot.R0 = eye(3);

radii = 15.06;   % mm, radii

% r1 = [1 0 0]';
% r2 = [0 1 0]';
% r3 = [-1 0 0]';
% r4 = [0 -1 0]'; 
% robot.ri = {radii*r1, radii*r2, radii*r3, radii*r4};   % ri{3} quote r3

r1 = [1 0 0]';
r2 = [-1 0 0]';
robot.ri = {radii*r1, radii*r2};   % ri{3} quote r3
robot.u_star = [0;0;0];   % straight line
robot.L = 500;  %mm, length
robot.d = 2;  % mm, diameter

I = pi / 64 * robot.d^4;  % mm^4
J = 2 * I;
nu = 0.3125;
E_coupled = 200e3;  % N/mm^2
G_coupled = 80e3;

robot.K_BT_c = diag([E_coupled * I, E_coupled * I, G_coupled * J]);   % N*mm^2

% robot.fe = [-0.47; 0; 0];    % 0.47 N/m, the self-weight of distributed force.

%% calc shape

% loads.f_body = [-0.47; 0; 0];    % 0.47 N/m, the self-weight of distributed force.
% loads.F_endPoint = [-0.148, 0, 0]'; % [0, 0, 0]';  % 

loads.f_body = [0; 0; 0];    % 0.47 N/m, the self-weight of distributed force.
loads.F_endPoint = [0, 0, 0]'; % [0, 0, 0]';  % 

actuation = [5 -5];

ini_guess = zeros(8,1);
ini_sol = ini_guess; 

state_traj = [];

for ds = 10
    actuation_i =  ds * [1 -1];
    
    [shape, g_array, ini_sol, res] = fk_shooting(robot, actuation_i, loads, ini_sol);

    tension_sol = ini_sol(7:8);

    state_i.shape = shape;
    state_i.actuation = actuation_i;
    state_i.ini = ini_sol;
    state_i.res = res;
    state_i.tension = tension_sol;

    state_traj = [state_traj, state_i];
end

%% plot

figure()
hold on

xlabel('x(mm)'); ylabel('y(mm)'); zlabel('z(mm)');
axis equal
zlim([-10, 500]); xlim([-50, 250]); ylim([-50 50]);
% zlim([-0.2, 0.2]); xlim([-0.15, 0.15]); ylim([0, 0.25]);
title('Figure 4.8')
view(45,10)
grid on; 

n = length(state_traj);

tension_traj = zeros(1, n);

for i=1:n
    state_i = state_traj(i);
    shape_i = state_i.shape;
    tension_traj(i) = state_i.tension(1) - state_i.tension(2);
    state_i.res
    plot3(shape_i(:, 1), shape_i(:, 2), shape_i(:, 3),'b');
end

% figure()
% plot(tension_traj)

%% functions

    
function [shape, g_array, ini8_sol, res] = fk_shooting(robot, actuation, loads, ini_guess)
    function ys = ode_disp(s, y, tension)
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
            dpi_b{i} = hat(u) * robot.ri{i} + v;
            Ai{i} = - tension(i) * (hat(dpi_b{i}))^2 / ((norm(dpi_b{i}))^3);
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
    
    function res = residual_disp(ini8)
        ini_18 = [robot.p0; reshape(robot.R0, 9,1); ini8(1:6)];
        tension = ini8(7:8);
        ode_toSol = @(s, y) ode_disp(s, y, tension);
        [s_sol, y_sol] = ode45(ode_toSol, [0, robot.L], ini_18); 

        % everything below is at s=L (end of the rod)
        p = y_sol(end, 1:3);
        R = reshape(y_sol(end, 4:12),3,3);
        u = reshape(y_sol(end, 13:15),3,1);
        n = reshape(y_sol(end, 16:18),3,1);
        v = [0;0;1];
        
        m = robot.K_BT_c * R * (u - robot.u_star);
        F_ex = n; 
        M_ex = m;
        p_total = zeros(2, 1);
        dpi = cell(1,2);

        pc = y_sol(:, 1:3)';
        ns = length(s_sol);
        Rc = reshape(y_sol(:, 4:12)', 3, 3, ns);

        for i = 1:2           
           pi_i = pc + reshape(pagemtimes(Rc, robot.ri{i}), 3, ns);
           p_total(i) = sum(vecnorm(diff(pi_i,1,2), 2, 1)); 
           dpi{i} = R * (hat(u)*robot.ri{i} + v);
           F_ex = F_ex + tension(i) * dpi{i} / norm(dpi{i});
           M_ex = M_ex + tension(i) * hat(R * robot.ri{i}) * dpi{i} / norm(dpi{i});
        end

        res = [F_ex - loads.F_endPoint; 
               M_ex - 0;
               p_total - (robot.L - actuation')];
    end

    options = optimoptions('fsolve', 'OptimalityTolerance', 1e-12,'Display', 'iter', ...
        'Algorithm','levenberg-marquardt');
    [ini8_sol, res] = fsolve(@residual_disp, ini_guess, options);
    ini6_sol = reshape(ini8_sol(1:6), 6, 1);
    ini_18_c = [robot.p0; reshape(robot.R0,9,1); ini6_sol];

    tension_sol = ini8_sol(7:8);
    ode_sol = @(s, y) ode_disp(s, y, tension_sol);
    [s_sol, y_sol] = ode45(ode_sol, [0, robot.L], ini_18_c);
    shape = y_sol(:, 1:3);
    g_array = [];    % to be changed !!

end

