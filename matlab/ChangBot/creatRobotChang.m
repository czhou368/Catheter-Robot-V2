function robot = creatRobotChang(robot_parameters, E, G)
% robot_parameters: [l1, l2, l3, l4, l5, dc_out, dc_in, dn_out, dn_in]
% unit: N, mm. All size needs calibration

robot.p0 = [0;0;0];
robot.R0 = eye(3);

% cathetor size
robot.dc_out = robot_parameters(6);
robot.dc_in = robot_parameters(7);
robot.l1 = robot_parameters(1);          % cathetor channel part (partially rigid)
robot.l2 = robot_parameters(2);           % bending part (compliant)
robot.l3 = robot_parameters(3);            % straight part (rigid)

% needle size
robot.dn_out = robot_parameters(8);
robot.dn_in = robot_parameters(9);
robot.l4 = robot_parameters(4);          % notched needle (compliant)
robot.l5 = robot_parameters(5);            % straight end (rigid)

robot.L = sum([robot.l1,robot.l2,robot.l3,robot.l4,robot.l5]);

% Young's Modules & torsional stiffness
E1 = E(1);                % N/mm^2, cathetor, frist part
E2 = E(2);                % N/mm^2, cathetor, bending part
E3 = E(3);               % N/mm^2, notched needle
E4x = E(4);                % N/mm^2, notched needle
E4y = E(5);
E5 = E(6);              % N/mm^2, notched needle

G1 = G(1);              % N/mm^2, cathetor, frist part (no torsion)
G2 = G(2);              % N/mm^2, cathetor, bending part (no torsion)
G3 = G(3);              % N/mm^2, cathetor, bending part (no torsion)
G4 = G(4);              % N/mm^2, cathetor, bending part (no torsion)
G5 = G(5);              % N/mm^2, notched needle

% tendon diam
d_wire = 0.5;

I1 = pi/64*(robot.dc_out^4-robot.dc_in^4);
J1 = 2*I1;
I4 = pi/64*(robot.dn_out^4-robot.dn_in^4);
J4 = 2*I4;

% robot bending stiffness
robot.K1 = diag([E1*I1,E1*I1,G1*J1]);   % N*mm^2
robot.K2 = diag([E2*I1,E2*I1,G2*J1]);   % N*mm^2
robot.K3 = diag([E3*I1,E3*I1,G3*J1]);   % N*mm^2
robot.K4 = diag([E4x*I4,E4y*I4,G4*J4]);   % N*mm^2
robot.K5 = diag([E5*I4,E5*I4,G5*J4]);   % N*mm^2

r1x = [1 0 0]';
r1y = [-1 0 0]';
r4x = [0 1 0]';
r4y = [0 -1 0]';
% r4x = [1 0 0]';
% r4y = [-1 0 0]';

radii1 = 1/4*(robot.dc_out + robot.dc_in);  % radii of cathetor
radii4 = 1/4*(robot.dn_out + robot.dn_in);  % radii of needle

robot.ri1 = {radii1*r1x, radii1*r1y};   % ri{3} quote r3
robot.ri4 = {radii4*r4x, radii4*r4y};   % ri{3} quote r3
robot.ri14 = {radii1*r1x, radii1*r1y, radii4*r4x, radii4*r4y};   % ri{3} quote r3

robot.u_star = [0;0;0];   % straight line

end