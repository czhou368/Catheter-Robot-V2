function hs = plotRobot(robot, g_arrays, trans)

if nargin == 2
    trans = 1;
end

color_c1 = 0.1*[1 1 1];
color_c2 = 0.4*[1 1 1];
color_s1 = [0 0 0];
color_s2 = [0 0 0];
color_n = 0.7* [1 1 1];

seg1 = plot3DTubes(g_arrays{1}, robot.dc_in/2, robot.dc_out/2, trans, color_c1);
seg2 = plot3DTubes(g_arrays{2}, robot.dc_in/2, robot.dc_out/2, trans, color_c2);
seg3 = plot3DTubes(g_arrays{3}, robot.dc_in/2, robot.dc_out/2, trans, color_s1);
seg4 = plot3DTubes(g_arrays{4}, robot.dn_in/2, robot.dn_out/2, trans, color_n);
seg5 = plot3DTubes(g_arrays{5}, robot.dn_in/2, robot.dn_out/2, trans, color_s2);

hs = [seg1, seg2, seg3, seg4, seg5];

end