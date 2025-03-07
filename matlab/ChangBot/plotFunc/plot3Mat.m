function name = plot3Mat(p, color, width, marker)

if nargin == 1
    color = 'b';
    width = 0.5;
    marker = 'none';
elseif nargin == 2
    width = 0.5;
    marker = 'none';
elseif nargin ==3
    marker = 'none';
end

if strcmp(marker, 'none')
    name = plot3(p(1,:), p(2,:), p(3,:), 'Color', color, 'LineWidth', width);
else
    name = plot3(p(1,:), p(2,:), p(3,:), 'Color', color, 'LineWidth', width, 'Marker',marker,'LineStyle','none');
end

end