function name = plot3DTubes(g, rin, rout, alpha, color)
% g: frames
% T0: Plot frame, g is considered to be expressed in T0

%% Initialization & Check inputs
if nargin < 4
    alpha = 1;
end

if nargin < 5
    color = [1,0,1];
end

T0 = eye(4);

res = size(g,3);
for i = 1:res
    g(:,:,i) = T0*g(:,:,i);
end

%% Compute bishop frames
for i = 2:res
    w = LogSO3(g(1:3,1:3,i-1)'*g(1:3,1:3,i));
    g(1:3,1:3,i) = g(1:3,1:3,i)*LargeSO3([0,0,-w(3)]);
end

%% Build meshs
m = 51; % points on cross sectional circle

X = zeros(2*res+1, m);
Y = zeros(2*res+1, m);
Z = zeros(2*res+1, m);

t = linspace(0,2*pi,m);
outCircle = [rout*cos(t); rout*sin(t); zeros(1,m);ones(1,m)];
inCircle = [rin*cos(t); rin*sin(t); zeros(1,m);ones(1,m)];
for i = 1:res
    outCross = g(:,:,i)*outCircle;
    inCross = g(:,:,i)*inCircle;
    
    X(i,:) = outCross(1,:);
    Y(i,:) = outCross(2,:);
    Z(i,:) = outCross(3,:);
    
    X(end-i,:) = inCross(1,:);
    Y(end-i,:) = inCross(2,:);
    Z(end-i,:) = inCross(3,:);
end
X(end,:) = X(1,:);
Y(end,:) = Y(1,:);
Z(end,:) = Z(1,:);

%% Plot
% C = [-ones(res-1,m); zeros(1,m); ones(res-1,m); zeros(1,m)];
name = surf(X,Y,Z, 'MeshStyle', 'both', 'LineStyle', 'none', ...
    'FaceAlpha', alpha, 'FaceColor', color);

% colors = [0.8;0.5;0.2]*reshape(color,1,3);
% colors = [colors; 0.7,0.7,0.7];
% colormap(colors);
    



end

