classdef objPlane
   properties
      p0
      n
      mu
      T_history
      cornerFlag
   end
   methods
       function plane = objPlane(p0, n, mu)
           plane.p0 = p0;
           plane.n = n;
           plane.mu = mu;
           plane.cornerFlag = false;
           plane.T_history(:,:,1) = [eye(3), p0; 0 0 0 1];
       end

       function d = computeDepth(plane, p)
           d = - plane.n' * (p - plane.p0);
       end

       function plane = rebuild(plane)
           plane.p0 = plane.T_history(1:3,4,end);
       end

       function q = project(plane, p)
           % q = p projected onto plane
           q = p - plane.n*(plane.n'*(p-plane.p0));
       end

       function n_ret = getNormal(plane, p)
           np = size(p,2);
           n_ret = plane.n * ones(1,np);
       end

       function [X, Y, Z2] = getMesh(plane, radius, grids)
            if (nargin < 1)
                radius = 60;
            elseif nargin == 1
                grids = 100;
            end

            % plane basis
            % B = null(plane.n');

            width = 5;  % 5mm width plane.

            [X,Y,Z] = cylinder(radius,grids);
            Z2 = Z*width + plane.p0(3);

       end

       function [X, Y, Z, tick, circle] = getMeshClock(plane, R, r, angleOffset, grids)

           if nargin == 3
               grids = 12;
           end

           height = plane.p0(3);  % z value of plane

           angles = linspace(0,2*pi,grids+1) + angleOffset;
           angles_dense = linspace(0,2*pi,50) + angleOffset;

           tick.x = [R*cos(angles); r*cos(angles)];
           tick.y = [R*sin(angles); r*sin(angles)];
           tick.z = ones(size(tick.x))*height;

           circle.x = [R*cos(angles_dense)', r*cos(angles_dense)'];
           circle.y = [R*sin(angles_dense)', r*sin(angles_dense)'];
           circle.z = ones(size(circle.x))*height;

           width = 5;  % 5mm width plane.

           [X,Y,Zo] = cylinder(R,50);
           Z = Zo*width + height;

       end

       function hs = plotPlaneClock(plane, R, r, angleOffset, grids)

           if nargin == 3
               grids = 12;
           end

           [X, Y, Z, tick, circle] = plane.getMeshClock(R, r, angleOffset, grids);

           hp1 = surf(X,Y,Z, 'FaceColor', 'k',  'FaceAlpha', 0.3, 'EdgeColor', 'k', 'MeshStyle','row', 'EdgeAlpha',0.6, 'LineWidth',0.5);
           hp2 = fill3(X(1,:),Y(1,:),Z(1,:),'k','FaceAlpha', 0.3);
           hp3 = fill3(X(2,:),Y(2,:),Z(2,:),'k','FaceAlpha', 0.3);

           h_tick = plot3(tick.x,tick.y,tick.z, 'Color', 0.3*[1 1 1], 'lineWidth', 1);
           h_circle = plot3(circle.x,circle.y,circle.z, 'Color', 0.3*[1 1 1], 'lineWidth', 1);
       
           hs = [hp1, hp2, hp3, h_tick', h_circle'];
       end

       % function obsContact = detectContact(plane, tube, p, cornerRange)
       %
       %     % p should be 3xn: s=0-L
       %     obsContact = [];
       %
       %     % Find surface contacts
        % 
        %     d = plane.computeDepth(p);
        %     IdxBodySurf = find(d > - tube.rout);
        %     dBodySurf = - d(IdxBodySurf);
        % 
        %     % remove the consective index
        %     [IdxBodySurfSelect, dBodySurfSelect] = removeConsective(IdxBodySurf, dBodySurf);
        % 
        %     contactSurfI = [];
        %     for Idxp = IdxBodySurfSelect % IdxBodySurf
        %         contactSurfI.type = 'surfaceContact';
        %         contactSurfI.tube_point = p(:, Idxp);
        %         contactSurfI.tube_point_id = Idxp;
        %         contactSurfI.point = plane.project(p(:, Idxp));
        %         contactSurfI.normal = plane.getNormal(p(:, Idxp));
        %         contactSurfI.penetrateDepth = d(Idxp)+tube.rout;  %signed dpeth
        %     end
        % 
        %     obsContact = [obsContact, contactSurfI];
        % end


   end
end