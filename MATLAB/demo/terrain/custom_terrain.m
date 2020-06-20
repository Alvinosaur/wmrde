function [fake_map, real_map] = custom_terrain(lx, ly, ppm, cx, cy)
setseed(123);
RmsHeight = .15;
CorrLength = .8;
Z = randommat(lx,ly,ppm,RmsHeight,CorrLength);
% GridSurf(xmin,xmax,nx,ymin,ymax,ny,Z)
fake_map = GridSurf(...
    cx-lx/2,cx+lx/2,size(Z,1),...
    cy-ly/2,cy+ly/2,size(Z,2),Z);

% add some true, unknown obstacle
obs_bounds_x = [lx/2, 3*lx/4] .* ppm;
obs_bounds_y = [ly/4, 3*ly/4] .* ppm;
current_Z = Z(obs_bounds_x(1):obs_bounds_x(2), ...
    obs_bounds_y(1):obs_bounds_y(2));
Z(obs_bounds_x(1):obs_bounds_x(2), ...
    obs_bounds_y(1):obs_bounds_y(2)) = current_Z + 2;
real_map = GridSurf(...
    cx-lx/2,cx+lx/2,size(Z,1),...
    cy-ly/2,cy+ly/2,size(Z,2),Z);


end