clear_all
close all
clc

opts = custom_simopts();

dt = opts.dt;
nsteps = opts.nsteps;

%make WmrModel object
if opts.animate
    [mdl, state, qvel, anim] = feval(opts.model_fh);
else
    [mdl, state, qvel] = feval(opts.model_fh);
end

feval(mdl.wgc_fh,mdl.wgc_p); %DEBUGGING, initialize wheel-ground contact model

mdl.bsm_fh = []; %DEBUGGING
if opts.ideal_actuators
    mdl.act_fh = [];
end

nf = mdl.nf;
nw = mdl.nw;
na = mdl.na;

% load-in or generate map
LOAD_MAP = true;
if LOAD_MAP
    data = load("/Users/Alvin/Documents/Code/safety_reachability_AV_research/assured_autonomy_car/search_planning_algos/terrains/test1_terrain.mat");
    xmin = data.xbounds(1); xmax = data.xbounds(2);
    ymin = data.ybounds(1); ymax = data.ybounds(2);
    nx = data.nx; ny = data.ny;
    Z = data.map;
    % GridSurf(xmin,xmax,nx,ymin,ymax,ny,Z)
    real_map = GridSurf(xmin,xmax,nx,ymin,ymax,ny,Z);
else
    lx = 20;
    ly = 10;
    ppm = 10;  % points per meter
    cx = 8;
    cy = 0;
    % real and fake maps have only different z-values
    [~, real_map] = feval(opts.terrain_fh, lx, ly, ppm, cx, cy);
    minx = min(fake_map.X);
    maxx = max(fake_map.X);
    miny = min(fake_map.Y);
    maxy = max(fake_map.Y);
    x_bounds = [minx, maxx];
    y_bounds = [miny, maxy];
    cellsize_m = 1 / ppm;  % 1m / ppm = 
end
    
%init contact geometry
if mdl.nw > 0
    contacts(1:mdl.nw) = WheelContactGeom();
    radii = [mdl.frames(mdl.wheelframeinds).rad];
elseif mdl.nt > 0
    contacts = initTrackContactGeom(mdl.frames(mdl.sprocketframeinds));
end
np = sum([contacts.np]);

%initialize wheel/terrain contact
if opts.init_in_contact
    state = initTerrainContact(mdl,surfs,contacts,state);
end

time = 0;
if opts.log
    dlog = SimulationLog(nsteps,nf,np,na,opts.dyn);

    %log initial values
    set_time(dlog,1,time);
    set_state(dlog,1,state);
    set_qvel(dlog,1,qvel);
end


if opts.animate
    figure(anim.h_fig)
    resizeFig(1.5,1.5)

    drawSurfaces({real_map},anim.h_axis);

    makeLegible(14)
    axis off

%     view(0,0)
    view(30,30)

    zoom(1.5)

end

do_pause = false;

%simulate

if opts.profile
    profile on
end
tic

% get other vehicle params
% [width, length]
rob_dims = [mdl.Wb, mdl.Lb];
mdl.min_turnrad = 7.3;

% state = ['rol'    'pit'    'yaw'    'x'    'y'    'z',  
%          'steerL'    'steerR'    'FL'    'FR'    'BL'    'BR']
current = [state(4), state(5), state(3)];  % x, y, theta
goal = [0.9*maxx, maxy/2];  % theta unconstrained
interr = zeros(na,1);
max_iters = 1000;
goal_dist_tol = 0.5;
iter = 0;
while (norm(current(1:2) - goal) > goal_dist_tol) && (iter < max_iters)
    % Pass map, robot pose to high-level planner, receive next finite
    % trajectory
%     double rob_dims[2],
%     double start_pose[3],
%     double end_pose[2],
%     double **map, 
%     double x_bounds[2], double y_bounds, double &cellsize_m)
    next_traj = main_planner(rob_dims, current, goal, fake_map, ...
        x_bounds, y_bounds, );
    
    y=[state; qvel; interr];
    [ydot,out] = odeDyn(time,y,mdl,surfs,contacts,dt);
    y = y + ydot*dt;
    [state,qvel,interr]=odeDynDecat(y,nf,na);
    time = time + dt;

    if opts.log
        set_time(dlog,i,time);
        set_state(dlog,i,state);
        set_qvel(dlog,i,qvel);
        %see SimulationLog for more to log
    end

    if opts.animate
        updateAnimation(anim, out.HT_parent, out.contacts);

        if do_pause
            pause
            do_pause = false;
        end
    end

end

comp_time = toc
if opts.profile
    profile off
    profile viewer
end

state


if opts.plot
    %%
    olen = SIZEORIENT();
    [isorient, ispos] = stateIs(nf);
    qnames = stateNames(mdl);
    lw = 1.5; %line width

    %path
    set(figure,'name','path')
    hold on
    plot3(dlog.pos(:,1), dlog.pos(:,2), dlog.pos(:,3),'LineWidth',lw)
    xlabel('x')
    ylabel('y')
    zlabel('z')
    makeLegible(14)
    axis equal

    %orientation vs time
    set(figure,'name','orientation vs time')
    hold on
    if olen==3
        plot(dlog.time, dlog.orient*180/pi,'LineWidth',lw)
        ylabel('deg')
    else
        plot(dlog.time, dlog.orient)
    end
    xlabel('time (s)')
    legend(qnames{isorient},'Location','Best')
    makeLegible(14)


end

%DEBUGGING
if 0
    %%
    %save the data log
    filename = '_autosave/ramp_ode23t.mat';
    save(filename,'opts','dlog','comp_time');
end