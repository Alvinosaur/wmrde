%test_simulate.m

clear all
close all
clc

opts = simopts();

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

data = load("/Users/Alvin/Documents/Code/safety_reachability_AV_research/assured_autonomy_car/search_planning_algos/terrains/test1_terrain.mat");
xmin = double(data.xbounds(1)); xmax = double(data.xbounds(2));
ymin = double(data.ybounds(1)); ymax = double(data.ybounds(2));
nx = double(data.nx); ny = double(data.ny);
Z = data.map;
surface = GridSurf(xmin,xmax,nx,ymin,ymax,ny,Z);
surfs = {surface};


% surfs = feval(opts.terrain_fh);
% vals = linspace(0,5,1);
% disp(surfs{1}.Z(vals, vals));

% disp(surfs{1}.DZDX);
% disp(surfs{1}.X);
% surfs{1}.X and .Y: = 201 x 101 grid
% example: size(ndgrid(linspace(0,5),linspace(0,5))) = 100 x 100
% 100x100 grid
% disp(surfs{1}.X);   
% disp(surfs.

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

state0 = state;
disp(state0);

interr = zeros(na,1);
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
    
    drawSurfaces(surfs,anim.h_axis);
    
    makeLegible(14)
    axis off
    
%     view(0,0)
    view(30,30)
    
    zoom(1.5)
    
end

do_pause = true;

%simulate

if opts.profile
    profile on
end
tic


if opts.use_builtin_solver
    %use built-in MATLAB ode solver
    mdl.use_constraints = false;
    opts.abs_tol = 1e-5;
    opts.rel_tol = 1e-03;
    

    options = odeset('AbsTol',opts.abs_tol,'RelTol',opts.rel_tol,'MaxStep',.20);

    tspan = [0 (nsteps-1)*dt];
    if opts.dyn
        y0 = [state; qvel; interr];
        addlin = {mdl,surfs,dt};
        [time,yout] = feval(opts.solver,@odeDyn,tspan,y0,options,addlin{:});

        [STATE,QVEL,~] = odeDynDecat(yout',nf,na);

        STATE = STATE';
        QVEL = QVEL';

    else
        addlin={mdl,surfaces};
        [time,yout] = feval(opts.solver,@odeKin,tspan,state,options,addlin{:});

        STATE = yout;
        [~,QVEL] = diffState(time,STATE,isorient);
    end

    nsteps = length(time);

    dlog = SimulationLog(nsteps,nf,np,na,opts.dyn); %number of steps changed

    %data logging
    set_time(dlog,1:nsteps,time);
    set_state(dlog,1:nsteps,STATE);
    set_qvel(dlog,1:nsteps,QVEL);
    
    state(:) = dlog.state(end,:);
else
    %use force-balance optimizaftion technique
    for i = 2:nsteps

        if opts.dyn
            y=[state; qvel; interr];
            [ydot,out] = odeDyn(time,y,mdl,surfs,contacts,dt);
            y = y + ydot*dt;

%             [y,out]=RK4step(@odeDyn,dt,time,y,{mdl,surfs,contacts,dt}); %DEBUGGING
            
            [state,qvel,interr]=odeDynDecat(y,nf,na);
        else
            [statedot,out] = odeKin(time,state,mdl,surfs,contacts);
            state = state + statedot*dt;
        end
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



