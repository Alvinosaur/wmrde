%% Read in params
clear_all
close all
clc

opts = simopts();

% Cost Scales 
dt = opts.dt;
nsteps = opts.nsteps;
ROUGH_COST = opts.ROUGH_COST;
INCL_COST = opts.INCL_COST;

% Load 3D Terrain
surfs = feval(opts.terrain_fh);

do_pause = false;

%make WmrModel object
if opts.animate
    [mdl, state, qvel, anim] = feval(opts.model_fh);
    figure(anim.h_fig)
    resizeFig(1.5,1.5)
    drawSurfaces(surfs,anim.h_axis);
    makeLegible(14)
    axis off
    view(30,30)
    zoom(1.5)
else
    [mdl, state, qvel] = feval(opts.model_fh);
end

if opts.profile
    profile on
end

feval(mdl.wgc_fh,mdl.wgc_p); %DEBUGGING, initialize wheel-ground contact model

mdl.bsm_fh = []; %DEBUGGING
if opts.ideal_actuators
    mdl.act_fh = [];
end

nf = mdl.nf;
nw = mdl.nw;
na = mdl.na;

%% init contact geometry
if mdl.nw > 0
    contacts(1:mdl.nw) = WheelContactGeom();
    radii = [mdl.frames(mdl.wheelframeinds).rad];
elseif mdl.nt > 0
    contacts = initTrackContactGeom(mdl.frames(mdl.sprocketframeinds));
end
np = sum([contacts.np]);

%% initialize wheel/terrain contact
if opts.init_in_contact
    state = initTerrainContact(mdl,surfs,contacts,state);
end

state0 = state;

interr = zeros(na,1);
time = 0;

%% Map Definition
surface = surfs{1};

Class Planner(map, state0, dt)

    def init_action_space():
        self.A = [(speed, turning_radius)..]

    self.dt = dt
    self.T = 0.5  % some time duration of a lattice trajectory
    self.N = int(T / dt);

    def step(state, action) -> float:
        traj = zeros(N, 3);
        iter = 0;
        for t=dt:dt:self.T+dt
            y=[state; qvel; interr];
            [ydot,out] = odeDynCustom(action,y,mdl,surfaces,contacts,dt);
            y = y + ydot*dt;
            [state,qvel,interr]=odeDynDecat(y,nf,na);
            traj(iter, :) = full_state_to_plan_state(state);
            time = time + dt;
            iter = iter + 1;
            
        cost = gen_traj_cost(traj);
        return cost;
        
    def full_state_to_plan_state(state):
        x = state(1);
        y = state(2);
        theta = state(something);
        return [x, y, theta];
        


% 201 x 1
XRange = surface.X(:, 1);
X_SIZE = length(XRange);
% 1 x 101
YRange = surface.Y(1, :);
Y_SIZE = length(YRange);
% 201 x 101
ZMap = surface.Z(surface.X, surface.Y);

%DEFINE THE 2-D MAP ARRAY
MAX_VAL=10;
dx = (max(XRange) - min(XRange)) / X_SIZE;
dy = (max(YRange) - min(YRange)) / Y_SIZE;

%This array stores the coordinates of the map and the 
%Objects in each coordinate
MAP=2*(ones(X_SIZE,Y_SIZE));

% State definition
% [x, y, psi(yaw heading)]
% simple version doesn't include actual dynamics
% Cost of trajectory should be defined by: 
% time taken, danger of terrain, jerk, target-termination state
x0 = 0; y0 = 0; psi0 = 0;
state = [x0, y0, psi0];
STATE_SIZE = length(state);

% Predefined Lattice Action Space: 
% constant velocity, turning radius trajectories
VELOCITY_SPACE = linspace(0.25, 1.0, 4);
TURN_RAD_SPACE = [Inf, -5, -2.5, 2.5, 5];
TRAJ_TIME = 0.5;
ACTION_SPACE = zeros(length(VELOCITY_SPACE)*length(TURN_RAD_SPACE), 2);
for i = 1:length(VELOCITY_SPACE)
    for j = 1:length(TURN_RAD_SPACE)
        action = [VELOCITY_SPACE(i), TURN_RAD_SPACE(j)];
        ACTION_SPACE(i*length(TURN_RAD_SPACE) + j, :) = action;
    end
end

actual_state = [];

% Dynamics for rollout
dt = 0.1;


function [neighbors] = get_neighbors(state)
    global ACTION_SPACE
    neighbors = zeros(size(ACTION_SPACE, 1), STATE_SIZE + 1);
    for ai = 1:size(ACTION_SPACE, 1)
        action = ACTION_SPACE(ai,:);
        time = TRAJ_TIME;
        traj = gen_rollout(state, action, time);
        state_f = traj(end, :);
        traj_cost = gen_traj_cost(traj);
        neighbors(ai, :) = [state_f, traj_cost];
    end
end

function [traj_cost] = gen_traj_cost(traj)
    global mdl, surface;
    traj_cost = 0;
    for i = 1:size(traj, 1)
        state = traj(i,:);
        x = state(1); y = state(2); theta = state(3);
        % More robust way:
        % [HT_parent, HT_world] = stateToHT(mdl,state);
        % %update contact geometry
        % contacts = updateModelContactGeom(mdl, surfaces, HT_world, 0, contacts);
        frontx = x + mdl.Lb*cos(theta);
        fronty = y + mdl.Lb*sin(theta);
        
        fL, fR = get_left_right_wheels(frontx, fronty, theta, mdl);
        bL, bR = get_left_right_wheels(x, y, theta, mdl);
        pts = [fL; fR; bL; bR];  % 4 x 2
        minX = min(pts(:,1));
        maxX = max(pts(:,1));
        minY = min(pts(:,2));
        maxY = max(pts(:,2));

        % roughness of area covered by car
        % COM defiend in back by Ackermann Model
        % TODO: Assumes heading defined wrt forwar
        % TODO: Need to look at coordinate frames
        xrange = linspace(minX, maxX, dx);
        yrange = linspace(minY, maxY, dy);
        z_grid = get_z_grid(xrange, yrange, surface);
        roughness = terrain_roughness(z_grid);

        % avg inclination of terrain
        % get avg height around back and front wheels
        % TODO: Simplified, can make more accurate with actual wheel contact
        normals = surfaceNormal(surface, pts')';  % 4 x 3
        avg_normal = mean(normals);  % 1 x 3
        avg_normal = avg_normal / norm(avg_normal);  % make sure unit vec
        z_axis = [0, 0, 1];  % 1 x 3
        % rot = vrrotvec(avg_normal, z_axis);
        % [yaw, pitch, roll] = rotm2eul(rot, 'ZYX');
        incl = acos(z_axis * avg_normal');

        traj_cost = traj_cost + (ROUGH_COST * roughness) + (INCL_COST * incl);
    end
end

function [left, right] = get_left_right_wheels(cx, cy, theta, mdl)
    d = mdl.Wb/2;
    right = [cx + d*sin(theta), cy + d*cos(theta)];
    left = [cx - d*sin(theta), cy - d*cos(theta)];
end

function [traj] = gen_rollout(actual_state, state, action, T, dt)
    % repeatedly calls ackermannControllerCustom and OdeDynCustom
    tspan = [0 T];
    y0 = [state; qvel; interr];
    addlin = {mdl,{surface},dt};
    [time,yout] = feval(opts.solver,@odeDyn,tspan,y0,options,addlin{:});

    [ydot,out] = odeDynCustom(action,y,mdl,surfaces,contacts,dt)
end

function [world_x, world_y] = IDX_TO_POS(X_idx, Y_idx, surface)
    world_y = surface.Y(1, Y_idx);
    world_x = surface.X(X_idx, 1);
end

% find roughness over some region
function [z_grid] = get_z_grid(x_range, y_range, surface)
% x_range and y_range are REALWORLD X/Y COORDINATES!!! NOT INDEXES!!!
    [xq, yq] = ndgrid(x_range, y_range);
    z_grid = surface.Z(xq, yq);
end

function [roughness] = terrain_roughness(z_grid)
% Roughness defined as standard deviation of z values in a 2D region R
    z_var = var(z_grid, 'all');
    roughness = sqrt(z_var);
end

function [dist] = dist_of_grid_idxs(gridp1, gridp2, ZMap, surface)
% gridp1 and gridp2 are integer indexes that ARE NOT ACTUAL X/Y VALUES!!!
% They are converted to real-world x,y values inside this function
    p1 = IDX_TO_POS(gridp1(1), gridp1(2), surface);
    p2 = IDX_TO_POS(gridp2(1), gridp2(2), surface);
    x1 = p1(1); y1  = p1(2); z1 = ZMap(gridp1(1), gridp1(2));  % ZMap takes indexes, not real values
    x2 = p2(1); y2  = p2(2); z2 = ZMap(gridp1(1), gridp1(2));
    dist = sqrt((x1-x2)^2 + (y1-y2)^2 + (z1-z2)^2);
end


xTarget=0;  %X Coordinate of the Target
yTarget=0;  %Y Coordinate of the Target
MAP(xTarget,yTarget) = 0;  %Initialize MAP with location of the target

xStart=5;  %Starting Position
yStart=10;  %Starting Position
MAP(xStart,yStart)=1;

% Obstacles
MAP(3:9, 4:6) = -1;  %Put on the closed list as well

%End of obstacle-Target pickup
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%LISTS USED FOR ALGORITHM
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%OPEN LIST STRUCTURE
%--------------------------------------------------------------------------
%IS ON LIST 1/0 |X val |Y val |Parent X val |Parent Y val |h(n) |g(n)|f(n)|
%--------------------------------------------------------------------------
OPEN=[];
%CLOSED LIST STRUCTURE
%--------------
%X val | Y val |
%--------------
% CLOSED=zeros(MAX_VAL,2);
CLOSED=[];
%Put all obstacles on the Closed list
k=1;%Dummy counter
for i=1:MAX_X
    for j=1:MAX_Y
        if(MAP(i,j) == -1)
            CLOSED(k,1)=i; 
            CLOSED(k,2)=j; 
            k=k+1;
        end
    end
end
CLOSED_COUNT=size(CLOSED,1);
%set the starting node as the first node
xNode=xval;
yNode=yval;
OPEN_COUNT=1;
path_cost=0;
goal_distance=distance(xNode,yNode,xTarget,yTarget);
OPEN(OPEN_COUNT,:)=insert_open(xNode,yNode,xNode,yNode,path_cost,goal_distance,goal_distance);
OPEN(OPEN_COUNT,1)=0;
CLOSED_COUNT=CLOSED_COUNT+1;
CLOSED(CLOSED_COUNT,1)=xNode;
CLOSED(CLOSED_COUNT,2)=yNode;
NoPath=1;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% START ALGORITHM
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
while((xNode ~= xTarget || yNode ~= yTarget) && NoPath == 1)
%  plot(xNode+.5,yNode+.5,'go');
 exp_array=expand_array(xNode,yNode,path_cost,xTarget,yTarget,CLOSED,MAX_X,MAX_Y);
 exp_count=size(exp_array,1);
 %UPDATE LIST OPEN WITH THE SUCCESSOR NODES
 %OPEN LIST FORMAT
 %--------------------------------------------------------------------------
 %IS ON LIST 1/0 |X val |Y val |Parent X val |Parent Y val |h(n) |g(n)|f(n)|
 %--------------------------------------------------------------------------
 %EXPANDED ARRAY FORMAT
 %--------------------------------
 %|X val |Y val ||h(n) |g(n)|f(n)|
 %--------------------------------
 for i=1:exp_count
    flag=0;
    for j=1:OPEN_COUNT
        if(exp_array(i,1) == OPEN(j,2) && exp_array(i,2) == OPEN(j,3) )
            OPEN(j,8)=min(OPEN(j,8),exp_array(i,5)); %#ok<*SAGROW>
            if OPEN(j,8)== exp_array(i,5)
                %UPDATE PARENTS,gn,hn
                OPEN(j,4)=xNode;
                OPEN(j,5)=yNode;
                OPEN(j,6)=exp_array(i,3);
                OPEN(j,7)=exp_array(i,4);
            end;%End of minimum fn check
            flag=1;
        end;%End of node check
%         if flag == 1
%             break;
    end;%End of j for
    if flag == 0
        OPEN_COUNT = OPEN_COUNT+1;
        OPEN(OPEN_COUNT,:)=insert_open(exp_array(i,1),exp_array(i,2),xNode,yNode,exp_array(i,3),exp_array(i,4),exp_array(i,5));
     end;%End of insert new element into the OPEN list
 end;%End of i for
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 %END OF WHILE LOOP
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 %Find out the node with the smallest fn 
  index_min_node = min_fn(OPEN,OPEN_COUNT,xTarget,yTarget);
  if (index_min_node ~= -1)    
   %Set xNode and yNode to the node with minimum fn
   xNode=OPEN(index_min_node,2);
   yNode=OPEN(index_min_node,3);
   path_cost=OPEN(index_min_node,6);%Update the cost of reaching the parent node
  %Move the Node to list CLOSED
  CLOSED_COUNT=CLOSED_COUNT+1;
  CLOSED(CLOSED_COUNT,1)=xNode;
  CLOSED(CLOSED_COUNT,2)=yNode;
  OPEN(index_min_node,1)=0;
  else
      %No path exists to the Target!!
      NoPath=0;%Exits the loop!
  end;%End of index_min_node check
end;%End of While Loop
%Once algorithm has run The optimal path is generated by starting of at the
%last node(if it is the target node) and then identifying its parent node
%until it reaches the start node.This is the optimal path
i=size(CLOSED,1);
Optimal_path=[];
xval=CLOSED(i,1);
yval=CLOSED(i,2);
i=1;
Optimal_path(i,1)=xval;
Optimal_path(i,2)=yval;
i=i+1;
if ( (xval == xTarget) && (yval == yTarget))
    inode=0;
   %Traverse OPEN and determine the parent nodes
   parent_x=OPEN(node_index(OPEN,xval,yval),4);%node_index returns the index of the node
   parent_y=OPEN(node_index(OPEN,xval,yval),5);
   
   while( parent_x ~= xStart || parent_y ~= yStart)
           Optimal_path(i,1) = parent_x;
           Optimal_path(i,2) = parent_y;
           %Get the grandparents:-)
           inode=node_index(OPEN,parent_x,parent_y);
           parent_x=OPEN(inode,4);%node_index returns the index of the node
           parent_y=OPEN(inode,5);
           i=i+1;
    end;
 j=size(Optimal_path,1);
 %Plot the Optimal Path!
 p=plot(Optimal_path(j,1)+.5,Optimal_path(j,2)+.5,'bo');
 j=j-1;
 for i=j:-1:1
  pause(.25);
  set(p,'XData',Optimal_path(i,1)+.5,'YData',Optimal_path(i,2)+.5);
 drawnow ;
 end;
 plot(Optimal_path(:,1)+.5,Optimal_path(:,2)+.5);
else
 pause(1);
 h=msgbox('Sorry, No path exists to the Target!','warn');
 uiwait(h,5);
end
