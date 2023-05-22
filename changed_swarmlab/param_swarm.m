% In your main, run this script after the swarm initialization

% Variables to be set
p_swarm.is_active_migration = false;
p_swarm.is_active_goal = true;
p_swarm.is_active_arena = false;
p_swarm.is_active_spheres = false;
p_swarm.is_active_cyl = true;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Number of agents
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if ~isfield(p_swarm, 'nb_agents')
    p_swarm.nb_agents = 5;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Max radius of influence - Metric distance
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

p_swarm.r = 150;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Max number of neighbors - Topological distance
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if ~isfield(p_swarm, 'max_neig')
    p_swarm.max_neig = 10;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Radius of collision -
% it is the radius of the sphere that approximates
% the drone. A collision is counted when two 
% spheres intersect.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

p_swarm.r_coll = 0.5;
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Arena parameters - Cubic arena
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% x,y,z positions of the walls
p_swarm.x_arena = [-100 100; % x wall
            -100 100; % y_wall
            -100 100]; % z_wall
p_swarm.center_arena = sum(p_swarm.x_arena, 2) / 2;

% Parameter that defines the influence radius of the arena repulsion force
p_swarm.d_arena = 1.5;

% Constant of proportionality of the arena repulsion force
p_swarm.c_arena = 10;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Spheric obstacles parameters
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

p_swarm.spheres = [
    -70 70 0; % x_obstacle
    50 50 200; % y_obstacle
    5 5 5; % z_obstacle
    50 50 50]; % r_obstacle

p_swarm.n_spheres = length(p_swarm.spheres(1, :));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Cylindric obstacles parameters
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if (exist('map','var') && ACTIVE_ENVIRONMENT)

    nb_obstacles = length(map.buildings_east);
    cylinder_radius = map.building_width / 2;
    
    p_swarm.cylinders = [
        map.buildings_north'; % x_obstacle
        map.buildings_east'; % y_obstacle
        repmat(cylinder_radius, 1, nb_obstacles)]; % r_obstacle

    p_swarm.n_cyl = length(p_swarm.cylinders(1, :));
    
else
    p_swarm.cylinders = 0;
    p_swarm.n_cyl = 0;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Reference values
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Inter-agent distance
if ~isfield(p_swarm, 'd_ref')
    p_swarm.d_ref = 25;
end

% Velocity direction
p_swarm.u_ref = [1 0 0]';

% Speed
if ~isfield(p_swarm, 'v_ref')
    p_swarm.v_ref = 6;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Velocity and acceleration bounds for the agents
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

p_swarm.max_a = 10;
% p_swarm.max_a = []; % leave empty if you use a real drone model
p_swarm.max_v = 7;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initial position and velocity for the swarm
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Initial positions are contained in a cubic area
p_swarm.P0 = [-50,125,-50]'; % [m] position of a vertex of the cube
p_swarm.P = 50; % [m] cube edge size

% Velocities are inizialized in a cubic subspace
p_swarm.V0 = [0,0,0]'; % [m/s]
p_swarm.V = 0; % [m/s]



% Set the goal
% --------- Setting 1 begins ----------
goal_y = 164.5;
goal_x = 180;
x_goal_temp = [goal_x; goal_y; -38];

p_swarm.x_goal = repmat(x_goal_temp, 1, p_swarm.nb_agents);
% --------- Setting 1 ends ----------

% --------- Setting 2 begins ----------
% set the seed to avoid random effects
myStream = RandStream('mt19937ar','Seed', p_swarm.seed);
p_swarm.Pos0 = p_swarm.P0 + p_swarm.P * rand(myStream,3,p_swarm.nb_agents);
% we make the z-axis of all swarm members the same
p_swarm.Pos0 = [p_swarm.Pos0(1:2, :); -38*ones(1, p_swarm.nb_agents)];

p_swarm.Vel0 = p_swarm.V0 + p_swarm.V * rand(myStream,3,p_swarm.nb_agents);
% p_swarm.Vel0 = p_swarm.V0 + p_swarm.V * rand(3,p_swarm.nb_agents);
% --------- Setting 2 ends ----------

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Call algorithm-specific swarm parameters
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if exist('SWARM_ALGORITHM','var')
    str = "param_";
    run(strcat(str, SWARM_ALGORITHM));
end
