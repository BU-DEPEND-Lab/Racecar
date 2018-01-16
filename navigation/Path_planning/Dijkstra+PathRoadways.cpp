#include <ctype.h>
#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>
#include <fstream>
#include <iostream>
#include <string>
#include <stdlib.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <numeric>
#include <math.h>
#include <cmath>
#include <utility>
#include <map>
#include <algorithm>
#include <unordered_map>
#include <time.h>
#define HEIGHT 2048
#define LENGTH 2048

using namespace std;
using namespace cv;

struct traj{
    vector<vector<int>> X;
    float time
};

vector<vector<int>> state_space = {{-4, -4}, {6, 8}};    // lower and upper corners
load_targets;
load_obstacles;

// Set initial state
float x_0 = -2.5;
float y_0 = 5.5;
float phi_0 = 0;
float v_0 = 0;
traj.X = {{x_0, y_0}, {phi_0, v_0}};
traj.time = 0;

int acc = 2;        // robot velocity
float delta = 0.01;   // discretization step

//PID parameters
float kp = 6;
float ki = 0.001;
float kd = 0;
float err_sum = 0;

// Generate random road maps (used to generate paths)
[nodes,adjM,costM] = gen_road_map(state_space,50,targets,obstacles);

// Plot road maps
plot_roadmaps(nodes,adjM,obstacles)
axis([-4 6  -4 8]);

path_i = 1; % Actual node 
target_i = randi([2 length(targets)]);          // Select a target
fprintf('Selected target = %i\n',target_i);
[~,path] = dijkstra(adjM,costM,1,target_i);     // Find path from actual node to target
x_goal = nodes(path(path_i+1),1);               // Get coordinates of next node 
y_goal = nodes(path(path_i+1),2);

t = 1;
while 1
    
    // Update graphics
    plot_scenario(traj.X(:,t),targets,obstacles);
    
    // Actual pose
    x = traj.X(1,t);      // x-coordinate
    y = traj.X(2,t);      // y-coordinate
    phi = traj.X(3,t);    // Orientation
    v = traj.X(4,t);      // Velocity  
    
    if close_to(x,y,x_goal,y_goal,0.2)      // Check if robot is close to a node
        if path_i == length(path)-1     // Is it the end of the path?
            
            // Select new target and compute new path
            targets_idx = [1:1:length(targets)];    
            targets_idx(target_i) = [];
            
            target_i = targets_idx(randi([1 length(targets_idx)]));
            fprintf('Selected target = %i\n',target_i);
            path_i = 1;
            [~,path] = dijkstra(adjM,costM,path(end),target_i);
        else
           path_i = path_i + 1;
        end
        // Select next node
        x_goal = nodes(path(path_i+1),1);
        y_goal = nodes(path(path_i+1),2);
    end
   
    // Difference between actual pose and node direction
    phi_d = atan2(y_goal-y,x_goal-x);
    err_ang = phi_d - phi;
    err = atan2(sin(err_ang),cos(err_ang));
    err_sum = err_sum + err;
    
    // New orientation (PID controller - just PI for now)
    omega = kp*err + ki*err_sum;
    // New state
    traj.X(:,end+1) = unicycle( [x;y;phi;v], [omega;acc], delta );   
    traj.time(end+1) = traj.time(end) + delta;
    
    t = t + 1;