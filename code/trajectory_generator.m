function [ desired_state ] = trajectory_generator(t, qn, map, path)
% TRAJECTORY_GENERATOR: Turn a Dijkstra or A* path into a trajectory
%
% NOTE: This function would be called with variable number of input
% arguments. In init_script, it will be called with arguments
% trajectory_generator([], [], map, path) and later, in test_trajectory,
% it will be called with only t and qn as arguments, so your code should
% be able to handle that. This can be done by checking the number of
% arguments to the function using the "nargin" variable, check the
% MATLAB documentation for more information.
%
% map: The map structure returned by your load_map function
% path: This is the path returned by your planner (dijkstra function)
%
% desired_state: Contains all the information that is passed to the
% controller, as in phase 2
%
% It is suggested to use "persistent" variables to store map and path
% during the initialization call of trajectory_generator, e.g.
% persistent map0 path0
% map0 = map;
% path0 = path;

function [co,T]=traj(pt1,pt2)
d=norm(pt2-pt1); %time to travel between 2 waypoints  
if(d<=0.6)
    T=1.3*d;
elseif(d<=1)
    T=1.0*d;        
elseif(d<=2)
    T=0.8*d;
elseif(d<3)
    T=0.6*d;
elseif(d<4)
    T=0.45*d;
elseif(d<=5)
    T=0.4*d;
elseif(d<=8)
    T=0.35*d;
elseif(d<=10)
    T=0.33*d;
elseif(d>10)
    T=0.3*d;
end
M=[1,0,0,0;1,T,T^2,T^3;0,1,0,0;0,1,2*T,3*T^2];
co=M\[pt1;pt2;0,0,0;0,0,0 ];        

end
desired_state = [];

% use the "persistent" keyword to keep your trajectory around
% inbetween function calls
persistent final c  cumtimer 
if(isempty(t)~=0)
    pts=path;     
    timer=zeros([size(pts,1)-1,1]);
    c= zeros([4,3,size(pts,1)-1]);
    for i=1:size(pts,1)-1
      [c(:,:,i),timer(i)]=traj(pts(i,:),pts(i+1,:));
    end
    cumtimer=[ 0; cumsum(timer)];
    final=pts(end,:)';
else   
    if(t<cumtimer(end))
        present=find(cumtimer>t)-1;
        prev=find(cumtimer<=t);
        p=c(:,:,present(1));
        t=t-cumtimer(prev(end));
        desired_state.pos =p'*[1;t;t^2;t^3];        
        desired_state.vel=p'*[0;1;2*t;3*t^2];
        desired_state.acc=p'*[0;0;2;6*t];
    else   
        if(t>13.7995 && t< 13.8)
            desired_state.pos=final;
        end
  
        desired_state.pos  =final;
        desired_state.vel=[0;0;0];
        desired_state.acc= [0;0;0];  
    end 
end
desired_state.yaw = 0;
desired_state.yawdot = 0;

end







