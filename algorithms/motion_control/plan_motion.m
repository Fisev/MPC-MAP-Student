function [public_vars] = plan_motion(read_only_vars, public_vars)
%PLAN_MOTION Summary of this function goes here

% I. Pick navigation target

target = get_target(public_vars.estimated_pose, public_vars.path);


% II. Compute motion vector

    %%  Task 5 
    i = read_only_vars.counter;

if i < 170
    public_vars.motion_vector = [0.5, 0.5];     % rovně
elseif i < 210
    public_vars.motion_vector = [0.125, 0.225]; % doprava
elseif i < 280  
    public_vars.motion_vector = [0.5, 0.5];     % rovně
elseif i < 320 
    public_vars.motion_vector = [0.125, 0.2];   % doprava
elseif i < 460  
    public_vars.motion_vector = [0.5, 0.5];     % rovně
elseif i < 500  
    public_vars.motion_vector = [0.2, 0.125];   % doleva
elseif i < 580  
    public_vars.motion_vector = [0.5, 0.5];     % rovně
elseif i < 620  
    public_vars.motion_vector = [0.2, 0.125];   % doleva
elseif i < 676  
    public_vars.motion_vector = [0.5, 0.5];     % rovně
elseif i < 1640  
    public_vars.motion_vector = [0.5, 0.5];     % rovně
else
    public_vars.motion_vector = [0, 0];
end

%%
end