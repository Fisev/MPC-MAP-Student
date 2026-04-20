function [path] = plan_path(read_only_vars, public_vars)
    %PLAN_PATH Summary of this function goes here

    % Task 1 (Preparation):
    % Manual trajectory for outdoor_1 from [2,2,pi/2] to goal [16,2].
    public_vars.path = [
        2, 2;
        2, 8;
        16, 8;
        16, 2
    ];

    planning_required = 0;

    if planning_required
        path = astar(read_only_vars, public_vars);
        path = smooth_path(path);
    else
        path = public_vars.path;
    end

end
