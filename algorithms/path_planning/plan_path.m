function [path] = plan_path(read_only_vars, public_vars)
    %PLAN_PATH Summary of this function goes here

    %% Task 2
    N = 20;
    x1 = (ones(N, 1) * 2);
    y1 = linspace(1.1, 9, N)';
    straightLine = [x1, y1];

    alpha = linspace(pi/2, pi, N)';
    x2 = 5*cos(alpha)+7;
    y2 = 5*sin(alpha)+1;
    circuralArc = [x2, y2];

    y3 = linspace(1, 9, N)';
    x3 = 0.5*sin(0.8*y3)+1.5;
    sineWawe = [x3, y3];
 
    public_vars.path = sineWawe;
%%

    planning_required = 0;

    if planning_required
        path = astar(read_only_vars, public_vars);
        path = smooth_path(path);
    else
        path = public_vars.path;
    end

end