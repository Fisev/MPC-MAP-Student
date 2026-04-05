function [public_vars] = plan_motion(read_only_vars, public_vars)
    %PLAN_MOTION Summary of this function goes here
    
    % I. Pick navigation target
    
    %target = get_target(public_vars.estimated_pose, public_vars.path);
    
    
    % II. Compute motion vector
    
    pose = read_only_vars.mocap_pose;
    x = pose(1);
    y = pose(2);
    theta = pose(3);

    bestDist = 1000000;
    N = size(public_vars.path, 1);
    nextWayPoint = public_vars.path(end,:);

    for i = 1:N
        dx = public_vars.path(i,1) - x;
        dy = public_vars.path(i,2) - y;
    
        Lx =  cos(theta)*dx + sin(theta)*dy;
        Ly = -sin(theta)*dx + cos(theta)*dy;
    
        if Lx > 0
            d = hypot(Lx, Ly);
            if d < bestDist
                bestDist = d;
                nextWayPoint = public_vars.path(i,:);
            end
        end
    end

    dx = nextWayPoint(1) - x;
    dy = nextWayPoint(2) - y;
    
    Lx =  cos(theta) * dx + sin(theta) * dy;
    Ly = -sin(theta) * dx + cos(theta) * dy;
    
    Ld = hypot(Lx, Ly);
    
    if Ld < 1e-4
        curvature = 0;
    else
        curvature = 2 * Ly / (Ld^2);
    end

    vMax = 0.5;
    wMax = 1.0;

    v = min(vMax, Ld);
    w = v * curvature;
    w = max(min(w, wMax), -wMax);

    public_vars.motion_vector = [v, w];

end