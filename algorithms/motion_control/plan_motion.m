function [public_vars] = plan_motion(read_only_vars, public_vars)
    %PLAN_MOTION Summary of this function goes here

    if isempty(public_vars.path) || any(isnan(public_vars.estimated_pose))
        public_vars.motion_vector = [0, 0];
        return;
    end

    if public_vars.kf_enabled && ~public_vars.kf.use_known_initial_pose && ~public_vars.kf.is_initialized
        public_vars.motion_vector = [0, 0];
        return;
    end

    current_pose = public_vars.estimated_pose;
    x = current_pose(1);
    y = current_pose(2);
    theta = current_pose(3);
    N = size(public_vars.path, 1);
    nextWayPoint = public_vars.path(end,:);

    lookahead = 0.7;
    for i = 1:N
        dx = public_vars.path(i,1) - x;
        dy = public_vars.path(i,2) - y;
    
        Lx =  cos(theta)*dx + sin(theta)*dy;
        Ly = -sin(theta)*dx + cos(theta)*dy;
    
        d  = hypot(Lx, Ly);
    
        if Lx > 0 && d >= lookahead
            nextWayPoint = public_vars.path(i,:);
            break;
        end
    end

    dx = nextWayPoint(1) - x;
    dy = nextWayPoint(2) - y;
    
    Lx =  cos(theta) * dx + sin(theta) * dy;
    Ly = -sin(theta) * dx + cos(theta) * dy;
    
    Ld = hypot(Lx, Ly);
    
    if Ld < 0.05
        curvature = 0;
    else
        curvature = 2 * Ly / (Ld^2);
    end

    vMax = 0.5;
    wMax = 1.0;

    v = min(vMax, Ld);
    w = v * curvature;
    w = max(min(w, wMax), -wMax);
    
    b = read_only_vars.agent_drive.interwheel_dist;
    
    vR = v + 0.5 * b * w;
    vL = v - 0.5 * b * w;

    public_vars.motion_vector = [vR, vL];

end
