function [path] = plan_path(read_only_vars, public_vars)
    %PLAN_PATH Summary of this function goes here

    if isempty(public_vars.path)
        waypoints = [...
            2.0, 2.0;
            2.0, 7.0;
            13.0, 7.0;
            16.0, 4.0;
            16.0, 2.0];
        
        spacing = 0.4;
        path = [];
        for i = 1:size(waypoints,1)-1
            p1 = waypoints(i,:);
            p2 = waypoints(i+1,:);
            seg_len = norm(p2 - p1);
            n = max(2, ceil(seg_len / spacing));
            t = linspace(0,1,n)';
            segment = p1 + (p2 - p1) .* t;
            if i > 1
                segment = segment(2:end,:);
            end
            path = [path; segment];
        end
    else
        path = public_vars.path;
    end

end
