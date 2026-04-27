function [path] = plan_path(read_only_vars, public_vars)

    if ~isempty(public_vars.path)
        path = public_vars.path;
        return;
    end

    path = astar(read_only_vars, public_vars);
    path = smooth_path(path);

end
