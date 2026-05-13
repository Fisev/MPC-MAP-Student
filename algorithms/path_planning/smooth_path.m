function [new_path] = smooth_path(old_path)
    % close to the original path alpha
    % waypoint towards its neighbours beta
    
    if size(old_path, 1) < 3
        new_path = old_path;
        return;
    end

    alpha   = 0.4;
    beta    = 0.1;
    maxIter = 60;

    X = old_path;
    Y = X;

    for iter = 1:maxIter
        for i = 2:size(X, 1) - 1
            Y(i, :) = Y(i, :) + alpha * (X(i, :) - Y(i, :)) + beta  * (Y(i-1, :) + Y(i+1, :) - 2*Y(i, :));
        end
    end

    new_path = Y;
end
