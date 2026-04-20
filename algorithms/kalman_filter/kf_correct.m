function [new_mu, new_sigma] = kf_correct(mu, sigma, z, kf)
%KF_CORRECT Linear KF correction for GNSS measurements.

if isempty(z) || any(isnan(z))
    new_mu = mu;
    new_sigma = sigma;
    return;
end

H = kf.C;
Q = kf.Q;
I = eye(size(sigma));

innovation = z - H * mu;
S = H * sigma * H' + Q;
K = sigma * H' / S;

new_mu = mu + K * innovation;
new_mu(3) = wrapToPi(new_mu(3));

new_sigma = (I - K * H) * sigma * (I - K * H)' + K * Q * K';

end
