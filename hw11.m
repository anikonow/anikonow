%% Hw 11 gro veh %%

function p = sph_2_cart (alpha, epsilon,r)

p = zeros(3);

%calc for x y and z
p = [r*cos(alpha)*cos(epsilon); r*sin(alpha)*cos(epsilon);r*sin(epsilon)];

end

function param_est = estimate_params(P)

param_est = zeros(3);

x= P(:,1);
y= P(:,2);
z= P(:,3);

A = [ones(size(P,1)) x y];

est = A \ z;

param_est= [ est(1) est(2) est(3)]';

end


