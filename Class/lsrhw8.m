

clc
clear

current = [.2 .3 .4 .5 .6]';  
voltage = [1.23 1.38 2.06 2.47 3.17]';  
i=[0:1:4];
r=voltage .* current;

H = [ones(size(current)), current];
h = (H' * H) \ (H' * voltage);


R = 1; 
P = eye(2);  
h = [1; R]; 

sigma_r = 0.1; 

for i = 1:length(current)
    y = voltage(i);
    H = [1, current(i)]; 
    K = P * H' * inv(H * P * H' + sigma_r^2); 
    h = h + K * (y - H * h);  
    P = (eye(2) - K * H) * P;  
    
    R(i) = h(2);
    P = P + sigma_r^2 * eye(2);  
    
    
end
plot (i, r)
hold on
plot([0,4],h(:,1))
plot(i,R)
hold off