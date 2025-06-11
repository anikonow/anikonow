%% sigmadecay %%

clc
clear
close all

% Given points
x = [0,30,45,75,90,100,120,140,180];
y = [.3, .33,.45,.63,.95,1.2,1.4,1.8,2.3];


% Fit a quadratic polynomial
p = polyfit(x, y, 2);

% Display the polynomial coefficients
disp('Sigma coefficients (a, b, c):');
disp(p);

% Define the function
y_func = @(x) p(1)*x.^2 + p(2)*x + p(3);

% Example usage
x_test = 0:10:180;
y_test = y_func(x_test);

% Plot
figure;
hold on

plot(x_test, y_test, 'b',linewidth=2);
xlabel('Degrees \phi',fontsize=12);
ylabel('Value',fontsize=12);


%x = [0, 90, 180];
yy = [0,.04,.05,.09,.11,.125,.16,.18,.19];

% Fit a quadratic polynomial
p = polyfit(x, yy, 2);

% Display the polynomial coefficients
disp('Decay coefficients (a, b, c):');
disp(p);

% Define the function
y_func = @(x) p(1)*x.^2 + p(2)*x + p(3);

% Example usage
x_test = 0:10:180;
y_test = y_func(x_test);

% Plot

plot(x_test, y_test, 'k',linewidth=2);
scatter(x,y,'k',"+",linewidth=2)
scatter(x,yy,'r',"+",linewidth=2)
% xlabel('x');
% ylabel('y');
% title('Quadratic Fit for Decay');
legend('Variance \sigma^2_{corner}','Decay \alpha',fontsize=12);