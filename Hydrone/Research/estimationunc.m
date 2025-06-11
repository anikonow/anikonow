clear
clc
close all

mu = [0, 0]; 
sigma_x = 1;  % Standard deviation in x
sigma_y = 1;  % Standard deviation in y

Sigma = [sigma_x, 0; 
         0, sigma_y];

x_range = linspace(-3*sigma_x, 3*sigma_x, 100);
y_range = linspace(-3*sigma_y, 3*sigma_y, 100);
[X, Y] = meshgrid(x_range, y_range);


X_vector = [X(:) Y(:)]; 
pdf_values = mvnpdf(X_vector, mu, Sigma); 
pdf_values = reshape(pdf_values, size(X)); 

%% 3D plot
figure;
surf(X, Y, pdf_values, 'EdgeColor', 'none');
colormap('parula');
colorbar; 


xlabel('X');
ylabel('Y');
zlabel('Probability Density');
title('PDF of Agent Position');

shading interp;
view(30, 30); 
grid on;


%% scatter plot
rng(42);
samples = mvnrnd(mu, Sigma, 1000);


figure;
scatter(samples(:,1), samples(:,2), 20, 'b', 'filled');
hold on;
plot(mu(1), mu(2), 'rx', 'MarkerSize', 12, 'LineWidth', 2);

xlabel('X');
ylabel('Y');
grid on;
axis equal;
legend('Samples', 'Mean');

%% interaction plot
% mu1 = -1; sigma1 = 2; % Mean and std
% mu2 = 2; sigma2 = 1; 
% 
% x = linspace(-5, 5, 1000);
% 
% 
% pdf1 = normpdf(x, mu1, sigma1);
% pdf2 = normpdf(x, mu2, sigma2);
% overlap = trapz(x, min(pdf1, pdf2));
% 
% 
% fprintf('Percentage overlap: %.2f%%\n', overlap * 100);
% 
% % Plot
% figure;
% plot(x, pdf1, 'b', 'LineWidth', 2); hold on;
% plot(x, pdf2, 'r', 'LineWidth', 2);
% fill(x, min(pdf1, pdf2), 'k', 'FaceAlpha', 0.3); 
% legend('PDF 1', 'PDF 2', 'Overlap Region');
% xlabel('x');
% ylabel('Probability Density');
% title(sprintf('Probablity of Collision: %.1f%%',overlap*100));
% grid on;