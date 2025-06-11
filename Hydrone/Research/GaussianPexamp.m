%% Gaussian Process examples %%

clc
clear
close all

table = [
35.35309347	1.79	2.32	2.39	2.33	2.46	2.47	2.5	2.47	2.53	2.3	2.41	2.35	2.45	2.28	2.8 2.3138
35.3529353	3.05	3.88	4.12	3.82	4.1	4.1	4.4	3.9	4.2	3.73	4.07	3.75	4.07	3.73	4.41 3.86965
35.35277623	4.43	5.07	5.31	5.01	5.39	4.93	5.65	4.86	5.2	4.84	5.49	4.84	5.37	4.77	5.34 5.05648
35.35262501	5.23	5.96	6.33	6.08	6.85	5.71	6.1	5.84	6.27	5.92	6.91	5.89	6.43	5.83	6.42 5.944106
35.35250981	5.54	6.24	6.51	6.24	7.06	6.02	6.32	6.21	6.52	6.2	6.73	6.17	6.77	6.21	6.6 6.2233
];



x = (table(:,1)-min(table(:,1)));
y = table(:,6);

gprMdl1 = fitrgp(x,y, 'kernelfunction','squaredexponential');

y= table(:,5);
gprMdl2 = fitrgp(x,y, 'kernelfunction','squaredexponential');

y=table(:,2);

gprMdlref = fitrgp(x,y, 'kernelfunction','squaredexponential');

xx = linspace(0,max(x),100);
y1 = predict(gprMdl1,xx');
[y2,ysd] = predict(gprMdl2,xx');
yref = predict(gprMdlref,xx');

%% intro
figure;
hold on

scatter(x,table(:,5),'MarkeredgeColor','k','marker','+','Linewidth',2)

plot(x,table(:,5),'color','r','LineStyle','--','linewidth',2)
plot(xx,y2,'color',[.1,.3,.4],'linewidth',2)
plot(xx,yref+.7,'color',[.1,.8,.5],'linewidth',2)
% 
plot(xx,y2+ysd,'Color',[.1,.3,.4],'LineStyle','--')
plot(xx,y2-ysd,'Color',[.1,.3,.4],'LineStyle','--')

legend('Data Set','Interpolation','Gaussian','True Depth')



xlabel('Decimal Degrees')
ylabel('Depth (m)')
title('Gaussian Process')
hold off
hold off



%% data results

% 
% figure;
% hold on
% scatter(x,table(:,2),'MarkeredgeColor',[.8,.2,.5])
% scatter(x,table(:,6),'MarkeredgeColor',[.5,.8,.2])
% scatter(x,table(:,5),'MarkeredgeColor',[.2,.5,.8])
% plot(xx,yref,'color','r')
% plot(xx,y1,'color','g')
% plot(xx,y2,'color','b')
% 
% 
% xlabel('Decimal Degrees')
% ylabel('Depth (m)')
% title('Gaussian Process on a Profile')
% hold off