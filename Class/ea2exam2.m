%% ea2 exam 2 %%
clc
clear

% A= [2 -1 3; 0 4 2; -1 1 5];
% 
% specr= A* A';
% disp(specr)
% 
% B=specr^.5;
% row=eig(B);
% 
% disp(row)
% max(row)

% A=[1 0 0 0 0 0;.5 0 .5 0 0 0 ; 0 .5 0 .5 0 0 ; 0 0 .5 0 .5 0;0 0 0 .5 0 .5; 0 0 0 0 1 1];
% det(A)
% B=[.5 0 .5 0 0 0;.5 0 .5 0 0 0 ; 0 .5 0 .5 0 0 ; 0 0 .5 0 .5 0;0 0 0 .5 0 .5; 0 0 0 .5 0 .5];
% det(B)
% rank(B)

k= [100 -50 0; -50 120 -70; 0 -70 60];
K= norm(k,2)/norm(k^-1,2);

umax= .05;

f1= umax/norm(k^-1,1);
f2= umax/norm(k^-1,2);
finf= umax/norm(k^-1,"inf");

k1= norm(k^-1,1);
k2= norm(k^-1,2);
kinf= norm(k^-1,"inf");




