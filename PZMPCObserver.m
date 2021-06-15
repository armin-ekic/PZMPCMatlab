%This function is a kalman filter implementation of the PZMPC problem to
    %understand the effect of an observer on the system.
    %These values are all determined based on the "state estimator" section
    %of the paper and is using code here that was written in ECE 663 -
    %Modern Controls

Utdi = 47.2;

A = [2.91,-2.822625,0.9126005; 1,0,0; 0,1,0];
B = [-3.969/Utdi; 0; 0];
C = [0,0,1];
D = 0;

Q = diag([1000,1000,1000]); %Given Q matrix 

t = [0:0.01:10]';

R = 15; %Given input weighting matrix

H = lqr(A',C',Q,R)';

Kx = lqr(A,B,Q,R) %Maybe find Kx like Nate found it?
DC = -C*inv(A-B*Kx)*B;
Kr = 1/DC

%%%%This here is what I am unsure on%%%%%%%%%%%%%

%maybe these are the matrices to give us the gains, and the other matrices
%come later??????? But these are basically whats shown in the book

% Aa = [A,zeros(3,3); H*C, A-H*C] %Augmented A matrix 
% Ba = [B;B] %Augmented B matrix
% Ca = diag([1,1,1,1,1,1]) %Augmented C matrix
% Da = zeros(6,1) %Augmented D Matrix
A6 = [A,-B*Kx; H*C,A-B*Kx-H*C]
B6 = [B*Kr;B*Kr]
C6 = diag([1,1,1,1,1,1])
D6 = zeros(6,1)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

setPoint = 0*t + 1; %Arbitrarily set to day values for entire duration

X0 = zeros(6,1); %Initial conditions for the system, set to 0

y = step3(A6,B6,C6,D6,t,X0,setPoint);
plot(t,y,t,setPoint,'--');