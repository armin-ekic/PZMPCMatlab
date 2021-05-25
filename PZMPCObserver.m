%This function is a kalman filter implementation of the PZMPC problem to
    %understand the effect of an observer on the system.
    %These values are all determined based on the "state estimator" section
    %of the paper and is using code here that was written in ECE 663 -
    %Modern Controls

Utdi = 47.2;

A = [2.91, -2.822625, 0.9126005; 1, 0, 0; 0, 1, 0];
B = [-3.969/Utdi; 0; 0];
C = [0, 0, 1];
D = 0;

Q = diag([1000,1000,1000]); %Given Q matrix 
R = 15; %Given input weightine matrix

t = [0:0.01:10]';

%%%%This here is what I am unsure on%%%%%%%%%%%%%

H = ppl(A',C',[-1,-2,-3,-4])' %Arbitrarily placed poles

Aa = [A,zeros(3,3); H*C, A-H*C] %Augmented A matrix 
Ba = [B;B] %Augmented B matrix
Ca = diag([1,1,1,1,1,1]) %Augmented C matrix
Da = zeros(6,6) %Augmented D Matrix

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

setPoint = 0*t+110; %Arbitrarily set to day values for entire duration

X0 = zeros(9,1); %Initial conditions for the system, set to 0

y = step3(Aa,Ba,Ca,Da,t,X0,setPoint);
plot(t,y);