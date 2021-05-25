function [Kx] = ppl(A, B, P)

%Step1: Find the similarity transform to get you to controller canonical
%form
    T1 = [B, A*B, A*A*B, A*A*A*B]; %Find the controllability matrix to make sure it controllable
    rankT1 = rank(T1); %If this is equal to the dimension of the B matrix we are controllable

    Pe = poly(eig(A));
    T2 = [Pe(1:4); 0,Pe(1:3); 0,0,Pe(1:2); 0,0,0,Pe(1)]; %Find the matrix related to the systems char. polynomial

    T3 = [0,0,0,1; 0,0,1,0; 0,1,0,0; 1,0,0,0]; %Define the flip matrix

    T = T1*T2*T3; %Find the transform to get to controller canonical form

    Az = inv(T)*A*T; %Find the controller canonical form of A
    Bz = inv(T)*B; %Find the controller canonical form of B

%Step2: Find the full-state feedback gains in controller form
    Pd = poly(P); %Polynomial form of the desired poles
    dP = Pd - Pe; %Difference between desired and "current" poles
    Kz = dP([5,4,3,2]); %Honestly, I'm not sure what this is doing
    
%Step3: Convert Kz to the gain times the state variables
    Kx = Kz*inv(T); 
    eig(A - B*Kx) %Check our answer

end

