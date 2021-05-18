function [outU] = DeltaU(pastU, pastYf, Utdi, setPoint, Ts, Tref, yk)

%This function is what will be used to calculate delta U matrix, or the 
%   change in the input from the previous time step for each subsequent
%   step. The first entry of the matrix will be used to calculate the next
%   optimal input for the system before the process is repeated

%   Theta is the matrix for the step response, setPoint is the vector of 
%       points that we want our output to reach at a given time step, Ts is
%       the sampling interval, Tref is the time constant for the speed of 
%       the response, and yk is the current output of the system.

%   Coincidence points basically indicate how many points along the
%       prediction trajectory there are at which the input is allowed to
%       change or be "non-constant". Note that here, I am assuming that P1
%       = 1 for our coincidence point calculation for the theta matrix, but
%       it still remains to be seen whether this is actually the case, in
%       this case I am using 5 as the number of coincidence points

tau = zeros(5, 1); %Declaration of the reference trajectory matrix
theta = zeros(5, 5); %Declaration of the theta matrix (I'm not sure if these dimentions are correct)
Yf = zeros(5, 1); %Declaration of the free response matrix

runYf = pastYf;

 for x = 1:5 %Nested loop to create the theta matrix
     for y = 1:5
         if y == 1
             theta(x,y) = freeResponse(runYf(1,1),runYf(2,1),runYf(3,1),1,Utdi); %Calculate for step response, so a value of 1 is used for all inputs
             runYf = circshift(runYf,1); %Shift all values of runYf to update them, then reassign the newest value, getting rid of the oldest
             runYf(1,1) = theta(x,y);
         elseif y > x
             theta(x,y) = 0;
         else
             theta (x,y) = theta(x-1,y-1);
         end
     end
 end

runYf = pastYf;

for x = 1:5 %Loop to create the tau matrix
    tau(x,1) = refTraj(setPoint(x,1), Ts, Tref, yk, x); %We pass setPoint(i:i+4,1) to get sets of 5 values for the set point at a time
end

for x = 1:5 %Loop to create the Yf matrix
    Yf(x,1) = freeResponse(runYf(1,1),runYf(2,1),runYf(3,1),pastU(1,1),Utdi); %Calculate for free response, so last applied input used for all inputs
    runYf = circshift(runYf,1); %Shift all values of runYf to update them, then reassign the newest value, getting rid of the oldest
    runYf(1,1) = Yf(x,1);
end

outU = theta\(tau-Yf);

%x = lsqlin(C,d,A,b,Aeq,beq,lb,ub); can try this bounded linear least
%square function to take care of the input boundaries????
end