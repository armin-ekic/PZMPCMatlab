function [rHat] = refTraj(setP, Ts, Tref, yk, i)

%This is the function that will be used to compute the reference trajectory

%   The setPoint input is the point at which we want our systems output to
%       be at time step i, Ts is the sampling interval, Tref is the time 
%       constant defining the speed of the response, and the error
%       calculated by the difference: (setPoint - yk), where yk is the 
%       current output of the system

%   This function will find the reference trajectory value at time step
%       "i", using the setPoint at time step "i", multiplying Ts by the
%       current step value "i", and finding the error by subtracting the
%       set point at point "i" by the current output we are at (time "i")

rHat = setP - ((exp(-i*Ts/Tref))*(setP - yk));

end