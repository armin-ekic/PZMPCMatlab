function [freeOut] = freeResponse(yk1, yk2, yk3, uk3, Utdi)

%This is the free response/step response function yf(k)/S(Pc)
%   This is the transfer function for the system in difference equation
%       form, where each coincidence point Pc is calculated using an
%       input(Pc), and the previously calculated outputs

%   The inputs yki denote the ith past output calculated by this function, 
%       uk3 denotes the input applied 3 time steps in the past, and Utdi
%       is a predetermined subject specific total daily insulin amount

%   This function will not create the theta matrix, but will be used to
%       calculate the values within the required matrix

%   In addition to helping calculate the theta matrix, this function will
%       also be used to Yf, the free response values and matrix

freeOut = (2.91*yk1) - (2.822625*yk2) + (0.9126005*yk3) + ((-3.969/Utdi)*uk3);

end