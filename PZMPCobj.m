A = [2.19,-2.822625,0.9126005; 1,0,0; 0,1,0];
B = [-3.969/47.2; 0; 0];
C = [0,0,1];
D = 0;
G = ss(A,B,C,D,5); %Assuming we have a sampling rate of 5

manVars = [0];
manVarsRate = [0.1];
outVars = [1];
ECR = 5; %idk what this is doing, just an arbitrary value for now

for i = 1:288
    if(i >= 72 || i <= 156)
        inputHigh(i,1) = 1.5;
    else
        inputHigh(i,1) = 10; %arbitrary pump max during day time
    end
end

for i = 1:288 %Transition from day to night at 5AM to 7AM, and night to day at 10PM to 12PM, sim starts at 4pm, ends at 4pm, but need to account for additional prediction horizon length at end (maybe only go to 296?)
    if(i < 72 || i > 180) 
        outputHigh(i,1) = 140;
    elseif(i >= 72 && i <= 96) %Check to see if we are between 10PM and 12PM (day to night transition
        outputHigh(i,1) = 1.25*i - 10;
    elseif(i >= 156 && i <= 180) %Check to see if we are between 5AM and 7AM (night to day transition)
        outputHigh(i,1) = -1.25*i + 305;
    else
        outputHigh(i,1) = 220;
    end
end

for i = 1:288 %Transition from day to night at 5AM to 7AM, and night to day at 10PM to 12PM, sim starts at 4pm, ends at 4pm, but need to account for additional prediction horizon length at end (maybe only go to 296?)
    if(i < 72 || i > 180) 
        outputLow(i,1) = 80;
    elseif(i >= 72 && i <= 96) %Check to see if we are between 10PM and 12PM (day to night transition
        outputLow(i,1) = (10/3)*i - 100;
    elseif(i >= 156 && i <= 180) %Check to see if we are between 5AM and 7AM (night to day transition)
        outputLow(i,1) = (-10/3)*i + 740;
    else
        outputLow(i,1) = 110;
    end
end

%MV = struct('Min',0, 'Max',inputHigh);
%OV = struct('Min',outputLow, 'Max',outputHigh);
MV = struct('Min',0, 'Max',1.5);
OV = struct('Min',110, 'Max',220); 
W = struct('ManipulatedVariables',manVars, 'ManipulatedVariablesRate',manVarsRate, 'OutputVariables',outVars, 'ECR',ECR);
%DV = struct('ScaleFactor',0);


% mpcobj = mpc(G);
% mpcobj.MV(1).Min = 0; mpcobj.MV(1).Max = 1.5;
% mpcobj.PredictionHorizon = 9;
% mpcobj.ControlHorizon = 5;
% mpcobj.Weights.ManipulatedVariables = manVars;
% mpcobj.Weights.ManipulatedVariablesRate = manVarsRate;
% mpcobj.Weights.OutputVariables = outVars;
% mpcobj.Weights.ECR = ECR;

mpcobj = mpc(G,300,9,5,W,MV,OV)