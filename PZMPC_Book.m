%C = 5; %It looks like C = Hu given the equations in the paper
Utdi = 47.2; %This is given as the mean for adults in a cited paper (in silico subjexts of the University of Padova/Virginia FDA - 23)

setPoint = zeros(297,1);
for i = 1:297 %Transition from day to night at 5AM to 7AM, and night to day at 10PM to 12PM, sim starts at 4pm, ends at 4pm, but need to account for additional prediction horizon length at end (maybe only go to 296?)
    if(i < 72 || i > 180) %If it is day time, the set point is 110
        setPoint(i,1) = 110;
    elseif(i >= 72 && i <= 96) %Check to see if we are between 10PM and 12PM (day to night transition
        setPoint(i,1) = ((1.25*i - 10)+((10/3)*i - 100))/2;
    elseif(i >= 156 && i <= 180) %Check to see if we are between 5AM and 7AM (night to day transition)
        setPoint(i,1) = ((-1.25*i + 305)+(-(10/3)*i + 740))/2;
    else
        setPoint(i,1) = 165; %If none of the above, it is night time and the set point is 165
    end
end
Ts = 5; %Sampling interval is every 5 minutes (given from graphs of various strategies)
Tref = 50; %Arbitrarily chosen value in order to perform simulation (book says to use Ts = Tref/10)

A = [2.91, -2.822625, 0.9126005; 1, 0, 0; 0, 1, 0];
B = [-3.969/Utdi; 0; 0];
C = [0, 0, 1];
D = 0;
P = [1; 2; 3; 4; 5]; %Matrix for our coincidence points, not sure if correct

%nump = [-3.969*Utdi 0 0 0];
%denp = [-0.9126005 -2.822625 2.91 1];

nump = [0 0 0 -3.969*Utdi]; %Numerator of the transfer function
denp = [1 2.91 -2.822625 -0.9126005]; %Denominator of the transfer function
plant = tf(nump, denp, Ts);
plant = tf(plant); %Define the transfer function for the plant

nump = get(plant,'num'); nump = nump{:}; %Numerator of the plant
denp = get(plant,'den'); denp = denp{:}; %Denominator of the plant

numm = get(plant,'num'); numm = numm{:}; %Numerator of the model (model = plant in this case)
denm = get(plant,'den'); denm = denm{:}; %Denominator of the model (model = plant in this case)


plant = ss(A,B,C,D);
plant = c2d(plant,Ts);
model = plant;
stepResp = step(model,[0:Ts:max(P)*Ts]);

pastU = [0; 0; 0]; %Assuming initial conditions
pastYf = [180; 180; 170]; %Randomly chosen values for blood glucose outside of the desired zone during the day, used for the free response
pastYm = [180; 180; 170]; %Initially set to plant, not sure what to set this as right away (may need to do some luenbereger observer state estimation for this)??? These are the ACTUAL outputs given the predicted inputs applied
deltaU = zeros(5,1); %Will hold the values for the predictions, only get values over the control horizon
theta = zeros(5, 5); %Declaration of the theta matrix (I'm not sure if these dimentions are correct)
Yf = zeros(5, 1); %Declaration of the free response matrix
uuk = zeros(288,1);

for i = 1:length(P)
    theta(i,:) = [stepResp(P(i):-1:max(P(i)-6,1))',zeros(1,5-P(i))];
end
S = stepResp(P)

for k = 1:288
    reftraj = refTraj(setPoint(k:k+4),Ts,Tref,pastYm(1,1),k);
    for kk = 1:max(P)
        ymfree(kk) = numm.*pastU-denm(2:ndenm+1).*pastYf; %changed (2:ndenm+1) to (2:length(denm)+1)
        pastYf = [ymfree(kk);pastYf(1:length(pastYf)-1)];
        pastU = [pastU(1);pastU(1:length(pastU)-1)];
    end
    dutraj = theta\(reftraj-pastYf(P)');
    uu(k) = dutraj(1) + uuk(k-1); 
    
    uppast = [uu(k);uppast(1:length(uppast)-1)];
    yp(k+1) = -denp(2:ndenp+1)*yppast+nump*uppast; %changed (2:ndenp+1) to (2:denp+1)
    yppast = [yp(k+1);yppast(1:length(yppast)-1)];
    
    umpast = [uu(k);umpast(1:length(umpast)-1)];
    ym(k+1) = -denm(2:ndenm+1)*ympast+numm*umpast; %changed (2:ndenm+1) to (2:denm+1)
    ympast = [ym(k+1);ympast(1:length(ympast)-1)];
end