%This is the script that is modeled after the book. It is similar to my own
    %implementation, but everything is done in one file, and the plant and
    %model are implemented as transfer functions with numerators and
    %denominators handled accordingly. I skip this process in my own
    %implementation.

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
P = round(0.8*Tref/Ts); %Matrix for our coincidence points, not sure if correct
M = 1; %Default control horizon is set to 1

%nump = [-3.969*Utdi 0 0 0];
%denp = [-0.9126005 -2.822625 2.91 1];

nump = [0 0 0 -3.969*Utdi]; %Numerator of the transfer function
denp = [1 2.91 -2.822625 -0.9126005]; %Denominator of the transfer function
plant = tf(nump, denp, Ts);
plant = tf(plant); %Define the transfer function for the plant

nump = get(plant,'num'); nump = nump{:}; %Numerator of the plant
denp = get(plant,'den'); denp = denp{:}; %Denominator of the plant

nnump = length(nump)-1; %Degree of the plant numerator
ndenp = length(denp)-1; %Degree of the plant denominator

model = plant;
model = tf(model);

numm = get(model,'num'); numm = numm{:}; %Numerator of the model (model = plant in this case)
denm = get(model,'den'); denm = denm{:}; %Denominator of the model (model = plant in this case)

nnumm = length(numm)-1; %Degree of model numerator
ndenm = length(denm)-1; %Degree of model denominator

nump = [zeros(1,ndenp-nnump-1),nump];
numm = [zeros(1,ndenm-nnumm-1),numm];

plant = ss(A,B,C,D);
plant = c2d(plant,Ts);
model = plant;
stepResp = step(model,[0:Ts:max(P)*Ts]);

theta = zeros(length(P), M); %Declaration of the theta matrix (I'm not sure if these dimentions are correct)

errfac = exp(-P*Ts/Tref); %Not sure if I'll need this, but used to find the reference trajectory

for i = 1:length(P)
    theta(i,:) = [stepResp(P(i):-1:max(P(i)-M+1,1))',zeros(1,M-P(i))];
end
S = stepResp(P)

tend = 10*Tref; %Duration of the simulation
nsteps = floor(tend/Ts);
tvec = (0:nsteps-1)'*Ts;

setPoint = ones(nsteps+max(P), 1)*110; %To simplify, we assume it's always day time

uu = zeros(nsteps,1); %Initial input
yp = zeros(nsteps,1); %Initial plant output
ym = zeros(nsteps,1); %Initial model output

umpast = zeros(ndenm,1); %Initial past inputs for model
uppast = zeros(ndenp,1); %Initial past inputs for plant
ympast = zeros(ndenm,1); %Initial past outputs for model
yppast = zeros(ndenp,1); %Initial past outputs for plant


%%%%%%%%%%%%%%%%%%%SIMULATION STARTS HERE%%%%%%%%%%%%%%%%%%%%%%%%%%%%


for k = 1:nsteps
    errornow = setPoint(k)-yp(k);
    reftraj = setPoint(k+P) - errornow*errfac;
    yfpast = ympast;
    ufpast = umpast;
    
    for kk = 1:max(P)
        ymfree(kk) = numm(2:nnumm+1)*ufpast-denm(2:ndenm+1)*yfpast;
        yfpast = [ymfree(kk);yfpast(1:length(yfpast)-1)];
        ufpast = [ufpast(1);ufpast(1:length(ufpast)-1)];
    end
    
    if k>1
        dutraj = theta\(reftraj-ymfree(P)');
        uu(k) = dutraj(1) + uu(k-1);
    else
        dutraj = theta\(reftraj-ymfree(P)');
        uu(k) = dutraj(1) + umpast(1);
    end
    
    uppast = [uu(k);uppast(1:length(uppast)-1)];
    yp(k+1) = -denp(2:ndenp+1)*yppast+nump(2:nnump+1)*uppast;
    yppast = [yp(k+1);yppast(1:length(yppast)-1)];
    
    umpast = [uu(k);umpast(1:length(umpast)-1)];
    ym(k+1) = -denm(2:ndenm+1)*ympast+numm(2:nnumm+1)*umpast;
    ympast = [ym(k+1);ympast(1:length(ympast)-1)];
end


%%%%%%%%%%%%%%%%%%%%%%%DISPLAY RESULTS HERE%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


disp('***** Results from simulation : ')
disp(['Tref = ', num2str(Tref), ', Ts = ', num2str(Ts), ', P = ', int2str(P'), '(steps), M = ', int2str(M)])
%diffpm = get(plant-model,'num');
figure
subplot(211)
plot(tvec,yp(1:nsteps),'-',tvec,setPoint(1:nsteps),'--');
grid; title('Plant output (solid) and set point (dashed)')
xlabel('Time')
subplot(212)
stairs(tvec,uu,'-');
grid; title('Input')
xlabel('Time')