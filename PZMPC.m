%This is the script that will run the mpc simulation

%NOTE: THE PZMPC SET OF SUMS SHOWN IN THE PAPER STILL NEEDS TO BE
%IMPLEMENTED (PROBABLY)

%Questions/Concerns:
    %Why not test deltau using the boundary least squares then test again
        %using non boundary least squares with a separately calculated zone
        %excursion addition to our inputs.
    %Do we need to use the Luenberger Observer? It requires actual outputs
        %obtained from a continuous glucose monitor and looks to minimize
        %the difference between that output and the observed state output.
        %This observer is used for the excursion function, so maybe just
        %use the simple state space representation for the zone excursion
        %if necessary.
    %It looks like theta is only computed once before the simulation is ran
        %to calculate DeltaU in the book. The only things that are
        %calculated each loop in the simulation are the reference
        %trejectory over the prediction horizon, the free response outputs,
        %and we have to reassign the newly calculated outputs and inputs
        %for the subsequent steps. It also mentions resimulating the model
        %and the plant for 1 step but im not sure this is applicable.
    %The paper uses a cosine function to get the smooth transition between
        %daytime and nighttime and vice-versa, but it's hard to say exactly
        %how this is done, so for simplicity, I am linearizing this
        %transition.
    %What should be put for pastYm initial condition, the past output
        %actaully calculated by the model (output of our mpc controller 
        %given the most recently applied input)? Do we need to use the
        %Luenberger Observer for this?
    %It seems like we're using the free response to calculate the outputs
        %of the model each time we get a new predicted input, but it's hard
        %to say what the desired initial conditions are. Also, are the
        %model inputs and the plant inputs different? Do we need different
        %"u" values for the free response when doing the plant calculations
        %and the model calculations? For now, I will assume the previous
        %inputs used for the plant and the model and the same.
    %Not sure if it is correct to take pastYm(1,1) as the last input for
        %the DeltaU function, because this is what is used for the refTraj
        %calculation in the DeltaU function for theta. But it looks like in
        %the textbook, the same output is used the entire time for the
        %calculation of the reference trajectory.
    %One of the big issues right now is that the inputs are not bounded,
        %and they need to have an upper and lower limit  (during night the
        %lower limit is 0 and upper is 1.5, but during the day the lower
        %limit is 0 and upper limit is pump maximum which is not given).
        
    %I need to make sure that the theta and Yf matrices are only calculated
        %once and not each loop, that's what's making our output explode
        %and the input bounce so wildly


%Initial conditions:
%Hu = 5; %Control horizon
%Hp = 9; %Prediction horizon
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

pastU = [0; 0; 0]; %Assuming initial conditions
pastYf = [180; 180; 170]; %Randomly chosen values for blood glucose outside of the desired zone during the day, used for the free response
pastYm = [180; 180; 170]; %Initially set to plant, not sure what to set this as right away (may need to do some luenbereger observer state estimation for this)??? These are the ACTUAL outputs given the predicted inputs applied
deltaU = zeros(5,1); %Will hold the values for the predictions, only get values over the control horizon


%Simulation (runs for 24 hours, starting at 4pm); AS IT STANDS RIGHT NOW,
%WE ARE NOT TAKING INTO CONSIDERATION THE EXCURSION ZONE BOUNDS
for i = 1:288 %This will simulate 288 5 minute time intervals to cover a 24 hour day
    deltaU = DeltaU(pastU, pastYf, Utdi, setPoint(i:i+4,1), Ts, Tref, pastYf(1,1));
    newIn = deltaU(1,1) + pastU(1,1)
    pastU = circshift(pastU,1);
    pastU(1,1) = newIn;
    
    newYf = freeResponse(pastYf(1,1), pastYf(2,1), pastYf(3,1), pastU(3,1), Utdi); %Calculate new output for plant
    pastYf = circshift(pastYf,1); %Update past outputs for the plant
    pastYf(1,1) = newYf;
    
    newYm = freeResponse(pastYm(1,1), pastYm(2,1), pastYm(3,1), pastU(3,1), Utdi); %Calculate the new output given the newly shifted input values
    pastYm = circshift(pastYm,1); %Here we update the past outputs for the model
    pastYm(1,1) = newYm;
end