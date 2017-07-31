% Define the system
A=[0 1;0 0];
B=[0;1];
h=0.001;
% the initial condition will be 
x=[1;0];
history=[]; % This is extremly inefficient, but the simulation is very small
history=[history x];
tmax=0.12; %Simulation stop time
tau1=0.0001; %Delay induced by the network
u_old=0; % The previos control signal
% Now discretize the system to perform the simulation
[Ad,Bd]=c2d(A,B,h);
[Ad_delay,Bd_delay]=c2d(A,B,tau1); %Evolution of the sistem for the delay
[Ad_end,Bd_end]=c2d(A,B,h-tau1); %evolution of the system up to the end of the period

A_e=[Ad Ad_end*Bd_delay; 0 0 0];
B_e=[Bd_end;1];


%Design a controller

K=-acker(A_e,B_e,[0.7,0.1+0.4i,0.1-0.4i])
for i=0:h:tmax
    u=K*[x;u_old]; %the control signal for the period h, we sample and we send the signal, but there is no update due to network delay
    x=Ad_delay*x+Bd_delay*u_old;
    %now we update the control signal
    x=Ad_end*x+Bd_end*u; %evolve the system state
    u_old=u; % prepare next step
    history=[history x]; %update the history to plot it later
end

plot(0:h:tmax+h,history(1,:));
xlabel('Time [s]');
ylabel('x[1]');

%% Experiment (iii): Networked control loop, a delay of 0.0005s is induced
x=[1;0]; %Initial state
history=[]; % This is extremly inefficient, but the simulation is very small
history=[history x];
tmax=0.12; %Simulation stop time
tau2=0.0005; %Delay induced by the network
u_old=0; % The previos control signal
%% 
% Now discretize the system to perform the simulation
[Ad_delay,Bd_delay]=c2d(A,B,tau2); %Evolution of the sistem for the delay
[Ad_end,Bd_end]=c2d(A,B,h-tau2); %evolution of the system up to the end of the period
%%
% The controller keeps the same, we still do not now how to design this
% controller
 
for i=0:h:tmax
    u=K*[x;u_old]; %the control signal for the period h, we sample and we send the signal, but there is no update due to network delay
    x=Ad_delay*x+Bd_delay*u_old;
    %now we update the control signal
    x=Ad_end*x+Bd_end*u; %evolve the system state
    u_old=u; % prepare next step
    history=[history x]; %update the history to plot it later
end

plot(0:h:tmax+h,history(1,:));
xlabel('Time [s]');
ylabel('x[1]');

%%
% The system is still stable, but there is an oscillation.

%% Experiment (iv): Networked control loop, an alternative delay if 0.0001s and 0.0005s is induced
% This case requires a two steps simulation with a selection of the delay.
x=[1;0]; %Initial state
history=[]; % This is extremly inefficient, but the simulation is very small
history=[history x];
tmax=0.12; %Simulation stop time
tau1=0.0001;
tau2=0.0005; %Delay induced by the network
u_old=0; % The previos control signal
%% 
% Now discretize the system to perform the simulation, 
[Ad_delay1,Bd_delay1]=c2d(A,B,tau1); %Evolution of the sistem for the delay
[Ad_end1,Bd_end1]=c2d(A,B,h-tau1); %evolution of the system up to the end of the period
[Ad_delay2,Bd_delay2]=c2d(A,B,tau2); %Evolution of the sistem for the delay
[Ad_end2,Bd_end2]=c2d(A,B,h-tau2); %evolution of the system up to the end of the period
%%
% The controller keeps the same, we still do not now how to design this
% controller
a=1; 
for i=0:h:tmax
    u=K*[x;u_old]; %the control signal for the period h, we sample and we send the signal, but there is no update due to network delay
    x=a*(Ad_delay1*x+Bd_delay1*u_old)+~a*(Ad_delay2*x+Bd_delay2*u_old);
    %now we update the control signal
    x=a*(Ad_end1*x+Bd_end1*u)+~a*(Ad_end2*x+Bd_end2*u); %evolve the system state
    u_old=u; % prepare next step
    a=~a; %Change system
    history=[history x]; %update the history to plot it later
end

plot(0:h:tmax+h,history(1,:));
xlabel('Time [s]');
ylabel('x[1]');
