
clear
clc
close all

Ts=1e-3; % Sampling time 1ms
Tsim=20; % Simulation time 20s
t=0:Ts:Tsim-1*Ts;
N=Tsim/Ts;

% All model are continuous model

% Plant matrix without nonlinear element tanh()
A=[0 1
   -50 -4.0];
B=[0
   50];
C=[1 0];
D=0;

% Disturbance input matrix
Bd=[0
   50];

% Initialize of plant state variable
dotx(1:2,1:N)=0;
x(1:2,1:N)=0;

% Observer matrix without nonlinear element tanh()
Aob=[0 1
   -50 -4.0]
Bob=[0
   50]
Cob=[1 0]
g1=Bob;
g2=Bob;

% Initialize of observer state variable
dotz(1:1,1:N)=0;
z(1:1,1:N)=0;

% Feedback gain L
Lob=[0 5]*0.02

% Estimated disturbance
dhat(1:1,1:N)=0;

% Difinition of plant input at 5s 
u(1:N/4)   = 0;
u(N/4+1:N) = 1;

% Difinition of disturbance input at 10s 
d(1:N/2)   = 0;
d(N/2+1:N) = 1*1;

% Simulation
for i=1:N-1
    % Caluculation of observer and estimation of disturbance
    fx=A*x(:,i)+[0;-5*tanh(2*x(2,i))];
    px=Lob*x(:,i);
    dotz(1,i+1)=-Lob*g2*z(:,i)-Lob*(g2*px+fx+g1*u(i));
    z(:,i+1)=z(:,i)+dotz(:,i+1)*Ts;
    dhat(:,i+1) = z(:,i+1)+px;
    
    % Caluculation of plant
    dotx(:,i+1)=A*x(:,i)+[0;-5*tanh(2*x(2,i))]+B*u(i)+Bd*d(i);
    x(:,i+1)=x(:,i)+dotx(:,i+1)*Ts;
    
end


% Plot data
figure
hold on
grid on
plot(t,x(1,:)) % position
xlabel('Time [s])')
ylabel('position')
title('Position');

figure
hold on
grid on
plot(t,d) % Actual disturbance
plot(t,dhat) % Estimated disturbance
xlabel('Time [s])')
ylabel('input')
title('Disturbance estimation');
legend('Actual disturbance', 'Estimated disturbance')
ylim([-0.2 1.2])













