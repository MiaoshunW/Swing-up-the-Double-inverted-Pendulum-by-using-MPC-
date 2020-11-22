clear;
close all;

x0 = 0.0; %initial cart position
alpha0 = pi*0.999; %initial pendulum position

Ts = 0.001; %sample time

% gravity acceleration
g = 9.81; % m/s^2

% length of pendulum:
l = 0.5 ; % m
b = 0.02; % m

% model of motor behaviour: PT1
T1 = 0.0395; % s

%% linearized system in top position

c = 6/(4*l + b^2/l);

% system matrices:
A = [   0           1       0       0 
        g*c        -0.1     0       c/T1
        0           0       0       1
        0           0       0       -1/T1    ];

b = [   0
        -c/T1
        0
        1/T1     ];
    
cT = [   0   0   1   0   ];

%% explicite Ackermann formula

% all poles in px
px = -8;

q1 = b;
q2 = A*q1;
q3 = A*q2;
q4 = A*q3;
Q = [  q1  q2  q3  q4  ]; %controllability matrix

t1 = [0 0 0 1]*inv(Q); %last row of inverse
t2 = t1*A;
t3 = t2*A;
t4 = t3*A;
t5 = t4*A;
T = [ t1; t2; t3; t4; t5 ];

syms s;
p = fliplr(sym2poly((s-px)^4)); %characteristic polynom
% p = [p0 p1 p2 p3 p4 1]

fT = -p*T;
%fT = -acker(A,b,px*ones(4,1)); %built-in Ackermann formula

R = -inv(cT*inv(A+b*fT)*b); %Prefilter

%% observer design

Cm = [1 0 0 0
      0 0 1 0]; %measurement of pendulum and cart position
obs = ss(A,b,Cm,[0;0]); %state space model

H = place(obs.a',obs.c',-abs(eig(obs.a))*6-1)'; %observer design
%observer poles 6 times as fast as plant poles

