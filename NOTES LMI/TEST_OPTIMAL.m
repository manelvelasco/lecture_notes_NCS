Anominal = [0 1 0;0 0 1;0 0 0];
B = [0;0;1];
Q = eye(3)
R = 1;
Y = sdpvar(3,3);
L = sdpvar(1,3,'full');
A1 = Anominal;A1(1,3) = -0.1;
F = [Y >= 0];
F = [F, [-A1*Y-B*L + (-A1*Y-B*L)' Y L';Y inv(Q) zeros(3,1);L zeros(1,3) inv(R)] >= 0];
optimize(F,-trace(Y))
K = value(L)*inv(value(Y));
K

lqr(A1,B,Q,R)
KLQR=lqr(A1,B,Q,R)
%%
%Prueba discreto
Anominal = [0 1 0;0 0 1;0 0 0];
B = [0;0;1];
Q = eye(3)
R = 1;
Y = sdpvar(3,3);
L = sdpvar(1,3,'full');
A1 = Anominal;A1(1,3) = -0.1;
F = [Y >= 0];
F = [F, [Y (A1*Y+B*L)' L' Y;A1*Y+B*L Y zeros(3,4);L [0 0 0] inv(R) 0 0 0; Y zeros(3,3) zeros(3,1) inv(Q)]> 0];
optimize(F,-trace(Y))
K = value(L)*inv(value(Y));
K
KLQR=dlqr(A1,B,Q,R)

%%
%Prueba discreto FULL
Anominal = [0 1 0;0 0 1;0 0 0];
B = [0;0;1];
Q = 2*eye(3)
R = 2;
N=[1;1;1];
Y = sdpvar(3,3);
L = sdpvar(1,3,'full');
A1 = Anominal;A1(1,3) = -0.1;
F = [Y >= 0];
F = [F, [Y (A1*Y+B*L)' Y L';A1*Y+B*L Y zeros(3,4);[Y zeros(3,3);L zeros(1,3)] inv([Q N;N' R])]> 0];
optimize(F,-trace(Y))
K = value(L)*inv(value(Y));
K
KLQR=dlqr(A1,B,Q,R,N)

%%
%Prueba de veracidad

syms   a
A=[0 1;0 0]
B=[0 ;1]
Q=2*eye(2)
R=10
N=[1;1]
[K,S,e]=dlqr(A,B,Q,R,N);
digits(4)
%vpa(-Q-K'*(B'*S*B+R)*K+N*R^(-1)*N' )
vpa(-Q -K'*(B'*S*B+R)*K  )

%K=(B'*S*B+R)^(-1)*(B'*S*A+N');
CL=A-B*K;
CL'*S*CL-S
-Q-K'*R*K+K'*N'+N*K
%%
clear all
syms A B K Q R S N At Bt Kt Nt
CL=A-B*K;
CL'*S*CL-S
At*S*B*K-(At*S*B*K)'+K'*B'*S*B*K-Q+K'*(B'*S*B+R)*K

%%

A1 = Anominal;A1(1,3) = -0.1;
A2 = Anominal;A2(1,3) =  0.1;
Y = sdpvar(3,3);
L = sdpvar(1,3,'full');
F = [Y >= 0];
F = [F, [-A1*Y-B*L + (-A1*Y-B*L)' Y L';Y inv(Q) zeros(3,1);L zeros(1,3) inv(R)] >= 0];
F = [F, [-A2*Y-B*L + (-A2*Y-B*L)' Y L';Y inv(Q) zeros(3,1);L zeros(1,3) inv(R)] >= 0];
optimize(F,-trace(Y))
K = value(L)*inv(value(Y));
K
KLQR