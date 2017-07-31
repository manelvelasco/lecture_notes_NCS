clear all ;close all
%A matrix
A=[1 0.5; 0.5 2];
B=[2 1;0 2.5]
%loop over t values
rb=0:100;
r=0:100;
t=(0:100)/100;
for j=0:100
   x=[cos(2*pi*t(j+1));sin(2*pi*t(j+1))];
   r(j+1)=x'*A*x; %radius
   rb(j+1)=x'*B*x; %radius
   hold on;
end
polar(2*pi*t,r);
hold on
polar(2*pi*t,rb,'r');