%Liliko Uchida
%ME0021 w/ Professor Wendell and Professor Koushyar
%Biomechanics Project

%Importing Data, Establish Constants
clear all
close all
clc

%Importing Data
load("HBP_input_stance_corrected.mat")

%Establish Constants
foot_mass = 0.86;
shank_mass = 2.73;
thigh_mass = 8.92;
foot_inertia = 0.0037;
shank_inertia = 0.0237;
thigh_inertia = 0.1903;
g = 9.81; 

%Equations of Motion
%Ankle
for i = 1:79
    RAx(i) = -GRF(i,1) + foot_mass*foot_COM_la(i,1);
    RAz(i) = g*foot_mass - GRF(i,2) + foot_mass*foot_COM_la(i,2); 
    Ma(i) = foot_inertia*foot_aa(i) - GRF(i,2)*(COP(i,1)-foot_COM(i,1))...
        -GRF(i,1)*foot_COM(i,2) + RAx(i)*(RANKLE(i,2)-foot_COM(i,2))...
        + RAz(i)*(foot_COM(i,1) - RANKLE(i,1));
end

%Shank
for i = 1:79
    RKx(i) = RAx(i) + shank_mass*shank_COM_la(i,1);
    RKz(i) = RAz(i) + g*shank_mass  + shank_mass*shank_COM_la(i,2); 
    Mk(i) = Ma(i) + RKx(i)*(RKNEE(i,2) - shank_COM(i,2))...
        +RKz(i)*(shank_COM(i,1) - RKNEE(i,1)) + RAx(i)*(shank_COM(i,2) - RANKLE(i,2))...
        + RAz(i)*(RANKLE(i,1) - shank_COM(i,1)) + shank_inertia*shank_aa(i);
   
end

%Hip
for i = 1:79
    RHx(i) = RKx(i) + thigh_mass*thigh_COM_la(i,1);
    RHz(i) = RKz(i) + g*thigh_mass  + thigh_mass*thigh_COM_la(i,2); 
    Mh(i) = Mk(i) + RHx(i)*(RTROCH(i,2) - thigh_COM(i,2))...
        +RHz(i)*(thigh_COM(i,1) - RTROCH(i,1)) + RKx(i)*(thigh_COM(i,2) - RKNEE(i,2))...
        + RKz(i)*(RKNEE(i,1) - thigh_COM(i,1)) + thigh_inertia*thigh_aa(i);
end 


%Plotting Forces and Moments for Each Joint
figure(1)
    subplot(3,1,1)
        plot(RAx)
        title()
        xlabel()
        ylabel()
      subplot(3,1,2)
      
figure(2)
    subplot(3,1,1)
        plot(RKx)
        title()
        xlabel()
        ylabel()
    subplot(3,1,2)
    
figure(3)
    subplot(3,1,1)
        plot(RHx)
        title()
        xlabel()
        ylabel()
      subplot(3,1,2)

    
