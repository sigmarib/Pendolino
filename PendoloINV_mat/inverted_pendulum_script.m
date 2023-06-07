clear 
clc
close all
%% Parametri
M = 0.61;
mb = 0.05;
ma = 0.116;
l = 0.45;
k = 0;
b = 0.1;
g = 9.81;
Ja = (1/12)*ma*l^2;
Jb = 0;

v0 = 0;
x0 = 0;
omega0 = 0;
theta0= pi/12;

T_f = 15;
%% General settings
font_size   = 18;
line_width  = 2;
plot_x0     = 500;
plot_y0     = 300;
plot_width  = 800;
plot_height = 400;
%% Transfer function variable
s = tf('s');
%% Simulink settings
MaxStep  = 1e-2; % solver max step size
RelTol   = 1e-2; % solver relative tolerance   
%% Linearizated model
% states: x_1: linear position
%         x_2: angular position
%         x_3: linear speed
%         x_4: angular speed
%Matrice di masse
M_lin = [(ma/2+mb)*l (ma/4+mb)*l^2+Ja+Jb;
            M+ma+mb (ma/2+mb)*l];
%Matrice dinamica
A_1 = [0 0 1 0;
        0 0 0 1];
A_2 = (M_lin^-1)*[0 (ma/2+mb)*g*l 0 0;
                    -k 0 -b 0];
A_lin = [A_1;
         A_2];
%Matrice di distribuzione degli ingressi
B1 = [0;0];
B2 = (M_lin^-1)*[0; 1];
B_lin=[B1; B2];
%Matrice di distribuzione delle uscite
C_lin_theta = [0 1 0 0];
%Legame algebrico ingresso–uscita
D_lin_theta = 0;

%Volendo tabilizzare anche il carrello avremo una seconda uscita y = x_1
C_lin_pos = [1 0 0 0];
D_lin_pos = 0;

%convert dynamic system models to state-space model form.
LTI_lin_ang = ss(A_lin, B_lin, C_lin_theta, D_lin_theta);

LTI_lin_pos = ss(A_lin, B_lin, C_lin_pos, D_lin_pos);

%Trasfer function model
G_lin_ang = tf(LTI_lin_ang);

G_lin_pos = tf(LTI_lin_pos);

disp ('Transfer function of the linearized model (Angle):')
zpk(G_lin_ang)

disp ('Transfer function of the linearized model (Cart Position):')
zpk(G_lin_pos)

%% Controller 

%Inseriamo il meno poichè è impossibile stabilizzarlo per il luogo diretto
%R = (s+4)(s+4.5)/s/(s+8); -> causa di oscillazioni

%Controller
%R_ang = -(s+1)*(s+4.9)/s/(s+12);

R_ang = -(s+0.1289)*(s+5)/s/(s+12);

%Gain per entrare nella zona di Re>0
K = 28;

%s/3,3,9

R_ang = K*R_ang;

%Sistema esteso
G_e_ang = minreal(G_lin_ang*R_ang);

rlocus(G_e_ang);
grid on;
 %% Tuning Rete anticipatrice
% %Trovo lo smorzamento per una sovraelongazione minore di 10
% delta_star = 0.52834; % per una sovraelong max di 10
% delta = 0.55;
% Mf = delta*100;
% 
% [M_a,P,W]=bode(G_e_ang);
% [V,i]=min(abs(W-100));
% GeWcd=M_a(i);
% ArgGeWcd=P(i);
% Pd=-180+60-ArgGeWcd;
% Md=1/GeWcd;
% Pd=Pd*pi/180; 
% 
% 
% 
% tau=(Md-cos(Pd))/(100*sin(Pd));
% alpha=(cos(Pd)-1/Md)/(100*sin(Pd))/tau; 
% 
% %Prova 
% 
% R_ant = (1+tau*s)/(1+alpha*tau*s);
% 
% R = minreal(R_ang*R_ant);
%% Fdt pos/ang
G_pos_ang = minreal(G_lin_pos/G_lin_ang);
rlocus(G_pos_ang)
grid on

R_pos_ang = (s+15)/(s-5.153)/(s+10);

L_pos_ang = minreal(G_pos_ang*R_pos_ang);
rlocus(-L_pos_ang)
grid on

R_pos_ang = R_pos_ang;

R = minreal(R_ang);
%%  digitale

G_s = minreal(G_e_ang/(1+G_e_ang));

Ts = 0.04;
G_est = G_s*exp(-3/2*Ts*s);

%% Controllore in the space state sistem

R_lti   = ss(R);

A_ctrl  = R_lti.A;
B_ctrl  = R_lti.B;
C_ctrl  = R_lti.C;
D_ctrl  = R_lti.D;

%Algoritmo di discretizzazione
alpha   = 0.5;    %Trustin
I       = eye(size(A_ctrl, 1));

%matrici regolatore tempo discreto
A_alpha   = I + Ts*(I - alpha*A_ctrl*Ts)^-1*A_ctrl;
B_alpha_1 = (1 - alpha)*Ts*(I - alpha*A_ctrl*Ts)^-1*B_ctrl;
B_alpha_2 = alpha*Ts*(I - alpha*A_ctrl*Ts)^-1*B_ctrl;

% %% Sentitivity functions 
% L_ang = minreal(G_lin_ang*R_ang);
% 
% S = minreal(1/(1+L_ang));
% F = minreal(L_ang/(1+L_ang));
% Q = minreal(R_ang/(1+L_ang));
% 
% %Tempo di assestamento massimo (entro il 5%) = 1.2s
% 
% 
% %checking the requirements
% mindecay   = 2.5;
% mindamping = 0.10;
% maxfreq    = inf;
% Goals      = TuningGoal.Poles(mindecay, mindamping, maxfreq);
% figure(2)
% viewGoal(Goals, F);
% 
% figure(3)
% rlocus(F)
% grid on
