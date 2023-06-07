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
T_f = 10;
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
T_f      = 10;    % final simulation time
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

LTI_lin_pos = ss(A_lin, B_lin, C_lin_pos, D_lin_pos);

G_lin_pos = tf(LTI_lin_pos);

disp ('Transfer function of the linearized model (Cart Position):')
zpk(G_lin_pos)
%% Controller 
%Proviamo per cancellazione
R = (s-5.642)/(s-5.153);

rlocus(minreal(R*G_lin_pos))
grid on;
K = 0.00295;
R = K*R;
%% Controllore in the space state sistem

R_lti   = ss(R);

A_ctrl  = R_lti.A;
B_ctrl  = R_lti.B;
C_ctrl  = R_lti.C;
D_ctrl  = R_lti.D;
