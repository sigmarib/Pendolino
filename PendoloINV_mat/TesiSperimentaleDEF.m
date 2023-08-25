clear
clc
close all
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
%% Parametri
p_M = 0.61;          % massa del carrello [kg]
p_ma = 0.116;        % massa asta [kg]
p_mb = 0.05;         % massa posta al termine dell'asta [kg]
p_l = 0.45;          % lunghezza dell'asta [m]
p_k = 0;             % rigidità della molla [N/m]
p_b = 0.1;           % coefficiente di smorzamento [Ns/m]
p_g = 9.81;          % accellerazione di gravità [m/s^2]
p_Ja = (1/12)*p_ma*p_l^2;% Inertia p_ma
p_Jb = 0;            % Inertia p_mb
% Initial Parameters
init_v0 = 0;              % to SLX
init_x0 = -1;             % to SLX
init_omega0 = 0;          % to SLX
init_theta0 = pi/12;      % to SLX
init_thetaRef = 0;        % to SLX
init_positionRef = 0;     % to SLX
Ts = 0.002;               % to SLX

Tf = 15;
%% Linearizated model
% states: x_1: linear position
%         x_2: angular position
%         x_3: linear speed
%         x_4: angular speed
% MATRICE di Masse
Matrix_Masse_lin = [(p_ma/2+p_mb)*p_l   (p_ma/4+p_mb)*p_l^2+p_Ja+p_Jb;
                        p_M+p_ma+p_mb         (p_ma/2+p_mb)*p_l       ];
% MATRICE Dinamica:  A = []
Matrix_A_1 = [0 0 1 0;
              0 0 0 1];
Matrix_A_2 = (Matrix_Masse_lin^-1) * [0      (p_ma/2+p_mb)*p_g*p_l  0    0;
                                      -p_k           0             -p_b  0];
Matrix_A_lin = [Matrix_A_1;
                Matrix_A_2];
% MATRICE di distribuizioine degli ingressi:  B = []
Matrix_B_1 = [0;
              0];
Matrix_B_2 = (Matrix_Masse_lin^-1) * [0;
                                      1];
Matrix_B_lin=[Matrix_B_1;
              Matrix_B_2];

% MATRICE di distribuzione delle uscite:  C = []
Matrix_C_angle_lin = [0 1 0 0];
%Legame algebrico ingresso–uscita:  D = []
Matrix_D_angle_lin = 0;

%Volendo stabilizzare anche il carrello avremo una seconda uscita y = x_1
Matrix_C_pos_lin = [1 0 0 0];
%Legame algebrico ingresso–uscita:  D = []
Matrix_D_pos_lin = 0;

%convert dynamic system models to state-space model form.
LTI_angle_lin = ss(Matrix_A_lin, Matrix_B_lin, Matrix_C_angle_lin, Matrix_D_angle_lin);
LTI_position_lin = ss(Matrix_A_lin, Matrix_B_lin, Matrix_C_pos_lin, Matrix_D_pos_lin);

%% Transfer functions

G_lin_ang = tf(LTI_angle_lin);
G_lin_ang  = minreal(zpk(G_lin_ang));

G_lin_pos = tf(LTI_position_lin);
G_lin_pos = minreal(zpk(G_lin_pos));

disp ('Transfer function of the linearized model (Angle):')
zpk(G_lin_ang)

disp ('Transfer function of the linearized model (Cart Position):')
zpk(G_lin_pos)

%% Angle Control

f_R_angle = -(s+3.1)*(s+6.8)/(s*(s+66));
f_K_angle_Gain = 357;

f_KR_angle = f_K_angle_Gain * f_R_angle;

% Funzione d'Anello
f_L_angle = minreal(f_KR_angle * G_lin_ang);

% Funzione di Sensitività 'S'
f_Sensitivity_S_angle = minreal(1/(1+f_L_angle));

% Funzione di Sensitività complementare F''
f_Sensitivity_F_angle = minreal(f_L_angle/(1+f_L_angle));

% Funzione di Sensitività del controllo 'Q'
f_Sensitivity_Q_angle = minreal(f_KR_angle/(1+f_L_angle));

%% Discretizzazione Regolatore Angolo

f_KR_angle_LTI = ss(f_KR_angle);
A_CTRL_angle = f_KR_angle_LTI.A;
B_CTRL_angle = f_KR_angle_LTI.B;
C_CTRL_angle = f_KR_angle_LTI.C;
D_CTRL_angle = f_KR_angle_LTI.D;

% algoritmo per la discretizzazione
alpha_angle = 0.5; % Discretizzazione tramite Tustin
I_angle     = eye(size(A_CTRL_angle, 1)); % matrice identità
% matrici del regolatore a tempo discreto
A_alpha_angle = I_angle + Ts * (I_angle - alpha_angle * A_CTRL_angle * Ts)^-1 * A_CTRL_angle;
B_alpha_1_angle = (1 - alpha_angle) * Ts * (I_angle - alpha_angle * A_CTRL_angle * Ts)^-1 * B_CTRL_angle;
B_alpha_2_angle = alpha_angle * Ts * (I_angle - alpha_angle * A_CTRL_angle * Ts)^-1 * B_CTRL_angle;

% Z TO ARDUINO TBD
% Continuos to Discrete
arduino_R1 = c2d(f_KR_angle, Ts);  %this is a short display, go to var panel and extract more decimals


% Automated function to arduino
% set 1 to display output and copy, 0 to block
returnArduinoCode(arduino_R1.Numerator, arduino_R1.Denominator, "_a" ,1)

%% Regolatore Posizione 
cascade_f_R_position = (s+0.38)/((s + 1.5) * (s + 3.5));
cascade_f_K_position_Gain =0.34;

%cascade_f_K_position_Gain = 0.0143;

%Regolatore con pid = 2 zeri e 2 poli

%cascade_f_R_position = (2.6*s+1)*(2.6*s+1)/s/(0.1*s+1);

cascade_f_KR_position = cascade_f_K_position_Gain * cascade_f_R_position;

cascade_f_G_XTheta = minreal((G_lin_pos)/(G_lin_ang));

%% Prog R_pos
%Generalizzazione del predittore di smith per compensazione di zeria a
%fase non minima
zpk(cascade_f_G_XTheta)

%    -0.36944 (s+5.153) (s-5.153)
%  = ----------------------------
%              s^2

Np = (s-5.153);
Nm = (s+5.153);
D = s^2;

%Tramite la modifica realizzativa del blocco P possiamo riscrivere la
%fdt del plant in un altro modo, introducendo da prima:
Np_m = (s+5.153);

cascade_f_G_XTheta_Smith = minreal(0.36944 * Np_m*Nm/D);
zpk(cascade_f_G_XTheta_Smith)


%    -0.36944 (s+5.153)^2
%  = --------------------
%          s^2

%Possiamo ora progettare il regolatore sulla base del plant con tutti i
%poli nel semipiano sinistro a parte reale negativa

R_position_Smith = 0.0143*(2.6*s+1)^2/s/(0.1*s+1);

%% Pre-filtro
%Dato che tau_p = 1/w_c
%e il Ta <= 3/re -> re(=delta*w_c) = 3/Ta = 3/1.2
R_pf = 1/((1.2*0.7/3)*s+1);
R_pf_LTI = ss(R_pf);

A_pf = R_pf_LTI.A;
B_pf = R_pf_LTI.B;
C_pf = R_pf_LTI.C;
D_pf = R_pf_LTI.D;

alpha_angle = 0.5; % Discretizzazione tramite Tustin
I_angle     = eye(size(A_pf, 1)); % matrice identità
% matrici del regolatore a tempo discreto
A_pf_alpha = I_angle + Ts * (I_angle - alpha_angle * A_pf * Ts)^-1 * A_pf;
B_pf_1_alpha = (1 - alpha_angle) * Ts * (I_angle - alpha_angle * A_pf * Ts)^-1 * B_pf;
B_pf_2_alpha = alpha_angle * Ts * (I_angle - alpha_angle * A_pf * Ts)^-1 * B_pf;

%% 

% Funzione d'Anello
%(Codice buono senza smith)cascade_f_L_position = minreal(cascade_f_KR_position * cascade_f_G_XTheta);

cascade_f_L_position = minreal(R_position_Smith * cascade_f_G_XTheta);

f_KR_position_LTI = ss(cascade_f_KR_position);
A_CTRL_position = f_KR_position_LTI.A;
B_CTRL_position = f_KR_position_LTI.B;
C_CTRL_position = f_KR_position_LTI.C;
D_CTRL_position = f_KR_position_LTI.D;


% algoritmo per la discretizzazione
cascade_alpha_position = 0.5; % Discretizzazione tramite Tustin
cascade_I_position  = eye(size(A_CTRL_position, 1)); % matrice identità
% matrici del regolatore a tempo discreto
cascade_A_alpha_position = cascade_I_position + Ts * (cascade_I_position - cascade_alpha_position * A_CTRL_position * Ts)^-1 * A_CTRL_position;
cascade_B_alpha_1_position = (1 - cascade_alpha_position) * Ts * (cascade_I_position - cascade_alpha_position * A_CTRL_position * Ts)^-1 * B_CTRL_position;
cascade_B_alpha_2_position = cascade_alpha_position * Ts * (cascade_I_position - cascade_alpha_position * A_CTRL_position * Ts)^-1 * B_CTRL_position;

% Z TO ARDUINO TBD
% Continuos to Discrete
arduino_R2 = c2d(cascade_f_KR_position, Ts);  %this is a short display, go to var panel and extract more decimals


% Automated function to arduino
% set 1 to display output and copy, 0 to block
returnArduinoCode(arduino_R2.Numerator, arduino_R2.Denominator, "_p" ,1)
