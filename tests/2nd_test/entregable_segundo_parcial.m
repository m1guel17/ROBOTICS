% ROBÓTICA - SEGUNDO PARCIAL
% Nombre: Miguel Esteban Flores Sierra
% Código: 2310949
clc; clear all; close all
clear imports
import ETS3.*

%======= TABLA DENAVIT - HARTENBERG =======
%                ESTANDAR
%          _________________________________
%          |-------Z-------|-------X-------|
%          | theta |   d   |   a   | alpha |
%===========================================
%  0_M_1 = |   q1  |  290  |   0   | -pi/2 |
%  1_M_2 = |q2-pi/2|   0   |  270  |   0   |
%  2_M_3 = |   q3  |   0   |  +70  | -pi/2 |
%  3_M_4 = |   q4  |  302  |   0   |  pi/2 |
%  4_M_5 = |   q5  |   0   |   0   | -pi/2 |
%  5_M_6 = |   q6  |  +72  |   0   |   0   |

% DECLARANDO LÍMITES DE CADA ESLABÓN
q1 = [-165 165]*pi/180;
q2 = [-110 110]*pi/180;
q3 = [-110 70]*pi/180;
q4 = [-160 160]*pi/180;
q5 = [-120 120]*pi/180;
q6 = [-400 400]*(pi/180); %*217.8 % (Para Obtener las ±242 revoluciones quitar el comentario al factor de multiplicación)

% CREANDO ESLABONES A PARTIR DE LA TABLA
%               |----------Z---------|---------X--------------|
%               |  theta   |    d    |    a    |     alpha    |
link_1=Revolute('qlim',  q1, 'd', 0.290, 'a',     0, 'alpha',-pi/2, 'offset',     0);
link_2=Revolute('qlim',  q2, 'd',     0, 'a', 0.270, 'alpha',    0, 'offset', -pi/2);
link_3=Revolute('qlim',  q3, 'd',     0, 'a', 0.070, 'alpha',-pi/2, 'offset',     0);
link_4=Revolute('qlim',  q4, 'd', 0.302, 'a',     0, 'alpha', pi/2, 'offset',     0);
link_5=Revolute('qlim',  q5, 'd',     0, 'a',     0, 'alpha',-pi/2, 'offset',     0);
link_6=Revolute('qlim',  q6, 'd', 0.072, 'a',     0, 'alpha',    0, 'offset',     0);

% ENLAZANDO ESLABONES 1 - 2 - 3 - 4 - 5 - 6
links=[link_1;link_2;link_3;link_4;link_5;link_6];

% CREACIÓN DE ROBOT
IRB120=SerialLink(links,'plotopt',{'workspace',[-0.5 0.5 -0.5 0.5 -0.25 1]});
figure(1)
IRB120.teach
figure(2)
IRB120.plot([0 0 0 0 0 0], 'nojoints')
