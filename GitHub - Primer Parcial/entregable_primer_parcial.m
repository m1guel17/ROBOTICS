% ROBÓTICA - PRIMER PARCIAL
% Nombre: Miguel Esteban Flores Sierra
% Código: 2310949
% 1. Con sus palabras: 
%     a. ¿Qué es un robot? 
%         Es una máquina que puede realizar debidas tareas pre-programadas con muy poca o casi nada de interacción humana, entre ellos se encuentras las máquinas de precisión como los industriales en plantas de ensamblaje, o acabados. , Se espera que en unos años con el avance de la IA y ML, se puedan producir robots semi-conscientes los cuales puedan empezar a tomar decisiones lógicas basadas en su entrenamiento.
%     b. De ejemplos de actuadores, sensores y elementos terminales en robots
%         Actuadores:
%             Actuadores neumáticos como los cilindros de doble efecto.
%             Motores DC y AC.
%         Sensores:
%             Sensores de fin de carrera.
%             Giroscópio.
%             Encoders.
%             Efecto Hall.
%             Ultrasónico.
%             Elementos terminales:
%             Pinzas hidráulicas.
%             Electroimanes.
%             Pinza de soldadura de arco.
%
% 2. Un sistema Fnoa ha sido girado 90º alrededor del eje X y posteriormente trasladado por el vector pxyz(8, -4, 12). Calcule las coordenadas Rx, Ry y Rz, del vector R con coordenadas Rnoa(-3, 4, -11). Use las matrices de transformación.
clc; clear all; close all;
M0 = eye(4,4); % Marco base, que coincide con el de referencia
P=[-3 4 -11 1]'
lmin = -20;
lmax = 20;
plotvol([lmin lmax lmin lmax lmin lmax]) % Especificando dimensiones del gráfico.
% trplot(M0, 'color', 'k','thick',2, 'perspective','rgb')
trplot(M0, 'color', 'k', 'view',[145 20], 'length', 1)
plot3(P(1), P(2), P(3),'o', 'color', 'k')
pause(3)
Rx = trotx(90) % rotación respecto a x
M1 = Rx*M0;
P1 = Rx*P;
trplot(M1, 'color', 'b', 'length', 2)
plot3(P1(1), P1(2), P1(3),'o', 'color', 'b')
pause(3)
T = transl(8, -4, 12)
M2 = T*M1;
P2 = T*P1;
trplot(M2, 'color', 'r', 'length', 2)
plot3(P2(1), P2(2), P2(3),'o', 'color', 'r')
%% 
% 3. Partiendo de que se está trabajando en el plano XY (z=0) y utilizando la información disponible en la siguiente figura determine: 
%     a. La Matriz de transformación oMa 
%     b. La Matriz de transformación aMb 
%     c. Las coordenadas del punto oP=[1, 2, 0, 1]T con respecto a los marcos {a} y {b}, es decir aP y bP, respectivamente
clc; clear all; close all;
M0 = eye(4,4)
% M0 = [1     0     0     2;...
%       0     1     0     3;...
%       0     0     1     0;...
%       0     0     0     1]
P = [1 2 0 1]'
lmin = -1;
lmax = 5;
plotvol([lmin lmax lmin lmax lmin lmax]) % Especificando dimensiones del gráfico.
trplot(M0, 'color', 'k','frame','0', 'length', 1)
plot3(P(1), P(2), P(3),'o', 'color', 'k')

o_M_a = transl(3,0,0)
M1 = o_M_a % EN CASO EL MARCO DE REF INICIAL SEA DIFERENTE AL ORIGEN SE AGREGA M0
trplot(M1, 'color', 'b','frame','a', 'length', 1)

a_M_b = transl(-1,3,0)*trotz(90)
o_M_b = o_M_a*a_M_b;
M2 = o_M_b;
trplot(M2, 'color', 'g','frame','b', 'length', 1)

a_p = inv(o_M_a)*P
a_p = inv(o_M_b)*P
%%
% 4. Un robot está descrito por los siguientes parámetros de Denavit-Hartenberg. (Nota: Todas las unidades están en metros o radianes
%     a. Teniendo en cuenta el algoritmo de Denavit-Hartenberg. Dibuje el robot y en cada una de las articulaciones los respectivos marcos.
clc; clear all; close all
clear imports
import ETS3.*

q1 = [0 0.35];
q2 = [-180 180]*pi/180;
q3 = [-180 180]*pi/180;
q4 = [-180 180]*pi/180;

link_1=Prismatic('theta', 0, 'qlim', q1, 'a', 0.30, 'alpha', 0) 
link_2=Revolute('qlim', q2,  'd', 0.35,   'a', 0,   'alpha', 90*pi/180)
link_3=Revolute('qlim', q3,  'd', 0,     'a', 0.25, 'alpha', 0)
link_4=Revolute('qlim', q4,  'd', 0,     'a', 0.25, 'alpha', 0)

links=[link_1; link_2; link_3; link_4]

%crear robot
Robot=SerialLink(links,'name', 'Robot','manufacturer', 'Parcial', ...
                'plotopt',{'workspace',[-1.5 1.5 -1.5 1.5 -0.2 1.5]})
Robot.plot(zeros(length(links)))
hold on
trplot(eye(4))
Robot.teach
%     b. A partir de la Matriz de transformación 0M4: obtenga expresiones matemáticas que permitan, para cualquier valor de q1, q2, q3 o q4, calcular las componentes “ax”, “ay” y “az” del vector unitario “a” del marco ubicado en la punta del robot.
clc; clear all; close all
syms q1 q2 q3 q4 theta d a alpha

% ESLABÓN 1
theta(1)=0;  d(1)=q1;   a(1)=0.3;    alpha(1)=0;
% ESLABÓN 2
theta(2)=q2; d(2)=0.35; a(2)=0;      alpha(2)=90*pi/180;
% ESLABÓN 3
theta(3)=q3; d(3)=0;    a(3)=0.25;   alpha(3)=0;
% ESLABÓN 4
theta(4)=q4; d(4)=0;    a(4)=0.25;   alpha(4)=0;

disp('Parámetros Denavit-Hartenberg, simbólicos:')
D_H=[theta.',d.',a.',alpha.']

for i=1:1:4
    A(:,:,i)=[
        cos(theta(i)) -cos(alpha(i))*sin(theta(i)) sin(alpha(i))*sin(theta(i)) a(i)*cos(theta(i));
        sin(theta(i)) cos(alpha(i))*cos(theta(i)) -sin(alpha(i))*cos(theta(i)) a(i)*sin(theta(i));
        0 sin(alpha(i)) cos(alpha(i)) d(i)
        0 0 0 1
        ];
for f=1:1:4 
    for c=1:1:4
        if abs(double(coeffs(A(f,c,i)))) < 1e-16
           A(f,c,i)=0;
        end
    end
end
end

for i=1:1:4 
    disp(['Matriz de transformación: ',num2str(i-1),'A',num2str(i)])
    A(:,:,i)
end

disp('Matriz de transformación: 0T4')
T_0a4 = A(:,:,1)*A(:,:,2)*A(:,:,3)*A(:,:,4) 

syms Sq1 Cq1 Sq2 Cq2 Sq3 Cq3 Sq4 Cq4
for f=1:1:4 
    for c=1:1:4
        T_0a4(f,c)=subs(T_0a4(f,c),[sin(q1),cos(q1), sin(q2),cos(q2), sin(q3),cos(q3), sin(q4),cos(q4)],[Sq1, Cq1, Sq2, Cq2, Sq3, Cq3, Sq4, Cq4]);
    end
end

T_0a4
