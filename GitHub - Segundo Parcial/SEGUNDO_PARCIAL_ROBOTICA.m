%% ROBÓTICA - SEGUNDO PARCIAL
% Nombre: Miguel Esteban Flores Sierra
% 
% Código: 2310949
%%
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
% UBICACIÓN DE MARCOS EN CADA ARTICULACIÓN
t = 0.25;
M0 = eye(4,4); T(:,:,1) = transl(0,0,0.290)*trotx(270);
M1 = M0*T(:,:,1); T(:,:,2) = trotz(-90)*transl(0.270,0,0);
M2 = M1*T(:,:,2); T(:,:,3) = trotx(-90)*transl(0.070,0,0);
M3 = M2*T(:,:,3); T(:,:,4) = trotx(90)*transl(0,0.302,0);
M4 = M3*T(:,:,4); T(:,:,5) = trotx(-90);
M5 = M4*T(:,:,5); T(:,:,6) = transl(0,0,0.072);
M6 = M5*T(:,:,6);

hold on
trplot(M0, 'rgb','frame','0', 'length', t, 'arrow')
trplot(M1, 'rgb','frame','1', 'length', t, 'arrow')
trplot(M2, 'rgb','frame','2', 'length', t, 'arrow')
trplot(M3, 'rgb','frame','3', 'length', t, 'arrow')
trplot(M4, 'rgb','frame','4', 'length', t, 'arrow')
trplot(M5, 'rgb','frame','5', 'length', t, 'arrow')
trplot(M6, 'rgb','frame','6', 'length', t, 'arrow')

% UBICACIÓN DE MARCOS EN EL ESPACIO
figure(3)
lmin = -0.75;
lmax = 0.75;
plotvol([lmin lmax lmin lmax -0.25 1])
trplot(M0, 'rgb','frame','0', 'length', t, 'view', [330 20])
trplot(M1, 'rgb','frame','1', 'length', t)
trplot(M2, 'rgb','frame','2', 'length', t)
trplot(M3, 'rgb','frame','3', 'length', t)
trplot(M4, 'rgb','frame','4', 'length', t)
trplot(M5, 'rgb','frame','5', 'length', t)
trplot(M6, 'rgb','frame','6', 'length', t, 'arrow')

% MATRIZ DE TRANSFORMACIÓN 0 -> 6
syms q1 q2 q3 q4 q5 q6 theta d a alpha

theta(1)=q1;        d(1)=0.290;     a(1)=0;        alpha(1)= -pi/2;
theta(2)=q2-pi/2;   d(2)=0;         a(2)=0.270;    alpha(2)=  0;
theta(3)=q3;        d(3)=0;         a(3)=0.070;    alpha(3)= -pi/2;
theta(4)=q4;        d(4)=0.302;     a(4)=0;        alpha(4)=  pi/2;
theta(5)=q5;        d(5)=0;         a(5)=0;        alpha(5)= -pi/2;
theta(6)=q6;        d(6)=0.072;     a(6)=0;        alpha(6)=  0;

for i=1:1:6
A(:,:,i)=[cos(theta(i)) -cos(alpha(i))*sin(theta(i)) sin(alpha(i))*sin(theta(i)) a(i)*cos(theta(i));...
          sin(theta(i)) cos(alpha(i))*cos(theta(i)) -sin(alpha(i))*cos(theta(i)) a(i)*sin(theta(i));...
          0 sin(alpha(i)) cos(alpha(i)) d(i);...
          0 0 0 1];
end
for i=1:1:6 
    disp(['Matriz de transformación: ',num2str(i-1),'A',num2str(i)])
    A(:,:,i)
end
Tx = T(:,:,1)*T(:,:,2)*T(:,:,3)*T(:,:,4)*T(:,:,5)*T(:,:,6);

Ax = simplify(A(:,:,1)*A(:,:,2)*A(:,:,3)*A(:,:,4)*A(:,:,5)*A(:,:,6));

syms S1 C1 S2 C2 S3 C3 S4 C4 S5 C5 S6 C6 S23 C23

for f=1:1:4 
    for c=1:1:4
        Ax_S(f,c)=subs(Ax(f,c),[sin(q1), cos(q1), sin(q2), cos(q2), sin(q3), cos(q3), sin(q4), cos(q4), sin(q5), cos(q5), sin(q6), cos(q6), sin(q2+q3), cos(q2+q3)],[S1, C1, S2, C2, S3, C3, S4, C4, S5, C5, S6, C6, S23, C23]);
    end
end
Ax_S
%% MÉTODO GEOMÉTRICO PARA LOS 3 PRIMEROS DoF
syms nx ox ax px ny oy ay py nz oz az pz
FDLOM = [nx, ox, ax, px;...
         ny, oy, ay, py;...
         nz, oz, az, pz;...
          0,  0,  0,  1];
pmx= FDLOM(1,4) - 0.072*FDLOM(1,3);
pmy= FDLOM(2,4) - 0.072*FDLOM(2,3);
pmz= FDLOM(3,4) - 0.072*FDLOM(3,3);
theta1 = atan2(pmy,pmx);

beta = atan(0.302/0.070);
dm2m4 = sqrt(0.070^2 + 0.302^2);
pm1m4 = sqrt(pmx^2 + pmy^2 + (pmz - 0.290)^2);
gamma = acos((pm1m4^2  - 0.270^2 - dm2m4^2)/(-2*0.270*dm2m4));
theta3 = pi - gamma - beta;

rho = atan((pmz - 0.290)/(sqrt(pmx^2 + pmy^2)));
mhu = asin((dm2m4*sin(gamma))/pm1m4);
theta2 = pi/2 - mhu - rho;
%% MÉTODO DE LA MATRIZ DE TRANSFORMACIÓN HOMOGÉNEA PARA LOS 3 ÚLTIMOS DoF
R123i = simplify(inv(A(1:3,1:3,1)*A(1:3,1:3,2)*A(1:3,1:3,3)))
R456 = simplify(A(1:3,1:3,4)*A(1:3,1:3,5)*A(1:3,1:3,6))
syms S1 C1 S2 C2 S3 C3 S4 C4 S5 C5 S6 C6 S23 C23
for f=1:1:3
    for c=1:1:3
        R123iS(f,c)=subs(R123i(f,c),[sin(q1), cos(q1), sin(q2), cos(q2), sin(q3), cos(q3), sin(q4), cos(q4), sin(q5), cos(q5), sin(q6), cos(q6), sin(q2+q3), cos(q2+q3)],[S1, C1, S2, C2, S3, C3, S4, C4, S5, C5, S6, C6, S23, C23]);
        R456S(f,c)=subs(R456(f,c),[sin(q1), cos(q1), sin(q2), cos(q2), sin(q3), cos(q3), sin(q4), cos(q4), sin(q5), cos(q5), sin(q6), cos(q6), sin(q2+q3), cos(q2+q3)],[S1, C1, S2, C2, S3, C3, S4, C4, S5, C5, S6, C6, S23, C23]);
        
    end
end

% MATRIZ DE POSICIÓN Y ORIENTACIÓN FINAL DESEADA 
% FDLOM = [nx,ox,ax,px;...
%          ny,oy,ay,py;...
%          nz,oz,az,pz;...
%           0, 0, 0, 1];

% DOMik = [nx,ox,ax;...
%          ny,oy,ay;...
%          nz,oz,az]
DOMik = FDLOM(1:3,1:3)
left_eq = R456S
right_eq = simplify(R123iS*DOMik)

left_eq = R456;
right_eq = simplify(R123i*DOMik);
equal_ = left_eq == right_eq;

e_1=simplify(equal_(3,3));
syms f_q5(ax,ay,az,q1,q2,q3) % DEFINIR FUNCIÓN PARA Q5 EN TÉRMINOS ax, ay, az, q1, q2 y q3
f_q5(ax,ay,az,q1,q2,q3) = solve(e_1,q5)
e_2=simplify(equal_(2,3));
syms f_q4(ax,ay,az,q1,q5) %DEFINIR FUNCIÓN PARA Q4 EN TÉRMINOS ax, ay, az, q1, q5
f_q4(ax,ay,az,q1,q5) = solve(e_2,q4)
e_3=simplify(equal_(3,1));
syms f_q6(nx,ny,nz,q1,q2,q3,q5) % DEFINIR FUNCIÓN PARA Q6 EN TÉRMINOS nx,ny,nz,q1,q2,q3 y q5
f_q6(nx,ny,nz,q1,q2,q3,q5) = solve(e_3,q6)

% IGUALAMOS LOS VALORES DE (3,3), (2,3), (3,1)
% R456S(3,3) = right_eq(3,3) || OBTENEMOS θ5
% R456S(2,3) = right_eq(2,3) || OBTENEMOS θ4
% R456S(3,1) = right_eq(3,1) || OBTENEMOS θ6
nx = DOMik(1,1); ox = DOMik(1,2); ax = DOMik(1,3);
ny = DOMik(2,1); oy = DOMik(2,2); ay = DOMik(2,3);
nz = DOMik(3,1); oz = DOMik(3,2); az = DOMik(3,3);

theta5 = acos(cos(theta2 + theta3)*(ax*cos(theta1) + ay*sin(theta1)) - az*sin(theta2 + theta3));
theta4 = asin((ay*cos(theta1) - ax*sin(theta1))/sin(theta5));
theta6 = pi - acos((nz*sin(theta2 + theta3) - cos(theta2 + theta3)*(nx*cos(theta1) + ny*sin(theta1)))/sin(theta5));
%% UBICACIÓN EN EL ESPACIÓ SEGUN LA MATRIZ PROPORCIONADA PARA EL PUNTO c.
FDLOM = [0.5194  0.3355  0.7859 0.3116;...
         0.0629 -0.9322  0.3564 0.3898;...
         0.8522 -0.1357 -0.5053 0.5039;...
         0       0       0      1];

% MÉTODO GEOMÉTRICO
pmx= FDLOM(1,4) -0.072*FDLOM(1,3);
pmy= FDLOM(2,4) -0.072*FDLOM(2,3);
pmz= FDLOM(3,4) -0.072*FDLOM(3,3);

theta1 = atan2(pmy,pmx);

beta = atan(0.302/0.070);
dm2m4 = sqrt(0.070^2 + 0.302^2);
pm1m4 = sqrt(pmx^2 + pmy^2 + (pmz - 0.290)^2);
gamma = acos((pm1m4^2  - 0.270^2 - dm2m4^2)/(-2*0.270*dm2m4));
theta3 = pi - gamma - beta;

rho = atan((pmz - 0.290)/(sqrt(pmx^2 + pmy^2)));
mhu = asin((dm2m4*sin(gamma))/pm1m4);
theta2 = pi/2 - mhu - rho;

% DESACOPLO CINEMÁTICO
DOMik = FDLOM(1:3,1:3);
nx = DOMik(1,1); ox = DOMik(1,2); ax = DOMik(1,3);
ny = DOMik(2,1); oy = DOMik(2,2); ay = DOMik(2,3);
nz = DOMik(3,1); oz = DOMik(3,2); az = DOMik(3,3);
theta5 = acos(cos(theta2 + theta3)*(ax*cos(theta1) + ay*sin(theta1)) - az*sin(theta2 + theta3));
theta4 = asin((ay*cos(theta1) - ax*sin(theta1))/sin(theta5));
theta6 = pi - acos((nz*sin(theta2 + theta3) - cos(theta2 + theta3)*(nx*cos(theta1) + ny*sin(theta1)))/sin(theta5));

disp(['θ1:  ',num2str(theta1),' rads || ',num2str(theta1*180/pi),' °'])
disp(['θ2:  ',num2str(theta2),' rads || ',num2str(theta2*180/pi),' °'])
disp(['θ3: ',num2str(theta3),' rads || ',num2str(theta3*180/pi),' °'])
disp(['θ4: ',num2str(theta4),' rads || ',num2str(theta4*180/pi),' °'])
disp(['θ5:  ',num2str(theta5),' rads || ',num2str(theta5*180/pi),' °'])
disp(['θ6:   ',num2str(theta6),' rads || ',num2str(theta6*180/pi),' °'])

figure(4)
angles = [theta1 theta2 theta3 theta4 theta5 theta6]

IRB120.teach(angles,'view', [39.32 14.73], 'rpy')

for f=1:1:4 
    for c=1:1:4
        Ax_F(f,c)=subs(Ax(f,c),[q1,q2,q3,q4,q5,q6],angles);
    end
end
Ax_F = round(double(Ax_F),4)
isequal(FDLOM, double(Ax_F))