%Definindo 2 sensores como A e B e sua leitura como Amed e Bmed
%e a referência deles como Aref e Bref, seus valores sao matrizes 3x1

%Aref = [xa, ya, za]
%Bref = [xb, yb, zb]

%Definindo o TRIAD frame de referência e o medido pelos sensores do satélite

tref1 = Aref;
tref2 = Bref;
tref3 = cros(Aref, Bref);

tmed1 = Amed;
tmed2 = Bmed;
tmed3 = cross(Amed, Bmed);

%Definindo o frame de referência a partir do TRIAD frame de referencia e
%seu DCM

nt1 = tref1;
nt2 = tref3/norm(tref3);
nt3 = cross(nt1, nt2);

NT = [nt1 nt2 nt3];

%Definindo o frame do corpo a partir do TRIAD frame medido pelos sensores e
%seu DCM

bt1 = tmed1;
bt2 = tmed3/norm(tmed3);
bt3 = cross(bt1, bt2);


BT = [bt1 bt2 bt3];

%Definindo o DCM do corpo para a referencia a partir dos oputros DCM's

BN = BT*transpose(NT);

%DCM em angulos de Euler para rotacao XYZ 
    
%RX = [1 0 0; 0 cos(theta1) sin(theta1); 0 -sin(theta1) cos(theta1)]
%RY = [cos(theta2) 0 -sin(theta2); 0 1 0; sin(theta2) 0 cos(theta2)]
%RZ = [cos(theta3) sin(theta3) 0; -sin(theta3) cos(theta3) 0; 0 0 1]
    
%BN = RZ*RY*RX
%BN = 
%[  cos(theta2)*cos(theta3), cos(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2), sin(theta1)*sin(theta3) - cos(theta1)*cos(theta3)*sin(theta2)]
%[ -cos(theta2)*sin(theta3), cos(theta1)*cos(theta3) - sin(theta1)*sin(theta2)*sin(theta3), cos(theta3)*sin(theta1) + cos(theta1)*sin(theta2)*sin(theta3)]
%[              sin(theta2),                                      -cos(theta2)*sin(theta1),                                       cos(theta1)*cos(theta2)]
    
%Definindo os angulos de Euler a partir do DCM do corpo para a referencia


theta2 = asin(BN(3,1));
theta1 = atan(-BN(3,2)/BN(3,3));
theta3 = atan(-BN(2,1)/BN(1,1));

%Matriz de angulos de Euler como uma 3x1 para realimentação

euler = [theta1 theta2 theta3]';
