
function xdot = aer404project2_2(X, U)

%commanded from joystick
deltaT1 =U(1);
deltaT2 =U(2);
deltaA = U(3);
deltaE = U(4);
deltaR = U(5);

U=X(7);
V = X(8);
W = X(9);

P = X(10);
Q = X(11);
R = X(12);
phi = X(4);
theta = X(5);
psi = X(6);

PN =X(3);
PE = X(2);
h =X(1);

%mass
mass = 120000; 

Matrix1 = [40.07 0 -2.0923; 0 64 0; -2.0923 0 99.92];

%inertia matrix
J = mass*Matrix1;

%mean aerodynamic chord
chord = 6.6;

%aircraft center of mass
cmass_og = [(0.23*chord) 0 (0.1*chord)];
cmass = cmass_og.';

%aircraft aerodynamic center
acenter_og = [(0.12*chord) 0 0];
acenter = acenter_og.';

%engine 1 thrust application point
apt1_og = [0 -7.94 -1.9];
apt1 = apt1_og.';

%engine 12 thrust application point
apt2_og = [0 7.94 -1.9];
apt2 = apt2_og.';

%wing platform area
S = 260;

%wing platform area of tail
St = 64;

%generalized length of wings
l = 6.6;

%distanve b/w aircraft cm and ac of tail
lt = 24.8;

%angle of attack when lift is 0 (degrees)
alpha0 = deg2rad(-11.5);

%air density
p = 1.225;

%acceleration due to gravity
g = 9.81;

%airspeed
Vt = sqrt(U^2 + V^2 + W^2);

%dynamic pressure
q = 0.5*p*Vt^2;

%angle of attack
alpha = atan(W/U);

%sideslipp angle
beta = asin(V/Vt);

%downwash angle of wings
E = 0.25*(alpha - alpha0);

%angle of attach of the tail
alphaT = alpha - E + deltaE + (1.3*Q*(lt/Vt));

%% AERODYNAMIC COEFFICIENTS %%

%lift coefficient of wing body
CL_wb = 5.5*(alpha - alpha0);
%lift coefficient of tail
CL_t = 3.1*(St/S)*alphaT;
%lift coefficient
CL = CL_wb + CL_t;

%drag coefficient
CD = 0.13 + (0.07*(((5.5*alpha) + 0.654)^2));

%crosswind coefficient
Cc = -1.6*beta + (0.24*deltaR);

%rolling moment coefficient
Cl = (-1.4*beta) - (11*(l/Vt)*P) + (5*(l/Vt)*R) - (0.6*deltaA) + (0.22*deltaR);

%pitching moment coefficient
Cm = -0.59 - 3.1*((St*lt)/(S*l))*(alpha-E) - 4.03*((St*lt^2)/(S*l^2))*(l/Vt)*Q - 3.1*((St*lt)/(S*l))*deltaE;

%yawing moment coefficient
Cn = (1 - 3.8179*alpha)*beta + 1.7*(l/Vt)*P - 11.5*(l/Vt)*R - 0.63*deltaR;

%% AERODYNAMIC FORCES %%
%subscript 'a' indicates aerodynamic source of the forces
%subscript 't' indicates engine thrust source

%drag
D = q*S*CD;

%crosswind
C = q*S*Cc;

%lift
L = q*S*CL;

%resolving D, C, L into FRD coordinates

%real drag
Xa = -D*(cos(alpha))*cos(beta) + C*(cos(alpha))*sin(beta) + L*sin(alpha);

%real crosswing
Ya = -D*sin(beta) - C*cos(beta);

%real lift
Za = -D*sin(alpha)*cos(beta) + C*sin(alpha)*sin(beta) - L*cos(alpha);

%% AERODYNAMIC MOMENTS %%

%rolling, pitching, yawning denoted as l, m, n

%rolling moment
l_a = q*C*S*Cl;

%pitching moment
m_a = q*C*S*Cm;

%yawing moment
n_a = q*C*S*Cn;

            %resolve into FRD coordinates, idk if its already done
moment_matrix = [l_a; m_a; n_a];
forces_matrix = [Xa; Ya; Za];
constant_matrix = [(0.11*chord); 0; (0.1*chord)];

%actual aerodynamic moments 

%unsure how to write this expression
%top of page 5 in supplementary
%gives [la; ma; na]

%double code
% l_a_prime=q*C*S*Cl;
% m_a_prime=q*C*S*Cm;
% n_a_prime=q*C*S*Cn;
% 
% aero_moments = [l_a_prime m_a_prime n_a_prime] + cross([Xa Ya Za], [0.11*C 0 0.1*C]);

Actual_moments = moment_matrix + (cross(forces_matrix,constant_matrix));

%%

%Engine Forces
XT1 = deltaT1*mass*g;
XT2 = deltaT2*mass*g;
XT = XT1 + XT2;

%Engine Moments
%for each engine, moment is generated
engine_moment_matrix_1 = [XT1; 0; 0];
engine_moment_matrix_2 = [XT2; 0; 0];

engine_moment_1 = cross(engine_moment_matrix_1, (cmass - apt1));
engine_moment_2 = cross(engine_moment_matrix_2, (cmass - apt2));

%Force Equation
Udot = R*V - Q*W - g*sin(theta) + ((XT + Xa/mass));
Yt=0;
Zt=0;
Vdot = - R*U + P*W + g*sin(phi)*cos(theta) + ((Yt + Ya)/mass);
Wdot = Q*U - P*V + g*cos(phi)*cos(theta) + ((Zt + Za)/mass);

%initializing matrix elements
Jx=J(1,1);
Jxy=J(1,2) && J(2,1);
Jxz=J(1,3) && J(3,1);
Jy=J(2,2);
Jyz=J(2,3) && J(3,2);
Jz=J(3,3);
%Moment Equations
CapGam = Jx*Jz-(Jxz)^2;

n_t = engine_moment_matrix_1(3, 1) + engine_moment_matrix_2(3, 1);
m_t = engine_moment_matrix_1(2, 1) + engine_moment_matrix_2(2, 1);

CapGam_Pdot = Jxz*(Jx-Jy+Jz)*(P*Q)-(Jz*(Jz-Jy)+(Jxz)^2)*(Q*R)+Jz*(l_a+lt)+Jxz*(n_a+n_t);
Jy_Qdot = (Jz-Jx)*(P*R)-Jxz*(P^2-R^2) + m_a + m_t;
CapGam_Rdot = ((Jx-Jy)*Jx+(Jxz)^2)*(P*Q)-Jxz*(Jx-Jy+Jz)*(Q*R)+Jxz*(l_a+lt)+Jx*(n_a+n_t);

%Kinematic Equations

phidot = P + tan(theta)*(Q*sin(phi) + R*cos(phi));

thetadot = Q*cos(phi) - R*sin(phi);

psidot = (Q*sin(phi) + R*cos(phi))/(cos(theta));


%Navigation Equations
PNdot = U*cos(theta)*cos(psi) + V*(-cos(phi)*sin(psi) + sin(phi)*sin(theta)*cos(psi)) + W*(sin(phi)*sin(psi) + cos(phi)*sin(theta)*cos(psi));

PEdot = U*cos(theta)*sin(psi) + V*(cos(phi)*cos(psi) + sin(phi)*sin(theta)*sin(psi)) + W*(-sin(phi)*cos(psi) + cos(phi)*sin(theta)*sin(psi));

hdot = U*sin(theta) - V*sin(phi)*cos(theta) - W*cos(phi)*cos(theta);

%% GEODETIC COORDINATES EQUATIONS %%

%earth's equitorial radius a
a = 6378137;

%the first eccentricity 
e = 0.081819190842622;

%meridian radius
M = (a*(1-e^2))/(sqrt((1-e^2*(sin(phi)^2)))^3);

%prime vertical radius 
N = a/(sqrt(1-e^2*(sin(phi))^2));

%latitude
latdot = PNdot/(M + h);

%longitude

longdot = PEdot/((N + h)*cos(PN));

xdot = [longdot; latdot; hdot; phidot; thetadot; psidot; Udot; Vdot; Wdot; CapGam_Pdot; Jy_Qdot; CapGam_Rdot];
end
