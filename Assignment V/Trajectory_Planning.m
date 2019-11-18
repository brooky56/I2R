% define all paramenters
global l1 l2 l3 q1 q2 q3
l1 = 30;
l2 = 30;
l3 = 25;
q1 = 10;
q2 = 20;
q3 = 30;
% define constraints
global f joint_vel cart_vel joint_a cart_a junction_p
f = 100;
joint_vel = 1; % rad/s
cart_vel = 1; % m/s
joint_a = 1; % rad/s^2
cart_a = 1; % m/s^2
junction_p = 0.2; %

T = FK([q1, q2, q3]);
T_invers = [inv(T(1:3,1:3)), zeros(3,1); 0 0 0 1];
% calculating Jacobians
T_d1 = Rz_d(q1)*Tz(l1)*Rx(q2)*Ty(l2)*Rx(q3)*Ty(l3)*T_invers;
T_d2 = Rz(q1)*Tz(l1)*Rx_d(q2)*Ty(l2)*Rx(q3)*Ty(l3)*T_invers;
T_d3 = Rz(q1)*Tz(l1)*Rx(q2)*Ty(l2)*Rx_d(q3)*Ty(l3)*T_invers;

J1 = [T_d1(1:3,4); T_d1(3,2); T_d1(1,3); T_d1(2,1)];
J2 = [T_d2(1:3,4); T_d2(3,2); T_d2(1,3); T_d2(2,1)];
J3 = [T_d3(1:3,4); T_d3(3,2); T_d3(1,3); T_d3(2,1)];

% we get Cartesian velosities and joint velocities for each joint
J=[J1, J2, J3]; 

global q0 qf v0 vf a0 af

q0 = 0;
v0 = 0;
a0 = 0;
qf = [T(1,4), T(2,4),T(3,4)];
vf = 0;
af = 0;

% To implement Point to point we need to know
% 1) Initial position, final position - using Forfard kinematics
% 2) Initial/final velocity, each time to calculate velocity by
% using Jaacobian
% 3) Initial/final acceleration

% We will use quantic polynomial trajectory q(t)=a0+a1t+a2t^2+a3t^3+a4t^4+a5t^5


function T = FK(q)
    global l1 l2 l3;
    T = Rz(q(1))*Tz(l1)*Rx(q(2))*Ty(l2)*Rx(q(3))*Ty(l3); 
end
function q = IK(T)
    global l1 l2 l3
    q1=atan2(T(2,4),T(1,4));
    a=sqrt(T(2,4)^2+T(1,4)^2);
    b=T(3,4)-l1;
    c3=(a^2+b^2-l2^2-l3^2)/...
        (2*l2*l3);
    s3=sqrt(1-c3^2);
    q3=atan2(s3,c3);
    q2=atan2(l3*cos(q3),(l2+l3*sin(q3)))+atan2(b,a);
    q = [q1, q2, q3];
end
function R_x = Rx(q)
R_x = [1 0 0 0; 0 cos(q) -sin(q) 0; 0 sin(q) cos(q) 0; 0 0 0 1];
end
function R_z = Rz(q)
R_z = [cos(q) -sin(q) 0 0; sin(q) cos(q) 0 0; 0 0 1 0; 0 0 0 1];
end
function T_y = Ty(d)
T_y = [1 0 0 0; 0 1 0 d; 0 0 1 0; 0 0 0 1];
end
function T_z = Tz(d)
T_z = [1 0 0 0; 0 1 0 0; 0 0 1 d; 0 0 0 1];
end

function R_x = Rx_d(q)
R_x = [0 0 0 0; 0 -sin(q) -cos(q) 0; 0 cos(q) -sin(q) 0; 0 0 0 0];
end
function R_z = Rz_d(q)
R_z = [-sin(q) -cos(q) 0 0; cos(q) -sin(q) 0 0; 0 0 0 0; 0 0 0 0];
end