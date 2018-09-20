function J = jacob0( jAngles )
%jacob0 Calculates the Jacobian of the arm
%   Detailed explanation goes here
sym pi
syms A B C t q1(t) q2(t) q3(t)
symVariables = [A, B, C, q1(t), q2(t), q3(t), pi];

link1= [0 pi/2 A q1(t)+pi/2];
link2 = [B 0 0 q2(t)];
link3 = [C 0 0 q3(t)];
links = [link1;link2;link3];
titles = ["Link", "A", "alpha", "d", "theta"];
disp([titles; [[1; 2; 3], links]]);

T01 = dhParam(link1(1), link1(2), link1(3), link1(4))
T12 = dhParam(link2(1), link2(2), link2(3), link2(4))
T23 = dhParam(link3(1), link3(2), link3(3), link3(4))
T02 = T01 * T12;
T03 = T01 * T12 * T23;

syms dq1 dq2 dq3
pos = T03(1:3, 4);
vel_kin = diff(pos, t);
vel_kin = subs(vel_kin, ...
 [diff(q1(t), t), diff(q2(t), t), diff(q3(t), t)], ...
 [dq1, dq2, dq3])
%vel_kin = [C*dq1*sin(q2(t))*sin(q3(t))*sin(pi/2 + q1(t)) - B*dq2*cos(pi/2 + q1(t))*sin(q2(t)) - B*dq1*cos(q2(t))*sin(pi/2 + q1(t)) - C*dq1*cos(q2(t))*cos(q3(t))*sin(pi/2 + q1(t)) - C*dq2*cos(q2(t))*cos(pi/2 + q1(t))*sin(q3(t)) - C*dq2*cos(q3(t))*cos(pi/2 + q1(t))*sin(q2(t)) - C*dq3*cos(q2(t))*cos(pi/2 + q1(t))*sin(q3(t)) - C*dq3*cos(q3(t))*cos(pi/2 + q1(t))*sin(q2(t));
% B*dq1*cos(q2(t))*cos(pi/2 + q1(t)) - B*dq2*sin(q2(t))*sin(pi/2 + q1(t)) - C*dq1*cos(pi/2 + q1(t))*sin(q2(t))*sin(q3(t)) - C*dq2*cos(q2(t))*sin(q3(t))*sin(pi/2 + q1(t)) - C*dq2*cos(q3(t))*sin(q2(t))*sin(pi/2 + q1(t)) - C*dq3*cos(q2(t))*sin(q3(t))*sin(pi/2 + q1(t)) - C*dq3*cos(q3(t))*sin(q2(t))*sin(pi/2 + q1(t)) + C*dq1*cos(q2(t))*cos(q3(t))*cos(pi/2 + q1(t));
%                                                                                                                                                                                                                                B*dq2*cos(q2(t)) + C*dq2*cos(q2(t))*cos(q3(t)) + C*dq3*cos(q2(t))*cos(q3(t)) - C*dq2*sin(q2(t))*sin(q3(t)) - C*dq3*sin(q2(t))*sin(q3(t))];
pretty(simplify(vel_kin));

col1 = simplify(subs(vel_kin, [dq1, dq2, dq3], [1, 0, 0]))
% col1 = [-cos(q1(t))*(C*cos(q2(t) + q3(t)) + B*cos(q2(t)));
%  -sin(q1(t))*(C*cos(q2(t) + q3(t)) + B*cos(q2(t)));
%                                                  0];
col2 = simplify(subs(vel_kin, [dq1, dq2, dq3], [0, 1, 0]))
% col2 = [sin(q1(t))*(C*sin(q2(t) + q3(t)) + B*sin(q2(t)));
%  -cos(q1(t))*(C*sin(q2(t) + q3(t)) + B*sin(q2(t)));
%                C*cos(q2(t) + q3(t)) + B*cos(q2(t))];
col3 = simplify(subs(vel_kin, [dq1, dq2, dq3], [0, 0, 1]))
% col3 =  [C*sin(q1(t))*sin(q2(t) + q3(t));
%    -C*cos(q1(t))*sin(q2(t) + q3(t));
%              C*cos(q2(t) + q3(t))];
pos_J = [col1, col2, col3]
% pos_J = [ -cos(q1(t))*(C*cos(q2(t) + q3(t)) + B*cos(q2(t))),  sin(q1(t))*(C*sin(q2(t) + q3(t)) + B*sin(q2(t))),  C*sin(q1(t))*sin(q2(t) + q3(t));
%  -sin(q1(t))*(C*cos(q2(t) + q3(t)) + B*cos(q2(t))), -cos(q1(t))*(C*sin(q2(t) + q3(t)) + B*sin(q2(t))), -C*cos(q1(t))*sin(q2(t) + q3(t));
%                                                  0,               C*cos(q2(t) + q3(t)) + B*cos(q2(t)),             C*cos(q2(t) + q3(t))];
ang_J = [[0;0;1],T01(1:3,3),T02(1:3,3)]
% ang_J = [ 0,  sin(pi/2 + q1(t)),  sin(pi/2 + q1(t));
%  0, -cos(pi/2 + q1(t)), -cos(pi/2 + q1(t));
%  1,                  0,                  0];

J = [pos_J; ang_J]
% J = [ -cos(q1(t))*(C*cos(q2(t) + q3(t)) + B*cos(q2(t))),  sin(q1(t))*(C*sin(q2(t) + q3(t)) + B*sin(q2(t))),  C*sin(q1(t))*sin(q2(t) + q3(t));
%  -sin(q1(t))*(C*cos(q2(t) + q3(t)) + B*cos(q2(t))), -cos(q1(t))*(C*sin(q2(t) + q3(t)) + B*sin(q2(t))), -C*cos(q1(t))*sin(q2(t) + q3(t));
%                                                  0,               C*cos(q2(t) + q3(t)) + B*cos(q2(t)),             C*cos(q2(t) + q3(t));
%                                                  0,                                 sin(pi/2 + q1(t)),                sin(pi/2 + q1(t));
%                                                  0,                                -cos(pi/2 + q1(t)),               -cos(pi/2 + q1(t));
%                                                  1,                                                 0,                                0];
end