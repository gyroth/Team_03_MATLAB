clear all
close all
clc

%%
% Declare symbolic varibles
syms A B C t q1(t) q2(t) q3(t) pi
symVariables = [A, B, C, q1(t), q2(t), q3(t), pi];

%%
%   D-H Param Table. 
% The following is the table containing the DH Parameters for the above
% diagram.

link1=  [0  pi/2 A -q1(t)];
link2 = [B  0  0 q2(t)];
link3 = [C  0  0 q3(t)-pi/2];

links = [link1;link2;link3];
titles = ["Link", "A", "alpha", "d", "theta"];
disp([titles; [[1; 2; 3], links]]);


%%
%   Intermediate homogeneous transforms
% The following is the function used to calculate the matrix from the given
% parameters. The values of the link_k arrays are as
% follows:
%
%   1: A 
%   2: Alpha
%   3: D
%   4: Theta
%

T01 = dhParam(link1(1), link1(2), link1(3), link1(4))
T12 = dhParam(link2(1), link2(2), link2(3), link2(4))
T23 = dhParam(link3(1), link3(2), link3(3), link3(4))

%%
% Forward Kinematics
%
% The complete transformation is found via multiplying intermediate
% transforms.
T02 = T01 * T12;
T03 = T01 * T12 * T23;
T03 = simplify(T03);
pretty(T03);

%%
% Numerical solution

% pi/12 = 15 deg
% pi/6 = 30 deg
% A, B, and C are measured in cm
vals = [60, 40, 20, pi/12, pi/6, -pi/12, pi];

% Substitute symbols for the values
numerical_T01 = subs(T01, symVariables, vals);
% Evaluate fractions as decimals
numerical_T01 = double(numerical_T01)

numerical_T12 = subs(T12, symVariables, vals);
numerical_T12 = double(numerical_T12)

numerical_T23 = subs(T23, symVariables, vals);
numerical_T23 = double(numerical_T23)

numerical_T03 = subs(T03, symVariables, vals);
numerical_T03 = double(numerical_T03)

%%
%  Approach vector
% Take the first three values of the complete transform. 
x3 = numerical_T03(1:3, 1)

%%
%  Velocity kinematics
% get time derivative of position vector

syms dq1 dq2 dq3

pos = T03(1:3, 4);
vel_kin = diff(pos, t);

vel_kin = subs(vel_kin, ...
        [diff(q1(t), t), diff(q2(t), t), diff(q3(t), t)], ...
        [dq1, dq2, dq3]);
    
pretty(simplify(vel_kin));
%%
% Jacobian
% obtains the jacobian of the robot

col1 = simplify(subs(vel_kin, [dq1, dq2, dq3], [1, 0, 0]));
col2 = simplify(subs(vel_kin, [dq1, dq2, dq3], [0, 1, 0]));
col3 = simplify(subs(vel_kin, [dq1, dq2, dq3], [0, 0, 1]));
pos_J = [col1, col2, col3];

ang_col_1 = [0; 0; 1];
ang_col_2 = T01(1:3, 3);
ang_col_3 = T02(1:3, 3);
ang_J = [ang_col_1, ang_col_2, ang_col_3];

J = [pos_J; ang_J]

%%
% Creates same Jacobian taking the partial derivative of the forward
% kinematics of q1, q2, and q3.

% turn the symbolic functions into symbolic variables to use diff
syms q1 q2 q3
symVarVals = [A, B, C, q1, q2, q3, pi];
alt_pos = subs(pos, symVariables, symVarVals);

% get partial derivatives
alt_col1 = simplify(diff(alt_pos, q1));
alt_col2 = simplify(diff(alt_pos, q2));
alt_col3 = simplify(diff(alt_pos, q3));

alt_pos_J = [alt_col1, alt_col2, alt_col3];

alt_J = [alt_pos_J; ang_J]
