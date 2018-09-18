function [ a_i ] = quintTraj( t_0, t_f, v_0, v_f, q_0, q_f, a_0, a_f)
%cubicTraj: Calculates the coefficients for a quintic trajectory polynomial
%   Uses the desired initial time, end time, initial velocity, end velocity, 
%     initial joint angle, end joint, initial acceleration, and end acceleration angle to calculate the 6x6 matrix of coefficients

a_i = inv([1,      t_0,        power(t_0,2),       power(t_0,3),    power(t_0,4),   power(t_0,5);
    
           0,      1,          2*t_0,              3*power(t_0,2),  4*power(t_0,3), 5*power(t_0,4);
           
           0,      0,          2,                   6*t_0,          12*power(t_0,2), 20*power(t_0,3);
                                
           1,      t_f,        power(t_f,2),       power(t_f,3),    power(t_f,4),   power(t_f,5);
    
           0,      1,          2*t_f,              3*power(t_f,2),  4*power(t_f,3), 5*power(t_f,4);
           
           0,      0,          2,                   6*t_f,          12*power(t_f,2), 20*power(t_f,3);])* [q_0; v_0; a_0; q_f; v_f; a_f] ;

end

 
