function [ a_i ] = cubicTraj( t_0, t_f, v_0, v_f, q_0, q_f )
%cubicTraj: Summary of this function goes here
%   Detailed explanation goes here

a_i = inv([1,      t_0,        power(t_0,2),       power(t_0,3);
    
           0,      1,          2*t_0,              3*power(t_0,2);
                                
           1,      t_f,        power(t_f,2),       power(t_f,3);
                                
           0,      1,          2*t_f,              3*power(t_f,2);])* [q_0; v_0;q_f;v_f] ;

end

 