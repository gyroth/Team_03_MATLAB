function Ftip = statics3001( q,t )
%Takes robot configuration and joint torques, and returns the end effector
%force

jaco = jacob0(q);
top3 = jaco(1:3,1:3);
inverse = inv(top3');
Ftip = inverse*t;

end

