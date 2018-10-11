function [ yOrN ] = isHeavy( Ftip )
%isHeavy: Takes in a Force and returns if it is greater than a determined
%limit
heavy = -9.2; %Newtons
yOrN = Ftip(3) < heavy;

end

