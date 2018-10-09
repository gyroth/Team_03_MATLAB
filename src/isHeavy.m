function [ yOrN ] = isHeavy( Ftip )
%isHeavy: Takes in a Force and returns if it is greater than a determined
%limit
heavy = -11; %Newtons %MAKE SURE TO TEST ME
yOrN = Ftip(3) < heavy;

end

