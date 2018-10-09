function outputArg = interpolate(pos1, pos2)
    %Interpolate 10 equally spaced points along the line between 2 points
    %Then places every point into a matrix
    vect=[pos2(1)-pos1(1); pos2(2)-pos1(2); pos2(3)-pos1(3)];
    
    i=2;
    outputArg= zeros(3,12, 'single');
    outputArg(:,1)=pos1;
    outputArg(:,12)=pos2;
    while i<12
       dist = sqrt(power(vect(1),2)+power(vect(2),2)+power(vect(3),2)) * ((i-1)/11);
       mag= sqrt(power(vect(1),2)+power(vect(2),2)+power(vect(3),2));
       outputArg(:,i)= pos1+((dist/abs(mag))*vect);
       i=i+1;
    end
end