function atSetpoint = reachedSetpoint(currentPos, desJ)
%Checks to see whether or not the arm has reached the designated setpoint
    atSetpoint = false;
        
    %convert back to angles   
    desJ = desJ * 90/1024;
    
    desPos = calcJointPos(desJ);
    
    close = abs(currentPos-desPos(:,4))< [2;2;2];
    if(close == [1;1;1])
        atSetpoint = true;
    end
end