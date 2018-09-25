function atSetpoint = reachedSetpoint(currentPos, desJ)
    atSetpoint = false;
        
    %convert back to angles   
    desJ = desJ * 90/1024;
    
    desPos = calcJointPos(desJ);
    
    close = abs(currentPos-desPos(:,4))< [5;5;5];
    if(close == [1;1;1])
        atSetpoint = true;
    end
end