function atSetpoint = reachedSetpoint(currentPos, desJ)
    atSetpoint = false;
    disp('desired pos')
    
    %convert back to angles   
    desJ = desJ * 90/1024
    
    desPos = calcJointPos(desJ)
    
    close = abs(currentPos-desPos(:,4))< [20;20;20];
    if(close == [1;1;1])
        atSetpoint = true;
    end
    disp('atSetpoint')
    disp(close)

end