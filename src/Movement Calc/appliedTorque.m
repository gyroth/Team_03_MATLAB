function [ x ] = appliedTorque( adc )
%appliedTorque Finds the torque given the adc reading

y_0 = [1788.2,1848.5,1896.3]; %offset values
 k = 178.5; %scaling factor
 y = adc;  %input ADC reading from 0 to 4095
 
x = (y-y_0)/k; %calculated torque Nm

x = x*1000; %torque Nmm

end

