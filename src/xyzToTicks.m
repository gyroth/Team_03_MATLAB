function [ viaJts ] = xyzToTicks( xyz )
% Converts xyz task space position to encoder ticks

                viaJtsAngles = zeros(size(xyz));
                
                numPoints = size(viaJtsAngles);
                
                for a = 1:numPoints(2)
                    viaJtsAngles(:,a) = iKin(xyz(:,a));
                end
                
                % Joint Angles in Ticks at each Setpoint
                viaJts = viaJtsAngles * 1024 / 90;
end

