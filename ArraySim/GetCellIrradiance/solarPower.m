function [solarOutput] = solarPower(sunVector,vertices,Irr)
    % SOLARPOWER Calculate the solar power [W/m^2] acting on a solar cell

    % Parameters:
    % sunVector:    The unit normal vector of the plane representation of the sun
    %               Note that the z component MUST be positive
    % vertices:     An n x 3 matrix of triangles describing an array cell

    N = size(vertices,1); 

    %Define solar output
    solarOutput = 0;

    % Loop through all triangles in the cell
    for i = 1:3:N
        
        %Define points
        A = vertices(i,:);
        B = vertices(i+1,:);
        C = vertices(i+2,:); 

        if (isequal(A, B) || isequal(A,C) || isequal(B,C))
            continue;
        end
        
        %Define vectors
        AB = B - A;
        AC = C - A;

        if (isequal(AB, AC))
            continue;
        end
        
        %Compute area of triangle
        area = 0.5*norm(cross(AB,AC));  
        
        %Generate unit normal vector of the triangle
        normal = cross(AB,AC)/norm(cross(AB,AC));
        
        %Ensure normal is facing the +ve Z direction 
        if normal(3) < 0 
            normal = (-1)*normal;
        end
        
        % Calculate solar energy for triangle. This works
        % since both the sunVector and normal of the triangle
        % have positive z component
        solar = dot(sunVector,normal)*area*Irr; 
        
        %Ensure sun is shining from above 
        if solar < 0 
            solar = 0;
        end
        solarOutput = solarOutput + solar;
        
    end

end