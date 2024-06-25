function [projectedPoints] = project_onto_sun_plane(sunPlane,pProject)
    % PROJECT_ONTO_SUN_PLANE Project a set of 3D points onto a plane

    % Parameters: 
    % sunPlane:     A 2 member struct describing the plane. See output of 
    %               create_sun_plane() 
    % pProject:     An n x 3 matrix with n points to project onto the plane

    % Return:
    % projectedPoints: An n x 3 matrix with the projected points

    numPoints = size(pProject,1); 
    projectedPoints = zeros(numPoints, 3);
    normalVector = sunPlane.normal;
    pointOnPlane = sunPlane.point;

    for i = 1:numPoints
        pointToPlane = pProject(i,:) - pointOnPlane;
        distance = dot(pointToPlane, normalVector);
        projectedPoints(i,:) = pProject(i,:) - distance * normalVector;

    end

end