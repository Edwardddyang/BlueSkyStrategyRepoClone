function [sourceVertices, dirs] = calculate_rays(cellVertices, sunPlane)
    % CALCULATE_RAYS Calculate the direction vectors from a set of vertices
    % to their respective orthogonal projections onto a given plane

    % Parameters:
    % cellVertices: Set of vertices whose rays are calculated. They are 
    %               formatted in an n x 3 matrix where each triplet of rows
    %               describe one triangle i.e. triangle 1 = row 1->row 3,
    %               triangle 2 = row 4->row 6
    % sunPlane:     The plane to project on as a 2 member struct. 
    %               See output of create_sun_plane

    % Return:
    % sourceVertices:   n x 3 matrix of source vertices (centroid of all triangles)
    % dirs:             n x 3 matrix of directional vectors

    % Loop over each triangle and find the ray from each triangle's centroid
    % to its orthogonal projection
    numPoints = size(cellVertices, 1);
    sourceVertices = zeros(numPoints/3,3);
    idx = 1;

    for i = 1:3:numPoints
        pt1 = cellVertices(i,:);
        pt2 = cellVertices(i+1,:);
        pt3 = cellVertices(i+2,:);

        centroid = (pt1 + pt2 + pt3) / 3;
        sourceVertices(idx,:) = centroid;
        idx = idx + 1;
    end

    projection = project_onto_sun_plane(sunPlane, sourceVertices);
    dirs = projection - sourceVertices;

end