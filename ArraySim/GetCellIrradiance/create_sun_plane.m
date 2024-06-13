function [sunPlane] = create_sun_plane(azimuth,elevation,dir,bearing,minPoint)
    % CREATE_SUN_PLANE Create an infinite plane representation of the sun by
    % converting az, el to cartesian coordinates and finding a point on the plane

    % Parameters:
    % azimuth:      Azimuth angle in degrees (clockwise from true north [0, 360])
    % elevation:    Elevation angle in degrees [0,90]
    % dir:          String indicating the axis that the nose of the car points in
    % bearing:      Bearing of the nose of the car from true north
    % minPoint:     The minimum distance that any point of the plane must be from the origin

    % Return:
    % sunPlane:     A 2 member struct. One is the a 3x1 vector indicating the unit normal (normal)
    %               and the other is a point on the plane (point)

    % NOTE: By convention, we use a right handed coordinate system where the +z axis points up towards
    % the sky. Therefore, the returned normal vector is really just the normalized position of the sun in
    % the sky and will always have +ve z component

    % Correct the azimuth angle such that it points clockwise from the nose of the car
    if bearing <= azimuth
        corrected_azimuth = azimuth - bearing;
    else
        corrected_azimuth = 360 - (bearing - azimuth);
    end

    x = cosd(elevation)*cosd(corrected_azimuth);
    y = cosd(elevation)*sind(corrected_azimuth); 
    z = sind(elevation);

    magnitude = 0;
    if minPoint < 1
        magnitude = 1;
    else
        magnitude = minPoint + 1;
    end

    % Correct based on the direction of the nose
    if dir == "+x"
        y = -1 * y;
    elseif dir == "-x"
        x = -1 * x;
    elseif dir == "+y"
        temp_x = x;
        x = y;
        y = temp_x;
    elseif dir == "-y"
        temp_x = x;
        x = -1 * y;
        y = -1 * temp_x;
    else
        disp("Please indicate the direction of the nose: +x, -x, +y, -y");
    end

    sunPlane = struct();
    sunPlane.normal = [x,y,z];
    sunPlane.point = [x*magnitude, y*magnitude, z*magnitude];

end 
