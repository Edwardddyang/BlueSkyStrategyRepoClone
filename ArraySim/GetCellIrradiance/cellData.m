function [cellIrr] = cellData(sunPlane,shadedArrayPoints,allArrayPoints,Irr)
    % CELLDATA Calculate effective irradiance [W/m^2] for a set of cells

    % Parameters:
    % sunPlane:             A 2 member struct describing the plane of the sun.
    %                       See create_sun_plane()
    % shadedArrayPoints:    A cell array of p x 3 matrices containing the unshaded 
    %                       triangles of each cell
    % allArrayPoints:       A cell array of n x 3 matrices containing all the triangles 
    %                       of each cell
    % Irr:                  The irradiance of the sun in W/m^2

    % Outputs:
    % cellIrr: An array of effective direct irradiances on each cell in W/m^2

    numCells = size(shadedArrayPoints, 2);
    sunNormal = sunPlane.normal;
    cellIrr = zeros(1,numCells);

    % Iterate over each cell and determine the effective power
    for i = 1:numCells    
                
        % Obtain power output of cell
        cellOut = solarPower(sunNormal,shadedArrayPoints{i},Irr); 
        
        % Obtain total area of cell 
        cellArea = return_areas(allArrayPoints{i});
                
        % Obtain irradiance for cell i 
        if cellArea > 0               
            cellIrr(i) = cellOut/cellArea;                
        else
            cellIrr(i) = 0;
        end
        
    end

end