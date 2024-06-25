function [] = main(nCells, direction, bearing, outputName, canopyPath, arrayCellsPath, positionsPath)
    % MAIN Create a csv containing the [W/m^2] effective irradiance on all
    % array cells over the duration of a day

    %   main(257, "-x", 180.0, "wscIrr.csv", "./ArraySTLs/canopy.stl", 
    %       "./ArraySTLs/NA", "./sun_position.csv")

    % Parameters:
    % nCells:        Number of cells of the array
    % direction:     The direction of the nose of the array. Can be one of
    %                "+x", "-x", "+y", "-y". Look at the CAD to get this.
    % bearing:       The angle of the nose of the car clockwise from true 
    %                north.
    % outputName:    The path to the output csv
    % canopyPath:    The path to the canopy stl file. "none" for no canopy
    %                simulation
    % arrayCellPath: The path to the array stl files. All stl files must be
    %                monotonically numbered and have the same prefix. 
    %                e.g. ./STLs/NA implies that all cells are named 
    %                NA(<number>).stl within the STLs folder
    % positionsPath: The path to the 4 column csv describing the path
    %                of the sun. See dayAzElIrr.py for details

    % Output: 
    % An n x d csv where n is the number of timestamps (rows) 
    % in the positionsPath csv and d is the number of array cells.
    % This csv is stored in the outputName path

    % Note: The nose of the car must point in one of the 4 principal axes +x, -y, +y, -x.
    % The top of the car MUST also be facing the +z direction. Verify this in the CAD 
    % diagram

    wscAngles = readmatrix(positionsPath);
    N = size(wscAngles,1); 
    wscIrrCell = cell(N, 1);

    % Import canopy and array cells
    numberOfCells = nCells;
    [canopyMesh, arrayCellMeshes, canopyPoints, arrayCellPoints] = import_canopy_array_stl(canopyPath, arrayCellsPath, numberOfCells);

    % Plot the array and canopy - testing purposes
    highlightCells = [103, 104];
    plotArrayCanopy(canopyPoints, arrayCellPoints, highlightCells, 0);

    % Find the largest coordinate value along any dimension
    if (canopyPath ~= "none")
        largestCoordinate = max(cat(1, canopyPoints, arrayCellPoints{:}), [], "all");
    else
        largestCoordinate = max(vertcat(arrayCellPoints{:}), [], "all");
    end

    % Loop over all sun positions described in wscAngles and find irradiance on each cell
    for i = 1:N
    
        % Extract sun data 
        Az = wscAngles(i,1);
        El = wscAngles(i,2);
        Irr = wscAngles(i,3);

        % Create sun plane
        sunPlane = create_sun_plane(Az, El, direction, bearing, largestCoordinate);

        % Loop over every cell and remove shaded triangles
        sCells = remShadCellStruc(sunPlane, canopyMesh, arrayCellMeshes, arrayCellPoints, i);

        % Find power output of each cell in W/m^2
        wscIrrCell{i} = cellData(sunPlane,sCells,arrayCellPoints,Irr);
        
        disp(append("Done Row ", int2str(i)));

    end

    % Export output csv
    writecell(wscIrrCell,outputName);

end
