function [canopyMesh, cellMeshes, canopyPoints, orderedCellPoints] = import_canopy_array_stl(canopyFileName, arrayCellPrefix, numberOfCells)
    % IMPORT_CANOPY_ARRAY_STL Load all array cell stls, the canopy stl and
    % center all geometries

    % Parameters:
    % canopyFileName:    The name of the canopy stl file. "none" for no canopy
    %                    simulation
    % arrayCellPrefixes: The prefix of all array cell stl files including their
    %                    relative path
    % numberOfCells:     Number of array cells

    % Output:
    % canopyMesh:        An opcode mesh of the canopy. "none" if no canopy
    %                    simulation
    % cellMeshes:        An array of opcode meshes for each array cell
    % canopyPoints:      An n x 3 matrix of points describing the canopy. "none" if
    %                    no canopy simulation
    % orderedCellPoints: An ordered cell array of n x 3 matrices that describe 
    %                    each cell of the array. Each matrix is ordered by 
    %                    groups of 3 points (triangle). Note that the index 
    %                    into this array corresponds with the array cell 
    %                    numbering i.e. NA(1).stl will be orderedCellPoints[1]

    % Load the canopy stl
    if (canopyFileName ~= "none")
        [canopyPoints, canopyFaces] = stlread2(canopyFileName);
    else
        canopyPoints = "none";
        canopyFaces = "none";
    end

    % Load all array stls
    [orderedCellPoints, origCellPoints, origCellFaces] = cellMake(numberOfCells, arrayCellPrefix); 

    % Center the array and canopy
    [orderedCellPoints, origCellPoints, canopyPoints] = centerArrayAndCanopy(orderedCellPoints, canopyPoints, origCellPoints);

    % Create canopy opcode mesh
    if (canopyFileName ~= "none")
        canopyMesh = opcodemesh(transpose(canopyPoints), transpose(canopyFaces));
    else
        canopyMesh = "none";
    end

    % Create opcode meshes for array cells
    cellMeshes = cell(1, numberOfCells);
    for i = 1:numberOfCells
        cellMeshes{i} = opcodemesh(transpose(origCellPoints{i}), transpose(origCellFaces{i}));
    end

end
