function [oCells, ogCells, v_C] = centerArrayAndCanopy(orderedCellV, canopy, origCells)
    % CENTERARRAYANDCANOPY Center all array cell points and the canopy point 
    % cloud around the origin

    % Parameters:
    % orderedCellV: An ordered cell array of n x 3 matrices that store the 
    %               triangle makeup of each cell
    % canopy:       An n x 3 matrix that describes the canopy ordered into
    %               triangles. "none" for no canopy simulation
    % origCells:    An unordered cell array of n x 3 matrices that store the 
    %               triangle makeup of each cell

    % Output:
    % oCells:  Same as orderedCellV with shifting
    % ogCells: Same as origCells with shifting
    % v_C:     Same as canopy with shifting. "none" if no canopy simulation

    % Collect all points from the array and canopy
    if (isempty(canopy) == 0)
        disp("Detected canopy");
        allArrayAndCanopyPoints = cat(1, canopy, orderedCellV{:});
    else
        allArrayAndCanopyPoints = vertcat(orderedCellV{:});
    end

    % Calculate the centroid by taking the mean along each axis
    centroid = mean(allArrayAndCanopyPoints);

    % Center the canopy
    if (isempty(canopy) == 0)
        disp("Detected canopy");
        canopy = bsxfun(@minus, canopy, centroid);
    end

    % Center the array
    numCells = size(orderedCellV, 2);
    for i=1:numCells
        orderedCellV{i} = bsxfun(@minus, orderedCellV{i}, centroid);
        origCells{i} = bsxfun(@minus, origCells{i}, centroid);
    end

    v_C = canopy;
    oCells = orderedCellV;
    ogCells = origCells;

end

