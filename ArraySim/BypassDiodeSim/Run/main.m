function [] = main(n_cores, irr_csv, num_folders, array_layout, base_folder, offset, starting_row, ending_row)
% Run a distributed simulink simulation
% @param n_cores: Number of available cores to run on
% @param irr_csv: Name of the n x num_cells csv generated from step 2
% @param num_folders: Number of instances of matlab to run
% @param offset: Offset row index to start from
% @param starting_row: starting row of irr_csv to start from
% @param ending_row: ending row of irr_csv to end from

% @param array_layout: .json file describing the string layout of the array. 
% See Borealis_Run/string_layout.json for an example

% @param base_folder: Base folder with totPower, dayPower, storeuData and simulink files
% that will be copied num_folders number of times. See Borealis_Run/ for an example

  % Create parallel pool
  pause(2);
  delete(gcp('nocreate'));
  parpool(n_cores);
  BasePath = pwd;

  % Load in the irradiance csv
  num_rows = ending_row-starting_row+1;
  step_size = round(num_rows / num_folders); % Number of rows to assign to each matlab instance

  % Start matlab instances and simulate
  start_row = starting_row;
  for iDir = 1:num_folders

    % Create copy of base folder
    dir_name = fullfile(BasePath, "/", strcat("m", int2str(iDir)));
    copyfile(base_folder, dir_name);
    cd(dir_name);

    % Assign range of rows to the instance
    end_row = start_row + step_size - 1;
    if iDir == num_folders
      end_row = ending_row;
    end
    
    % Start the matlab instance
    irr_csv_from_instance = strcat('''', '../', irr_csv, '''');
    array_layout_from_instance = strcat('''', '../', array_layout, '''');
    command = strcat('matlab -r dayPower(', irr_csv_from_instance, ', ', array_layout_from_instance,', ',num2str(start_row), ', ', num2str(end_row), ') &');
    system(command);
    start_row = end_row + 1;

    cd("..");
    pause(12);
  end
end