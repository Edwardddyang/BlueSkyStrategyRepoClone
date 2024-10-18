# Array Simulation

This array simulation aims to simulate the power output of a solar car array across a variety of azimuth/elevation positions. It first generates effective irradiances on cells and simulates the electrical layout of the array via. a simulink model. Before running the following steps, please make sure that the you have an stl file for the canopy and a folder holding the stl files for **each** array cell named with ascending integers ex. NA(1).stl . You can do this from spaceclaim and export by bodies. The units **must** be in mm.

The simulation supports two different modes.
- <ins>Static Simulation</ins>: The car is located at one location on earth and faces some direction for the entire duration of the simulation. The sun rotates about the car. This is an appropriate use case for WSC where the car faces south for most of the race.
- <ins>Dynamic Simulation</ins>: The car moves through a route at some constant speed. The position of the sun will depend on the time of day and the location of the car in the route. This simulation is appropriate for FSGP where the car is constantly changing its bearing at each moment.

1. Generate an n x 3 csv that describes sun position and irradiances to simulate ```Azimuth(Degrees) | Elevation(Degrees) | Irradiance(W/m^2)``` This can be done by running ```python dayAzElIrr.py --lat <lat> --lon <lon> --start_time <utc start time> --end_time <utc end time> --utc_adjustment <signed adjustment in HH:MM:SS> --num_timesteps <number of timesteps> --out_csv <output csv file>``` in ./GetSunPosition in order to simulate the car's power input throughout a day at some static location on earth. You could also manually generate one if you have a more specific configuration. The former is generally used in order to compute array layout performances against one another and the latter is used in order to compute a lookup table for the selected array. You can also do this through step 1 of the C++ GUI application, though running the Python script from the command line is much easier.
    - Convention: Azimuth is taken to be degrees clockwise from true north and can range from 0 to 360. Elevation ranges from 0 to 90.

2. Generate an (n x number of cells) where the value at index (i,j) is the effective irradiance in W/m^2 on cell j when the sun is positioned according to a specific row from the csv generated in step 1. For a static simulation, the position of the sun is exactly row j in the sun positions csv. For a dynamic simulation, it entirely depends on where the car is located in the route at what time. Canopy and array shading are taken into account by constructing an AABB tree of the canopy and drawing a ray from the centroid of each array cell triangle towards the plane of the sun. If the ray intersects the canopy, we say that the triangle is partially shaded. If the imprecise calculation is used, we consider these triangles to be entirely shaded. If we are precisely calculating partial shading, we'll generate 1000 rays from the triangle and define [(intersecting rays / 1000) * area of triangle] as the shaded area. Naturally, the precise calculation is much much slower and we only use it to generate the final lookup table. To run this simulation, follow step 2 of the C++ GUI application. Note that if you run a dynamic simulation, then an additional metadata csv will be generated that holds the following information ```Bearing(degrees)|Latitude(degrees)|Longitude(degrees)|Altitude(m)|Time(YYYY-MM-DD HH:MM:SS)|Sun Position Index Cache|```
    - Convention: The stl of the car MUST be oriented such that the nose of the car points in one of the +/- x, y directions while the canopy must be pointing in the +z direction. Bearing of the car is taken to be the degrees from true north.

    The MATLAB version found in GetCellIrradiance has a slightly less advanced version of the algorithm but it doesn't need the C++ application to be built. You can run it by running ```./GetCellIrradiance/main(<number of cells>, <bearing of the car>, <output csv name>, <Path to canopy stl>, <Path to array cell stls>, <Path to input csv>)```. Before this, you must compile the mesh ray tracing library by going to ./src_matlab and running mexall.m which compiles the mex code and copies it to the ./B/Run folder. You may need to modify the compile command in mexall.m to be compatible with your C++ compiler. Generally, the compiler that comes with the mingw_w64 MATLAB add-on works smoothly. Refer to https://www.mathworks.com/help/matlab/matlab_external/install-mingw-support-package.html for installation.

2a. After generating the effective irradiance CSV, you can visualize the entire array by following the Visualization step inside the C++ GUI application. When visualizing, you can use the following controls:
   - Left and right arrow keys to navigate through the cell irradiances shown according to the irradiance CSV generated in step 2.
   - Escape to exit out of mouse control orbiting.
   - Q to enter into mouse control orbiting.

   Below is an example of the visualization tool for the results of a static simulation. Build examples can be found inside CPP/

https://github.com/user-attachments/assets/a1e3b0fe-b9b7-4c5d-ac96-9a4bef312d51

3. Get the total power output of the entire array in W by feeding the output csv from step 2 into a custom simulink model that simulates either bypass diodes or mppt controllers. This simulink model must be custom made for each array layout in order to adhere to string configurations.

# Limitations

- Canopy is interpreted as opaque in step 2. In practice, this isn't too big of an issue since a large amount of the volume would be blocked by the driver and rollcage.
- Solar cells are used for 100% of their area. However, most silicon cells have a very small gap between the edge of the cell and the actual edge of the silicon
- Step 3 does not factor in performance under temperature

# Improvements

- Seperate out the irradiance generation simulation from the C++ GUI application as it's not practical to get people to build the C++ application just to run some command line arguments
- In the visualization step of the C++ GUI, add a checkbox to allow users to see just the static simulation or a dynamic simulation. Right now, if the user picks a metadata csv, they can only view a dynamic result. They would have to close the application to be able to view a static simulation's reult
- Multithread the irradiance generation simulation
- The C++ GUI Application is still not very robust to errors
- The build process for the C++ GUI application is terrible
