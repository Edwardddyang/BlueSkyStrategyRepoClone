#include "gui.hpp"
#include <cstdlib>
#include "imgui_stdlib.h"
#include <sstream>
#include <iomanip>  // for std::setprecision
#include "ImGuiFileDialog.h"
#include "Shader.hpp"
#include "stl_reader.h"
#include "Utilities.hpp"

/* Static declarations */
std::shared_ptr<GUI> GUI::instance = nullptr;
GLFWwindow* GUI::window = nullptr;

void RenderUnderlinedText(const char* text) {
    ImGui::Text(text);  // Render the text

    // Get the current ImGui window's draw list
    ImDrawList* draw_list = ImGui::GetWindowDrawList();

    // Calculate the size and position of the text
    ImVec2 text_pos = ImGui::GetItemRectMin();  // Get the top-left corner of the text
    ImVec2 text_size = ImGui::CalcTextSize(text);

    // Define the line position (under the text) and thickness
    float line_y = text_pos.y + text_size.y;    // Position of the underline
    float thickness = 1.0f;                     // Thickness of the underline

    // Draw the underline
    draw_list->AddLine(ImVec2(text_pos.x, line_y), ImVec2(text_pos.x + text_size.x, line_y), IM_COL32(255, 255, 255, 255), thickness);
}

std::shared_ptr<GUI> GUI::get_instance() {
    if (instance == nullptr) {
        instance = std::make_shared<GUI>(GUI());
    }
    return instance;
}
void GUI::glfw_error_callback(int error, const char* description)
{
    fprintf(stderr, "GLFW Error %d: %s\n", error, description);
}

// NOTE: This function's code was almost entirely copied from ImGUI's opengl3 example
void GUI::initialize() {
    glfwSetErrorCallback(glfw_error_callback);
    if (!glfwInit())
        return;

    // Decide GL+GLSL versions
#if defined(IMGUI_IMPL_OPENGL_ES2)
    // GL ES 2.0 + GLSL 100
    const char* glsl_version = "#version 100";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
    glfwWindowHint(GLFW_CLIENT_API, GLFW_OPENGL_ES_API);
#elif defined(__APPLE__)
    // GL 3.2 + GLSL 150
    const char* glsl_version = "#version 150";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);  // 3.2+ only
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);            // Required on Mac
#else
    // GL 3.0 + GLSL 130
    const char* glsl_version = "#version 130";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
    //glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);  // 3.2+ only
    //glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);            // 3.0+ only
#endif

    // Create window with graphics context
    window = glfwCreateWindow(window_width, window_height, APP_NAME, nullptr, nullptr);
    if (window == nullptr) {
        std::cout << "Could not initialize glfw window" << std::endl;
        glfwTerminate();
        return;
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); // Enable vsync

    // Setup GLAD
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        std::cout << "Failed to initialize GLAD" << std::endl;
        return;
    }
    // Render pixels based on depth ordering
    glEnable(GL_DEPTH_TEST);

    glfwSetWindowUserPointer(window, this);

    /* Set mouse capture */
    // glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
    /* Set window callbacks */
    glfwSetCursorPosCallback(window, cursor_callback_bridge); 
    glfwSetScrollCallback(window, scroll_callback_bridge);
    glfwSetMouseButtonCallback(window, mouse_button_callback_bridge);
    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls

    // Setup Dear ImGui style
    ImGui::StyleColorsDark();
    //ImGui::StyleColorsLight();

    // Setup Platform/Renderer backends
    ImGui_ImplGlfw_InitForOpenGL(window, true);
#ifdef __EMSCRIPTEN__
    ImGui_ImplGlfw_InstallEmscriptenCanvasResizeCallback("#canvas");
#endif
    ImGui_ImplOpenGL3_Init(glsl_version);

    // Load Fonts
    // - If no fonts are loaded, dear imgui will use the default font. You can also load multiple fonts and use ImGui::PushFont()/PopFont() to select them.
    // - AddFontFromFileTTF() will return the ImFont* so you can store it if you need to select the font among multiple.
    // - If the file cannot be loaded, the function will return a nullptr. Please handle those errors in your application (e.g. use an assertion, or display an error and quit).
    // - The fonts will be rasterized at a given size (w/ oversampling) and stored into a texture when calling ImFontAtlas::Build()/GetTexDataAsXXXX(), which ImGui_ImplXXXX_NewFrame below will call.
    // - Use '#define IMGUI_ENABLE_FREETYPE' in your imconfig file to use Freetype for higher quality font rendering.
    // - Read 'docs/FONTS.md' for more instructions and details.
    // - Remember that in C/C++ if you want to include a backslash \ in a string literal you need to write a double backslash \\ !
    // - Our Emscripten build process allows embedding fonts to be accessible at runtime from the "fonts/" folder. See Makefile.emscripten for details.
    //io.Fonts->AddFontDefault();
    //io.Fonts->AddFontFromFileTTF("c:\\Windows\\Fonts\\segoeui.ttf", 18.0f);
    //io.Fonts->AddFontFromFileTTF("../../misc/fonts/DroidSans.ttf", 16.0f);
    //io.Fonts->AddFontFromFileTTF("../../misc/fonts/Roboto-Medium.ttf", 16.0f);
    //io.Fonts->AddFontFromFileTTF("../../misc/fonts/Cousine-Regular.ttf", 15.0f);
    //ImFont* font = io.Fonts->AddFontFromFileTTF("c:\\Windows\\Fonts\\ArialUni.ttf", 18.0f, nullptr, io.Fonts->GetGlyphRangesJapanese());
    //IM_ASSERT(font != nullptr);
    mouse_control = false;
}

void GUI::render() {
    // Main render loop
    while (!glfwWindowShouldClose(window))
    {
        initialize_new_frame();
        // continue;
        /* Step selection window pane */
        render_step_selection_pane();

        /* Simulation configuration pane */
        render_simulation_configuration_pane();

        /* Render the frame */
        render_frame();
    }
}

void GUI::render_step_selection_pane() {
    set_window_size_position_on_resize(step_selection_size, step_selection_position);

    ImGui::Begin("Steps");

    ImGui::RadioButton("Step 1", &selected_step, static_cast<int>(steps::STEP_1));
    insert_tooltip("Create a sun position CSV");

    ImGui::RadioButton("Step 2", &selected_step, static_cast<int>(steps::STEP_2));
    insert_tooltip("Create effective irradiance csv");

    ImGui::RadioButton("Visualization", &selected_step, static_cast<int>(steps::STEP_3));
    insert_tooltip("Visualizations of the sun path csv (step 1) or the array heat map (step 2)");

    // Recalculate the simulation configuration window component size/positions
    // when the application resizes or when the selected step changes
    if (window_resized || selected_step != last_selected_step) {
        if (selected_step == static_cast<int>(steps::STEP_1)) {
            help_popup_size = ImVec2(
                window_width*help_popup_width_fraction, window_height*help_popup_height_fraction
            );
            help_popup_position = ImVec2( // Center of the screen
                window_width * 0.5f - help_popup_size[WIDTH_INDEX] * 0.5f,
                window_height * 0.5f - help_popup_size[HEIGHT_INDEX] * 0.5f
            );

            num_timesteps_width = sim_config_size[WIDTH_INDEX] * num_timesteps_width_fraction;
            location_width = sim_config_size[WIDTH_INDEX] * location_width_fraction;
            time_field_width = sim_config_size[WIDTH_INDEX] * time_field_width_fraction;
            text_field_width = sim_config_size[WIDTH_INDEX] * text_field_width_fraction;
        } else if (selected_step == static_cast<int>(steps::STEP_2)) {
            button_size = ImVec2(button_width_fraction*sim_config_size[WIDTH_INDEX], button_height);
        } else if (selected_step == static_cast<int>(steps::STEP_3)) {
            irr_row_width = sim_config_size[WIDTH_INDEX] * irr_row_width_fraction;
            button_size = ImVec2(button_width_fraction*sim_config_size[WIDTH_INDEX], button_height);
        }
    }

    // Help popup
    if (ImGui::Button("Help")) {
        ImGui::OpenPopup("Guide");
        is_help_popup_open = true;
    }

    if (is_help_popup_open) {
        set_window_size_position_on_resize(help_popup_size, help_popup_position);
    }

    if (ImGui::BeginPopupModal("Guide", NULL, 0))
    {
        ImGui::PushTextWrapPos(ImGui::GetWindowContentRegionMax().x);
        ImGui::Text("The array simulation is split into three distinct steps:");
        ImGui::Text("1. Generate a 4 column csv with format"
                    " | Azimuth (degrees) | Elevation (degrees) | Irradiance (W/m^2) | Timestamp (24 hour) | "
                    "describing the path and intensity of the sun throughout the simulation day.");
        ImGui::Text("2. Generate a (number of timestamps) x (number of array cells) csv "
                    "where each cells stores the effective irradiance acting on the cell. "
                    "Note that (number of timestamps) = (number of rows in csv from step 1)");
        ImGui::Text("3. Generate the final cumulative array power for each timestamp by running a "
                    "simulink model to simulate the bypass diode behaviour in the electrical layout "
                    "of the strings.");
        ImGui::Text("This application can perform steps 1 and 2 with primary focus on step 2 along with its visualization. "
                    "To run either step, simply select it from the selection pane and configure the options in the window pane below");
        ImGui::PopTextWrapPos();
        if (ImGui::Button("Close")) {
            ImGui::CloseCurrentPopup();
            is_help_popup_open = false;
        }
        ImGui::EndPopup();
    }

}

void GUI::render_simulation_configuration_pane() {
    if (window_resized || last_selected_step != selected_step) {
        set_window_size_position(sim_config_size, sim_config_position);
    }

    if (selected_step == static_cast<int>(steps::STEP_1)) {
        render_step_one_layout();
    } else if (selected_step == static_cast<int>(steps::STEP_2)) {
        render_step_two_layout();
    } else if (selected_step == static_cast<int>(steps::STEP_3)) {
        render_step_three_layout();
    }
    ImGui::End();
}

void GUI::render_step_one_layout() {
    ImGui::Begin("Step 1 - Generate Sun Path CSV");
    ImGui::SetNextItemWidth(num_timesteps_width);
    ImGui::InputInt("No. Timesteps", &num_timesteps);
    insert_tooltip("Number of sun positions to generate from the Start Time [inclusive] to the End Time");

    ImGui::SetNextItemWidth(location_width);
    ImGui::InputFloat2("Latitude / Longitude", location, "%.3f");
    insert_tooltip("Static location on earth where the sun position will be generated");

    ImGui::NewLine();
    ImGui::SetNextItemWidth(text_field_width);
    ImGui::InputText("Start Time", &start_time);
    insert_tooltip("Start time of the simulation day in 24 hour local time with format: YYYY-MM-DD HH:MM:SS");

    ImGui::NewLine();
    ImGui::SetNextItemWidth(text_field_width);
    ImGui::InputText("End Time", &end_time);
    insert_tooltip("End time of the simulation day in 24 hour local time with format: YYYY-MM-DD HH:MM:SS");

    ImGui::NewLine();
    ImGui::Text("UTC Adjustment");
    insert_tooltip("Adjustment from local time to UTC time in HH:MM:SS format. Forwards implies (+) adjustment, Backward implies (-) adjustment.\n"
                   "E.g. 9:30:00 local time with a UTC adjustment of 4:30:00 hours forward implies 14:00:00 UTC time");
    ImGui::RadioButton("Forward", &adjustment, FORWARD_ADJUSTMENT);
    ImGui::SameLine();
    ImGui::RadioButton("Backward", &adjustment, BACKWARD_ADJUSTMENT);
    ImGui::SetNextItemWidth(text_field_width);
    ImGui::InputText("Adjustment", &utc_adjustment);

    ImGui::NewLine();
    ImGui::SetNextItemWidth(text_field_width);
    ImGui::InputText("Output File Name", &output_file_buffer);
    if (ImGui::Button("Create")) {
        std::string sign = adjustment == FORWARD_ADJUSTMENT ? "+" : "-";
        std::ostringstream oss;
        oss << "python dayAzElIrr.py " << "--lat " << std::fixed << std::setprecision(6) << location[0]
                                       << " --lon " << std::fixed << std::setprecision(6) << location[1]
                                       << " --start_time \"" << start_time << "\""
                                       << " --end_time \"" << end_time << "\""
                                       << " --utc_adjustment " << sign << utc_adjustment
                                       << " --num_timesteps " << num_timesteps;
        if (output_file_buffer != "") {
            oss << " --out_csv " << output_file_buffer;
        }
        std::cout << oss.str() << std::endl;
        
        system(oss.str().c_str());
    }
}

void GUI::render_step_two_layout() {    
    ImGui::Begin("Step 2 - Effective Irradiance CSV");
    std::filesystem::path path; // Dummy variable
    insert_file_dialog_button("Array Cell STL Directory", &button_size,"arraySTLDir", "Choose Directory", nullptr, path, cell_stl_folder_path);
    insert_tooltip("Directory that exclusively contains all the STL files for the array cells");
    insert_file_dialog_button("Canopy STL File", &button_size, "canopySTLFile", "Choose File", ".stl", canopy_stl_file_path, path);
    insert_tooltip("STL file of the canopy");
    insert_file_dialog_button("Sun Positions CSV", &button_size, "sunPositionPath", "Choose File", ".csv", sun_positions_path, path);
    insert_tooltip("CSV file describing the path of the sun generated according to step 1.");
    insert_file_dialog_button("Route CSV", &button_size, "routePath", "Choose File", ".csv", route_path, path);
    insert_tooltip("CSV file of the route. This should be used ONLY for dynamic simulations");
    ImGui::RadioButton("Dynamic Simulation", &sim_type, static_cast<int>(SimType::DYNAMIC));
    ImGui::RadioButton("Static Simulation", &sim_type, static_cast<int>(SimType::STATIC));

    ImGui::SetNextItemWidth(text_field_width);
    ImGui::InputText("Direction", &direction);
    insert_tooltip("Direction of the nose of the car. Usually \"-x\". Confirm with the CAD of the car");

    ImGui::SetNextItemWidth(text_field_width);
    ImGui::InputText("Bearing", &bearing_text);
    if (bearing_text != "" && isDouble(bearing_text)) {
        bearing = std::stod(bearing_text);
    }
    insert_tooltip("The clockwise angle of the nose of the car from true north in degrees\n"
                   "when positioned at the location in the sun path csv. This should ONLY be used\n"
                   "for a static simulation. Example: In WSC, if the sun position csv was generated\n"
                   "at Alice Springs, then this would be 180 since the car is facing south at that\n"
                   "point during the race.");

    ImGui::SetNextItemWidth(text_field_width);
    ImGui::InputFloat("Car Speed", &car_speed);
    insert_tooltip("Speed that the car travels around the Route in m/s.\n"
                    "This should only be used for a dynamic simulation.");

    ImGui::SetNextItemWidth(text_field_width);
    ImGui::InputText("Start Route Time", &start_route_time);
    insert_tooltip("Start time of the simulation in 24 hour local time with format: YYYY-MM-DD HH:MM:SS\n"
                    "This must be within the time range of the sun position csv. This should only be used\n"
                    "for a dynamic simulation.");

    ImGui::SetNextItemWidth(text_field_width);
    ImGui::InputText("End Route Time", &end_route_time);
    insert_tooltip("End time of the simulation in 24 hour local time with format: YYYY-MM-DD HH:MM:SS\n"
                    "This must be within the time range of the sun position csv. This should only be used\n"
                    "for a dynamic simulation.");

    ImGui::SetNextItemWidth(text_field_width);
    ImGui::InputText("Output Name", &irradiance_csv_name);
    insert_tooltip("Name of the output irradiance csv");
    ImGui::NewLine();

    if (ImGui::Button("Generate Irradiance CSV")) {
        std::cout << "Generating irradiance csv..." << std::endl;
        car_model = std::make_shared<Model>();
        car_model->loadCanopy(canopy_stl_file_path.string());
        car_model->loadArray(cell_stl_folder_path.string());
        sun_position_lut = std::make_shared<SunPositionLUT>(sun_positions_path);

        irradiance_csv = std::make_shared<CellIrradianceSim>(false);
        car_speed = kph2mps(car_speed);
        if (sim_type == static_cast<int>(SimType::DYNAMIC)) {
            route_lut = std::make_shared<RouteLUT>(route_path);
            irradiance_csv->run_dynamic_sim(sun_position_lut, car_model, route_lut, (double)car_speed, direction,
                                            start_route_time, end_route_time);
            irradiance_csv->write_dynamic_csv(irradiance_csv_name, "metadata.csv");
        } else {
            irradiance_csv->run_static_sim(sun_position_lut, car_model, bearing, direction);
            irradiance_csv->write_static_csv(irradiance_csv_name);
        }
    }
}

void GUI::render_step_three_layout() {
    ImGui::Begin("Visualizations");
    RenderUnderlinedText("See Canopy and Array");
    std::filesystem::path path;
    insert_file_dialog_button("Array Cell STL Directory", &button_size,"arraySTLDir", "Choose Directory",
                             nullptr, path, cell_stl_folder_path_v);
    insert_file_dialog_button("Canopy STL File", &button_size, "canopySTLFile", "Choose File",
                             ".stl", canopy_stl_file_path_v, path);
    insert_file_dialog_button("Sun Position CSV", &button_size, "SunCsvFile", "Choose File", ".csv", sun_positions_path, path);
    insert_file_dialog_button("Irradiance CSV", &button_size, "csvFile", "Choose File", ".csv", irradiance_csv_file_path, path);
    insert_file_dialog_button("Metadata CSV", &button_size, "metadataFile", "Choose File", ".csv", metadata_csv_file_path, path);
    ImGui::SetNextItemWidth(irr_row_width);
    ImGui::InputInt("Sun Position Row", &irr_row);
    insert_tooltip("The row in the irradiance csv to display. This is equivalent\n"
                    "to the desired sun position row in the sun positions csv.");

    ImGui::SetNextItemWidth(irr_row_width);
    ImGui::InputInt("Cell Highlight", &cell_idx);
    insert_tooltip("The cell to highlight in white");

    // Sun position information
    if (irradiance_visualized && (0 < irr_row < num_csv_rows) && sun_position_lut != nullptr) {
        time_of_day = "Time Of Day: " + sun_position_lut->get_time(irr_row);
        azimuth = "Azimuth: " + std::to_string(sun_position_lut->get_azimuth_value(irr_row));
        elevation = "Elevation: " + std::to_string(sun_position_lut->get_elevation_value(irr_row));
        irradiance = "Irradiance: " + std::to_string(sun_position_lut->get_irradiance_value(irr_row));
    }

    // Display information if the user has requested heat map mode
    if (irradiance_visualized && (0 < irr_row < num_csv_rows)) {
        if (show_dynamic_params) {
            size_t sun_position_idx = irradiance_csv->get_sun_position_cache_value(irr_row);
            time_of_day = "Time Of Day: " + sun_position_lut->get_time(sun_position_idx);
            azimuth = "Azimuth: " + std::to_string(sun_position_lut->get_azimuth_value(sun_position_idx));
            elevation = "Elevation: " + std::to_string(sun_position_lut->get_elevation_value(sun_position_idx));
            irradiance = "Irradiance: " + std::to_string(sun_position_lut->get_irradiance_value(sun_position_idx));  
            bearing_label = "Car Bearing: " + std::to_string(irradiance_csv->get_bearing_value(irr_row));
            Coord coord = irradiance_csv->get_coordinate_value(irr_row);
            coordinate_label = "Coordinates of Car: " + std::to_string(coord.lat) + ", " + std::to_string(coord.lon) + ", " + std::to_string(coord.alt);
        } else {
            time_of_day = "Time Of Day: " + sun_position_lut->get_time(irr_row);
            azimuth = "Azimuth: " + std::to_string(sun_position_lut->get_azimuth_value(irr_row));
            elevation = "Elevation: " + std::to_string(sun_position_lut->get_elevation_value(irr_row));
            irradiance = "Irradiance: " + std::to_string(sun_position_lut->get_irradiance_value(irr_row));
        }

        if (0 < cell_idx < num_cells) {
            cell_irradiance = "Highlighted Cell Irradiance: " + std::to_string(irradiance_csv->get_irr_value(irr_row, cell_idx));
        }
    }
    if (time_of_day != "") {
        ImGui::Text(time_of_day.c_str());
    }
    if (azimuth != "") {
        ImGui::Text(azimuth.c_str());
    }
    if (elevation != "") {
        ImGui::Text(elevation.c_str());
    }
    if (irradiance != "") {
        ImGui::Text(irradiance.c_str());
    }
    if (bearing_label != "") {
        ImGui::Text(bearing_label.c_str());
    }
    if (coordinate_label != "") {
        ImGui::Text(coordinate_label.c_str());
    }
    if (cell_irradiance != "") {
        ImGui::Text(cell_irradiance.c_str());
    }  

    ImGui::NewLine();
    // If we have both a canopy STL and the array cell stl folder, render them
    if (ImGui::Button("See Car", button_size)) {
        std::cout << "Rendering car..." << std::endl;
        // Reset visualization variables
        car_visualized = true;
        first_mouse_movement = true;
        mouse_control = true;
        delta_time = 0.0f;
        last_frame = 0.0f;
        glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
        car_model = std::make_shared<Model>();
        car_model->loadCanopy(canopy_stl_file_path_v.string());
        car_model->loadArray(cell_stl_folder_path_v.string());
        car_model->calc_centroid();
        car_model->center_model();
        car_model->init_camera();
        car_model->init_shaders();
        glm::vec3 min = car_model->get_min_values();
        glm::vec3 max = car_model->get_max_values();
        std::cout << "Finished rendering car" << std::endl;
    }

    if (ImGui::Button("See Irradiance Map", button_size)) {
        std::cout << "Rendering car..." << std::endl;
        // Reset visualization variables
        car_visualized = true;
        first_mouse_movement = true;
        mouse_control = true;
        irradiance_visualized = true;
        delta_time = 0.0f;
        last_frame = 0.0f;

        if (!sun_positions_path.empty()) {
            sun_position_lut = std::make_shared<SunPositionLUT>(sun_positions_path);
        }

        if (!metadata_csv_file_path.empty()) {
            show_dynamic_params = true;
            irradiance_csv = std::make_shared<CellIrradianceSim>(irradiance_csv_file_path,
                                                                metadata_csv_file_path);
        } else {
            show_dynamic_params = false;
            irradiance_csv = std::make_shared<CellIrradianceSim>(irradiance_csv_file_path);
        }
        
        std::vector<double> irradiance_values;
        std::pair<double, double> irradiance_limits = irradiance_csv->get_irradiance_limits();
        std::vector<std::vector<double>> values = irradiance_csv->get_irradiance_csv();
        num_csv_rows = values.size();
        for (size_t i = 0; i < num_csv_rows; i++) {
            std::vector<glm::vec3> colours;
            std::vector<double> irradiance_values = values[i];
            for (size_t j = 0; j < irradiance_values.size(); j++) {
                double normalized_value = (irradiance_values[j] - irradiance_limits.first) /
                                      (irradiance_limits.second - irradiance_limits.first);
                const tinycolormap::Color color = tinycolormap::GetColor(normalized_value, tinycolormap::ColormapType::Jet);
                colours.push_back(glm::vec3(color.r(), color.g(), color.b()));
            }
            cell_colours.push_back(colours);
        }
        glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
        car_model = std::make_shared<Model>();
        car_model->init_shaders();
        car_model->loadCanopy(canopy_stl_file_path_v.string());
        car_model->loadArray(cell_stl_folder_path_v.string());
        car_model->calc_centroid();
        car_model->center_model();
        car_model->init_camera();
        num_cells = car_model->get_array_cell_meshes().size();
        std::cout << "Finished rendering car" << std::endl;    
    }

    if (car_visualized) {
        // Draw the meshes
        float current_frame = glfwGetTime();
        delta_time = current_frame - last_frame;
        last_frame = current_frame;
        process_input(window);
        std::vector<double> irradiance_values = {};
        std::pair<double, double> irradiance_limits = {};
        std::vector<glm::vec3> colours = {};
        if (irradiance_csv != nullptr && irradiance_visualized) {
            if (irr_row >= num_csv_rows || irr_row < 0) irr_row = 0;
            irradiance_values = irradiance_csv->get_irradiance_csv()[irr_row];
            irradiance_limits = irradiance_csv->get_irradiance_limits();
            colours = cell_colours[irr_row];
            if (-1 > cell_idx || cell_idx >= num_cells) cell_idx = -1;
        }
        car_model->Draw(window_width, window_height, colours, cell_idx, true);
    }
}

void GUI::insert_file_dialog_button(const char* button_name, 
                                   ImVec2* button_size,
                                   const std::string key, 
                                   const std::string window_title,
                                   const char* filter,
                                   std::filesystem::path& file_path,
                                   std::filesystem::path& folder_path) {
    ImVec2 size;
    if (button_size == nullptr) {
        size = ImVec2(0,0);
    } else {
        size = *button_size;
    }
    if (ImGui::Button(button_name, size)) {
        ImGui::SetNextWindowPos(file_dialog_position);
        ImGui::SetNextWindowSize(file_dialog_size);
        IGFD::FileDialogConfig config;
        config.path = ".";
        ImGuiFileDialog::Instance()->OpenDialog(key, window_title, filter, config);
    }

    if (ImGuiFileDialog::Instance()->Display(key)) {
        if (ImGuiFileDialog::Instance()->IsOk()) {
            folder_path = ImGuiFileDialog::Instance()->GetCurrentPath();
            file_path = folder_path / ImGuiFileDialog::Instance()->GetCurrentFileName();
        }
        ImGuiFileDialog::Instance()->Close();
    }
}

void GUI::initialize_new_frame() {
    io = ImGui::GetIO();

    // Window component size and positions
    window_width = io.DisplaySize.x;
    window_height = io.DisplaySize.y;

    // File dialog popup dimensions and position
    file_dialog_size = ImVec2(
        window_width*file_dialog_width_fraction, window_height*file_dialog_height_fraction
    );
    file_dialog_position = ImVec2(
        window_width * 0.5f - file_dialog_size[WIDTH_INDEX] * 0.5f,
        window_height * 0.5f - file_dialog_size[HEIGHT_INDEX] * 0.5f
    );

    // Recompute the window sizes upon the application resizing
    if (window_width != prev_window_width || window_height != prev_window_height) {
        window_resized = true;
        // Simulation configuration window
        sim_config_size = ImVec2(
            window_width*sim_config_width_fraction, window_height*sim_config_height_fraction
        );
        sim_config_position = ImVec2(
            0.0f, window_height*step_selection_height_fraction
        );
        // Step selection window
        step_selection_size = ImVec2(
            window_width*step_selection_width_fraction, window_height*step_selection_height_fraction
        );
    } else {
        window_resized = false;
    }
    glClearColor(clear_color.x * clear_color.w, clear_color.y * clear_color.w, clear_color.z * clear_color.w, clear_color.w);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
}

void GUI::set_window_size_position(const ImVec2 size, const ImVec2 position) const {
    ImGui::SetNextWindowSize(size, ImGuiCond_Always);
    ImGui::SetNextWindowPos(position, ImGuiCond_Always);
}

void GUI::set_window_size_position_on_resize(const ImVec2 size, const ImVec2 position) const {
    if (window_resized) {
        ImGui::SetNextWindowSize(size, ImGuiCond_Always);
        ImGui::SetNextWindowPos(position, ImGuiCond_Always);
    }
}

void GUI::insert_tooltip(const char* tip) const {
    ImGui::SameLine();
    ImGui::TextDisabled("(?)");
    if (ImGui::IsItemHovered())
    {
        ImGui::BeginTooltip();
        ImGui::Text(tip);
        ImGui::EndTooltip();
    }
}

void GUI::render_frame() {
    ImGui::End();
    ImGui::Render();
    int display_w, display_h;
    glfwGetFramebufferSize(window, &display_w, &display_h);
    glViewport(0, 0, display_w, display_h);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    glfwSwapBuffers(window);
    glfwPollEvents();  // Mouse + keyboard input

    // Save current frame information
    prev_window_height = window_height;
    prev_window_width = window_width;
    last_selected_step = selected_step;
}

void GUI::cleanup() {
    // Cleanup
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();
}

void GUI::cursor_callback_bridge(GLFWwindow* window, double xpos, double ypos) {
    instance->cursor_callback(xpos, ypos);
}

void GUI::cursor_callback(double xpos, double ypos) {
    if (!car_visualized || !mouse_control) {
        return;
    }

    if (first_mouse_movement)
    {
        last_x = xpos;
        last_y = ypos;
        first_mouse_movement = false;
    }

    float xoffset = xpos - last_x;
    float yoffset = last_y - ypos; 
    last_x = xpos;
    last_y = ypos;

    car_model->camera->ProcessMouseMovement(xoffset, yoffset);
}

void GUI::process_input(GLFWwindow *window)
{
    if (!car_visualized) {
        return;
    }

    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
        glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
        last_key_pressed = GLFW_KEY_ESCAPE;
        mouse_control = false;
    }
    if (glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS) {
        glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
        last_key_pressed = GLFW_KEY_Q;
        mouse_control = true;
    }
    
    if (irradiance_visualized) {
        bool perform_row_mod = false;
        bool valid_key_pressed = true;
        int key_pressed;
        int irr_mod;
        if (glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS) {
            key_pressed = GLFW_KEY_RIGHT;
            irr_mod = 1;
        } else if (glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS) {
            key_pressed = GLFW_KEY_LEFT;
            irr_mod = -1;
        } else {
            valid_key_pressed = false;
        }

        if (valid_key_pressed) {
            if (last_key_pressed != key_pressed) {
                key_press_start = glfwGetTime();
            } else {
                float key_press_duration = glfwGetTime() - key_press_start;
                if (key_press_duration >= key_press_duration_threshold) {
                    perform_row_mod = true;
                    key_press_start = glfwGetTime();
                }
            }

            last_key_pressed = key_pressed;
            if (!perform_row_mod) return;
            irr_row = irr_row + irr_mod;
        }
    }

    if (!mouse_control) return;

    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) {
        car_model->camera->ProcessKeyboard(Camera_Movement::FORWARD, delta_time);
        last_key_pressed = GLFW_KEY_W;
    }
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) {
        car_model->camera->ProcessKeyboard(Camera_Movement::BACKWARD, delta_time);
        last_key_pressed = GLFW_KEY_S;
    }
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) {
        car_model->camera->ProcessKeyboard(Camera_Movement::LEFT, delta_time);
        last_key_pressed = GLFW_KEY_A;
    }
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) {
        car_model->camera->ProcessKeyboard(Camera_Movement::RIGHT, delta_time);
        last_key_pressed = GLFW_KEY_D;
    }
}

void GUI::scroll_callback_bridge(GLFWwindow* window, double xpos, double ypos) {
    instance->scroll_callback(xpos, ypos);
}

void GUI::scroll_callback(double xpos, double ypos) {
    if (!car_visualized || !mouse_control) {
        return;
    }

    car_model->camera->ProcessMouseScroll(static_cast<float>(ypos));
}

void GUI::mouse_button_callback_bridge(GLFWwindow* window, int button, int action, int mods) {
    instance->mouse_button_callback(button, action, mods);
}

void GUI::mouse_button_callback(int button, int action, int mods) {
    if (button == GLFW_MOUSE_BUTTON_LEFT) {
        if (action == GLFW_PRESS)
            is_left_click_held = true;
        else if (action == GLFW_RELEASE)
            is_left_click_held = false;
    }
}
