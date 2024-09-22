#include "gui.hpp"
#include <cstdlib>
#include "imgui_stdlib.h"
#include <sstream>
#include <iomanip>
#include "ImGuiFileDialog.h"
#include "Shader.hpp"
#include "stl_reader.h"
#include <chrono>
#include <date.h>
#include "Utilities.hpp"

/* Static declarations */
std::unique_ptr<GUI> GUI::instance = nullptr;
GLFWwindow* GUI::window = nullptr;

void GUI::render_underlined_text(const char* text) {
    ImGui::Text(text);  // Render the text

    // Get the current ImGui window's draw list
    ImDrawList* draw_list = ImGui::GetWindowDrawList();

    // Calculate the size and position of the text
    ImVec2 text_pos = ImGui::GetItemRectMin();  // Get the top-left corner of the text
    ImVec2 text_size = ImGui::CalcTextSize(text);
    float line_y = text_pos.y + text_size.y;
    float thickness = 1.0f;

    // Draw the underline
    draw_list->AddLine(ImVec2(text_pos.x, line_y), ImVec2(text_pos.x + text_size.x, line_y),
                       IM_COL32(255, 255, 255, 255), thickness);
}

std::unique_ptr<GUI>& GUI::get_instance() {
    if (instance == nullptr) {
        instance = std::make_unique<GUI>(GUI());
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
    if (!glfwInit()) {
        std::cout << "GLFW could not be initialized. Exiting" << std::endl;
        exit(1);  
    }

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
        std::cout << "Could not initialize glfw window. Exiting" << std::endl;
        glfwTerminate();
        exit(1);
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); // Enable vsync

    // Setup GLAD
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        std::cout << "Failed to initialize GLAD. Exiting" << std::endl;
        exit(1);
    }
    // Render pixels based on depth ordering
    glEnable(GL_DEPTH_TEST);
    
    /* Set window callbacks */
    glfwSetWindowUserPointer(window, this);
    glfwSetWindowFocusCallback(window,window_iconify_callback);
    glfwSetCursorPosCallback(window, cursor_callback); 
    glfwSetScrollCallback(window, scroll_callback);

    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls
    ImGui::StyleColorsDark();

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
}

void GUI::render() {
    // Main render loop
    while (!glfwWindowShouldClose(window))
    {
        // Get mouse and keyboard inputs
        glfwPollEvents();

        if (is_minimized) continue;

        initialize_new_frame();

        /* Step selection window pane */
        render_step_selection_pane();

        /* Simulation configuration pane */
        render_simulation_configuration_pane();

        /* Error message popup */
        render_error_message_popup();

        /* Render the frame */
        render_frame();
    }
}

void GUI::render_error_message_popup() {
    if (!is_error_popup_open) return;

    ImGui::OpenPopup("Error");
    set_window_size_position(error_popup_size, error_popup_position);
    if (ImGui::BeginPopupModal("Error", NULL, 0))
    {
        ImGui::PushTextWrapPos(ImGui::GetWindowContentRegionMax().x);
        ImGui::Text(error_popup_message.c_str());
        ImGui::PopTextWrapPos();
        if (ImGui::Button("Close")) {
            ImGui::CloseCurrentPopup();
            is_error_popup_open = false;
            error_popup_message = "";
        }
        ImGui::EndPopup();
    }
}

void GUI::render_step_selection_pane() {
    // Adjust the size only when the application window is resized
    set_window_size_position_on_resize(step_selection_size, step_selection_position);

    ImGui::Begin("Steps");

    ImGui::RadioButton("Step 1", &selected_step, static_cast<int>(steps::STEP_1));
    insert_tooltip("Create a sun position CSV");

    ImGui::RadioButton("Step 2", &selected_step, static_cast<int>(steps::STEP_2));
    insert_tooltip("Create effective irradiance csv");

    ImGui::RadioButton("Visualization", &selected_step, static_cast<int>(steps::STEP_3));
    insert_tooltip("Visualizations of the sun path csv (step 1) or the array heat map (step 2)");

    // Recalculate the simulation configuration pane size/positions
    // when the application resizes or when the selected step changes
    if (window_resized || selected_step != last_selected_step) {
        error_popup_size = ImVec2(
            window_width * error_popup_width_fraction,
            window_height * error_popup_height_fraction
        );
        error_popup_position = ImVec2( // Center of the screen
            window_width * 0.5f - error_popup_size[WIDTH_INDEX] * 0.5f,
            window_height * 0.5f - error_popup_size[HEIGHT_INDEX] * 0.5f
        );
        if (selected_step == static_cast<int>(steps::STEP_1)) {
            help_popup_size = ImVec2(
                window_width * help_popup_width_fraction,
                window_height * help_popup_height_fraction
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
        first_frame_help_render = true;
    }

    if (is_help_popup_open) {
        if (first_frame_help_render) {
            set_window_size_position(help_popup_size, help_popup_position);
            first_frame_help_render = false;
        } else {
            set_window_size_position_on_resize(help_popup_size, help_popup_position);
        }
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
    ImGui::InputText("Start Time", &start_time_buffer);
    insert_tooltip("Start time of the simulation day in 24 hour local time with format: YYYY-MM-DD HH:MM:SS");

    ImGui::NewLine();
    ImGui::SetNextItemWidth(text_field_width);
    ImGui::InputText("End Time", &end_time_buffer);
    insert_tooltip("End time of the simulation day in 24 hour local time with format: YYYY-MM-DD HH:MM:SS");

    ImGui::NewLine();
    ImGui::Text("UTC Adjustment");
    insert_tooltip("Adjustment from local time to UTC time in HH:MM:SS format. Forwards implies (+) adjustment, Backward implies (-) adjustment.\n"
                   "E.g. 9:30:00 local time with a UTC adjustment of 4:30:00 hours forward implies 14:00:00 UTC time");
    ImGui::RadioButton("Forward", &utc_adjustment_direction, FORWARD_ADJUSTMENT);
    ImGui::SameLine();
    ImGui::RadioButton("Backward", &utc_adjustment_direction, BACKWARD_ADJUSTMENT);
    ImGui::SetNextItemWidth(text_field_width);
    ImGui::InputText("Adjustment", &utc_adjustment);

    ImGui::NewLine();
    ImGui::SetNextItemWidth(text_field_width);
    ImGui::InputText("Output File Name", &output_file_buffer);
    if (ImGui::Button("Create")) {
        std::string sign = utc_adjustment_direction == FORWARD_ADJUSTMENT ? "+" : "-";
        // Check for input argument validity
        if (!check_dayAzElIrr_args()) {
            return;
        }
        try {
            std::ostringstream oss;
            const char* array_root = std::getenv("ARRAY_ROOT");
            RUNTIME_ASSERT(array_root != nullptr, "ARRAY_ROOT environment variable not found. Set it to the full path to gen12_strategy/ArraySim.");
            oss << "python $ARRAY_ROOT/GetSunPosition/dayAzElIrr.py " << "--lat " << std::fixed << std::setprecision(6) << location[0]
                                        << " --lon " << std::fixed << std::setprecision(6) << location[1]
                                        << " --start_time \"" << start_time_buffer << "\""
                                        << " --end_time \"" << end_time_buffer << "\""
                                        << " --utc_adjustment " << sign << utc_adjustment
                                        << " --num_timesteps " << num_timesteps;
            if (output_file_buffer != "") {
                oss << " --out_csv " << output_file_buffer;
            }
            std::cout << oss.str() << std::endl;
            
            system(oss.str().c_str());
        } catch (const std::exception& e) {
            is_error_popup_open = true;
            error_popup_message += "dayAzElIrr.py could not be run with exception: " + std::string(e.what());
            return;
        }
    }
}

bool GUI::check_dayAzElIrr_args() {
    bool are_args_valid = true;
    if (num_timesteps < 0) {
        are_args_valid = false;
        error_popup_message += "Must specify at least one timestep\n";
    }

    if (location[0] < -90.0 || location[0] > 90.0) {
        are_args_valid = false;
        error_popup_message += "Latitude is not within range [-90, 90]\n";
    }

    if (location[1] < -180.0 || location[1] > 180.0) {
        are_args_valid = false;
        error_popup_message += "Longitude is not within range [-180, 180]\n";
    }

	std::istringstream rss(start_time_buffer);
    date::sys_time<std::chrono::seconds> epoch_time;
    rss >> date::parse("%F %T", epoch_time);

    if (rss.fail()) {
        are_args_valid = false;
        error_popup_message += "Start time is not in YYYY-MM-DD HH:MM:SS format\n";
    }

    std::istringstream ess(end_time_buffer);
    ess >> date::parse("%F %T", epoch_time);
    
    if (ess.fail()) {
        are_args_valid = false;
        error_popup_message += "End time is not in YYYY-MM-DD HH:MM:SS format\n";
    }

    if (utc_adjustment.empty()) {
        are_args_valid = false;
        error_popup_message += "UTC adjustment not entered\n";
    }
    std::istringstream tss(utc_adjustment);
	int hours, minutes, seconds;
	char delimiter;
	tss >> hours >> delimiter >> minutes >> delimiter >> seconds;
    if (hours < 0 || hours > 23 || minutes < 0 || minutes > 59 || seconds < 0 || seconds > 59) {
        are_args_valid = false;
        error_popup_message += "UTC adjustment has invalid HH:MM:SS format\n";
    }

    is_error_popup_open = !are_args_valid;

    return are_args_valid;
}

void GUI::render_step_two_layout() {    
    ImGui::Begin("Step 2 - Effective Irradiance CSV");
    std::filesystem::path path; // Dummy variable
    insert_file_dialog_button("Array Cell STL Directory", &button_size,
                              "arraySTLDir", "Choose Directory", nullptr,
                              path, cell_stl_folder_path);
    insert_tooltip("Directory that exclusively contains all the STL files for the array cells");

    insert_file_dialog_button("Canopy STL File", &button_size, "canopySTLFile",
                            "Choose File", ".stl", canopy_stl_file_path, path);
    insert_tooltip("STL file of the canopy");

    insert_file_dialog_button("Sun Positions CSV", &button_size, "sunPositionPath",
                              "Choose File", ".csv", sun_positions_path, path);
    insert_tooltip("CSV file describing the path of the sun generated according to step 1.");

    insert_file_dialog_button("Route CSV", &button_size, "routePath", "Choose File",
                              ".csv", route_path, path);
    insert_tooltip("CSV file of the route. This should be used ONLY for dynamic simulations");

    ImGui::RadioButton("Dynamic Simulation", &sim_type, static_cast<int>(CellIrradianceSim::SimType::DYNAMIC));
    ImGui::RadioButton("Static Simulation", &sim_type, static_cast<int>(CellIrradianceSim::SimType::STATIC));

    ImGui::Checkbox("Precise Shadow Calculation", &precise_shadow_calculation);
    insert_tooltip("Whether to perform precise shadow calculations for partially shaded cells.\n"
                    "This is significantly slower if set to true.");

    ImGui::SetNextItemWidth(text_field_width);
    ImGui::InputText("Direction", &direction);
    insert_tooltip("Direction of the nose of the car. Usually \"-x\". Confirm with the CAD of the car");

    ImGui::SetNextItemWidth(text_field_width);
    ImGui::InputFloat("Bearing", &bearing);
    insert_tooltip("The clockwise angle of the nose of the car from true north in degrees\n"
                   "when positioned at the location in the sun path csv. This should ONLY be used\n"
                   "for a static simulation. Example: In WSC, if the sun position csv was generated\n"
                   "at Alice Springs, then this would be 180 since the car is facing south at that\n"
                   "point during the race.");

    ImGui::SetNextItemWidth(text_field_width);
    ImGui::InputFloat("Car Speed", &car_speed);
    insert_tooltip("Speed that the car travels around the Route in kph.\n"
                    "This should only be used for a dynamic simulation.");

    ImGui::SetNextItemWidth(text_field_width);
    ImGui::InputText("Start Route Time", &start_route_time_buffer);
    insert_tooltip("Start time of the simulation in 24 hour local time with format: YYYY-MM-DD HH:MM:SS\n"
                    "This must be within the time range of the sun position csv. This should only be used\n"
                    "for a dynamic simulation.");

    ImGui::SetNextItemWidth(text_field_width);
    ImGui::InputText("End Route Time", &end_route_time_buffer);
    insert_tooltip("End time of the simulation in 24 hour local time with format: YYYY-MM-DD HH:MM:SS\n"
                    "This must be within the time range of the sun position csv. This should only be used\n"
                    "for a dynamic simulation.");

    ImGui::SetNextItemWidth(text_field_width);
    ImGui::InputText("Output Name", &irradiance_csv_name);
    insert_tooltip("Name of the output irradiance csv");
    ImGui::NewLine();

    if (ImGui::Button("Generate Irradiance CSV")) {
        if (!check_irrCsv_args()) return;
        try {
            car_model = std::make_shared<Model>();
            car_model->loadCanopy(canopy_stl_file_path.string());
            car_model->loadArray(cell_stl_folder_path.string());
            std::unique_ptr<SunPositionLUT> sun_position = std::make_unique<SunPositionLUT>(sun_positions_path);

            // Run the simulation
            irradiance_sim = std::make_unique<CellIrradianceSim>(car_model, precise_shadow_calculation);
            if (sim_type == static_cast<int>(CellIrradianceSim::SimType::DYNAMIC)) {
                std::unique_ptr<RouteLUT> route_lut = std::make_unique<RouteLUT>(route_path);
                Time start_route_time(start_route_time_buffer);
                Time end_route_time(end_route_time_buffer);
                std::string metadata_csv_name = irradiance_csv_name.substr(0, irradiance_csv_name.find(".csv")) + "_metadata.csv";
                irradiance_sim->run_dynamic_sim(sun_position, route_lut, static_cast<double>(kph2mps(car_speed)), direction,
                                                start_route_time, end_route_time);
                irradiance_sim->write_dynamic_csv(irradiance_csv_name, metadata_csv_name);
            } else {
                irradiance_sim->run_static_sim(sun_position, static_cast<double>(bearing), direction);
                irradiance_sim->write_static_csv(irradiance_csv_name);
            }
        } catch (const std::exception& e) {
            is_error_popup_open = true;
            error_popup_message = "Irradiance CSV could not be generated. Caught exception " + std::string(e.what());
            return;
        }
    }
}

bool GUI::check_irrCsv_args() {
    bool are_args_valid = true;

    if (cell_stl_folder_path.empty()) {
        are_args_valid = false;
        error_popup_message += "No cell STL folder supplied\n";
    }

    if (canopy_stl_file_path.empty()) {
        are_args_valid = false;
        error_popup_message += "No canopy STL file supplied\n";
    }

    if (direction.empty()) {
        are_args_valid = false;
        error_popup_message += "No nose direction entered\n";
    } else if (direction != "-x" && direction != "+x" &&
               direction != "-y" && direction != "+y") {
        are_args_valid = false;
        error_popup_message += "Nose direction must be one of {-x, +x, -y, +y}\n";
    }

    if (bearing < 0.0 || bearing > 360.0) {
        are_args_valid = false;
        error_popup_message += "Bearing out of range [0, 360]\n";
    }

    if (sun_positions_path.empty()) {
        are_args_valid = false;
        error_popup_message += "No sun position csv (step 1) supplied\n";
    }

    if (irradiance_csv_name.empty()) {
        are_args_valid = false;
        error_popup_message += "No output csv name given\n";
    } else if (irradiance_csv_name.find(".csv") == std::string::npos) {
        are_args_valid = false;
        error_popup_message += "Irradiance CSV must end in .csv";
    }

    if (sim_type == static_cast<int>(CellIrradianceSim::SimType::DYNAMIC)) {
        if (car_speed <= 0.0) {
            are_args_valid = false;
            error_popup_message += "Car speed must be greater than 0 kph for a dynamic simulation\n";
        }
        
        if (route_path.empty()) {
            are_args_valid = false;
            error_popup_message += "No route path was supplied for a dynamic simulation\n";
        }

        std::istringstream rss(start_route_time_buffer);
        date::sys_time<std::chrono::seconds> epoch_time;
        rss >> date::parse("%F %T", epoch_time);

        if (rss.fail()) {
            are_args_valid = false;
            error_popup_message += "Route start time is not in YYYY-MM-DD HH:MM:SS format for a dynamic simulation\n";
        }

        std::istringstream ess(end_route_time_buffer);
        ess >> date::parse("%F %T", epoch_time);
        if (ess.fail()) {
            are_args_valid = false;
            error_popup_message += "Route end time is not in YYYY-MM-DD HH:MM:SS format for a dynamic simulation\n";
        }
    }
    is_error_popup_open = !are_args_valid;

    return are_args_valid;
}

void GUI::render_step_three_layout() {
    ImGui::Begin("Visualizations");

    render_underlined_text("See Canopy and Array");
    std::filesystem::path path;  // Dummy variable
    insert_file_dialog_button("Array Cell STL Directory", &button_size,"arraySTLDir", "Choose Directory",
                             nullptr, path, cell_stl_folder_path_v);
    insert_tooltip("Directory that exclusively contains all the STL files for the array cells");

    insert_file_dialog_button("Canopy STL File", &button_size, "canopySTLFile", "Choose File",
                             ".stl", canopy_stl_file_path_v, path);
    insert_tooltip("STL file of the canopy");

    insert_file_dialog_button("Sun Position CSV", &button_size, "SunCsvFile", "Choose File", ".csv", sun_positions_path_v, path);
    insert_tooltip("CSV file describing the path of the sun (Step 1)");
    
    insert_file_dialog_button("Irradiance CSV", &button_size, "csvFile", "Choose File", ".csv", irradiance_csv_file_path, path);
    insert_tooltip("CSV file describing the effective irradiances on each cell at each sun position (Step 2). This is necessary\n"
                    "only for visualizing shadows");
    
    insert_file_dialog_button("Metadata CSV", &button_size, "metadataFile", "Choose File", ".csv", metadata_csv_file_path, path);
    insert_tooltip("CSV file generated from a dynamic simulation. This is only necessary if you want to visualize shadows\n"
                    "from a dynamic simulation");

    ImGui::SetNextItemWidth(irr_row_width);
    ImGui::InputInt("Sun Position Row", &curr_irr_row);
    insert_tooltip("The row in the irradiance csv to display. For the results of\n"
                    "a static simulation, this is equivalent to the desired sun\n"
                    "position row in the sun positions csv.");

    ImGui::SetNextItemWidth(irr_row_width);
    ImGui::InputInt("Cell Highlight", &curr_cell_idx);
    insert_tooltip("The cell to highlight in red");

    ImGui::NewLine();

    // User chooses to see only the array and canopy
    if (ImGui::Button("See Car", button_size)) {
        if (!check_visualization_args()) {
            return;
        }
        try {
            // Take input from both the mouse and keyboard
            glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

            // Prepare the car model for visualization
            car_model = std::make_shared<Model>();
            car_model->loadCanopy(canopy_stl_file_path_v.string());
            car_model->loadArray(cell_stl_folder_path_v.string());
            car_model->init_geometries();
            car_model->init_camera();
            car_model->init_shaders();

            // Reset visualization variables
            car_visualized = true;
            first_mouse_movement = true;
            mouse_control = true;
            delta_time = 0.0f;
            last_frame = 0.0f;
        } catch (const std::exception& e) {
            is_error_popup_open = true;
            error_popup_message += "Could not visualize the car. Caught error: " + std::string(e.what());
            return;
        }
    }

    // User chooses to see the array, canopy and irradiance heat map
    if (ImGui::Button("See Irradiance Map", button_size)) {
        if (!check_visualization_args()) {
            return;
        }

        if (!check_irradiance_visualization_args()) {
            return;
        }

        try {
            sun_position_lut = std::make_unique<SunPositionLUT>(sun_positions_path_v);
            
            // If a metadata csv file was selected, then show the irradiance heat map from the
            // results of a dynamic simulation
            if (!metadata_csv_file_path.empty()) {
                show_dynamic_params = true;
                irradiance_csv = std::make_shared<CellIrradianceCsv>(irradiance_csv_file_path,
                                                                     metadata_csv_file_path);
            } else {
                show_dynamic_params = false;
                irradiance_csv = std::make_shared<CellIrradianceCsv>(irradiance_csv_file_path);
            }
            
            irradiance_limits = irradiance_csv->get_irradiance_limits();
            num_irr_csv_rows = irradiance_csv->get_num_irr_rows();

            // Pre-compute cell colourings for each row of the irradiance csv
            for (size_t i = 0; i < num_irr_csv_rows; i++) {
                std::vector<glm::vec3> row_colours;
                std::vector<double> irradiance_values = irradiance_csv->get_csv_row(i);
                for (size_t j = 0; j < irradiance_values.size(); j++) {
                    double normalized_value = (irradiance_values[j] - irradiance_limits.first) /
                                              (irradiance_limits.second - irradiance_limits.first);
                    const tinycolormap::Color color = tinycolormap::GetColor(normalized_value, tinycolormap::ColormapType::Jet);
                    row_colours.push_back(glm::vec3(color.r(), color.g(), color.b()));
                }
                cell_colours.push_back(row_colours);
            }
            RUNTIME_ASSERT(cell_colours.size() == num_irr_csv_rows, "Pre-computation for cell-colourings failed");

            // Take input from the keyboard and mouse
            glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

            // Prepare the car model for visualization
            car_model = std::make_shared<Model>();
            car_model->init_shaders();
            car_model->loadCanopy(canopy_stl_file_path_v.string());
            car_model->loadArray(cell_stl_folder_path_v.string());
            car_model->init_geometries();
            car_model->init_camera();
            num_array_cells = car_model->get_array_cell_meshes().size();

            // Reset visualization variables
            car_visualized = true;
            first_mouse_movement = true;
            mouse_control = true;
            irradiance_visualized = true;
            delta_time = 0.0f;
            last_frame = 0.0f;
        } catch (const std::exception& e) {
            is_error_popup_open = true;
            error_popup_message += "Could not visualize irradiance map. Caught exception: " + std::string(e.what());
            return;
        }
    }

    // Visualize the array and canopy
    if (car_visualized) {
        float current_frame = glfwGetTime();
        delta_time = current_frame - last_frame;
        last_frame = current_frame;
        process_input(window);

        // Visualize the irradiance map if the users wishes to see this mode
        std::vector<double> irradiance_values = {};
        std::pair<double, double> irradiance_limits = {};
        std::vector<glm::vec3> colours = {};
        if (irradiance_visualized) {
            // Ensure that a valid irradiance row is displayed. Default to 0
            if (curr_irr_row >= num_irr_csv_rows || curr_irr_row < 0) curr_irr_row = 0;
            irradiance_values = irradiance_csv->get_csv_row(curr_irr_row);
            colours = cell_colours[curr_irr_row];
            // Ensure that a valid highlight is shown. Default to -1 which highlights nothing
            if (-1 > curr_cell_idx || curr_cell_idx >= num_array_cells) curr_cell_idx = -1;
        }
        car_model->Draw(window_width, window_height, colours, curr_cell_idx, true);
    }

    // Display information if the user has requested heat map mode
    if (irradiance_visualized && (0 < curr_irr_row < num_irr_csv_rows)) {
        if (show_dynamic_params) {
            size_t sun_position_idx = irradiance_csv->get_sun_position_cache_value(curr_irr_row);
            time_of_day = "Time Of Day: " + irradiance_csv->get_time_string_value(curr_irr_row);
            azimuth = "Azimuth: " + std::to_string(sun_position_lut->get_azimuth_value(sun_position_idx));
            elevation = "Elevation: " + std::to_string(sun_position_lut->get_elevation_value(sun_position_idx));
            irradiance = "Irradiance: " + std::to_string(sun_position_lut->get_irradiance_value(sun_position_idx));  
            bearing_label = "Car Bearing: " + std::to_string(irradiance_csv->get_bearing_value(curr_irr_row));
            Coord coord = irradiance_csv->get_coord_value(curr_irr_row);
            coordinate_label = "Coordinates of Car: " + std::to_string(coord.lat) + ", " + std::to_string(coord.lon) + ", " + std::to_string(coord.alt);
            ImGui::Text(bearing_label.c_str());
            ImGui::Text(coordinate_label.c_str());
        } else {
            time_of_day = "Time Of Day: " + sun_position_lut->get_time_value(curr_irr_row).get_local_readable_time();
            azimuth = "Azimuth: " + std::to_string(sun_position_lut->get_azimuth_value(curr_irr_row));
            elevation = "Elevation: " + std::to_string(sun_position_lut->get_elevation_value(curr_irr_row));
            irradiance = "Irradiance: " + std::to_string(sun_position_lut->get_irradiance_value(curr_irr_row));
        }
        ImGui::Text(time_of_day.c_str());
        ImGui::Text(azimuth.c_str());
        ImGui::Text(elevation.c_str());
        ImGui::Text(irradiance.c_str());

        if (0 < curr_cell_idx < num_array_cells) {
            cell_irradiance = "Highlighted Cell Irradiance: " + std::to_string(irradiance_csv->get_irr_value(curr_irr_row, curr_cell_idx));
            ImGui::Text(cell_irradiance.c_str());
        }
    }
}

bool GUI::check_visualization_args() {
    bool are_args_valid = true;
    if (cell_stl_folder_path_v.empty()) {
        are_args_valid = false;
        error_popup_message += "No cell STL folder supplied.\n";
    }

    if (canopy_stl_file_path_v.empty()) {
        are_args_valid = false;
        error_popup_message += "No canopy STL file supplied.\n";
    }

    is_error_popup_open = !are_args_valid;
    return are_args_valid;
}

bool GUI::check_irradiance_visualization_args() {
    bool are_args_valid = true;
    
    if (sun_positions_path_v.empty()) {
        are_args_valid = false;
        error_popup_message += "No sun positions csv file supplied.\n";
    }

    if (irradiance_csv_file_path.empty()) {
        are_args_valid = false;
        error_popup_message += "No irradiance csv file supplied.\n";
    }

    is_error_popup_open = !are_args_valid;
    return are_args_valid;
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
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    io = ImGui::GetIO();

    // Window component size and positions
    window_width = io.DisplaySize.x;
    window_height = io.DisplaySize.y;

    // File dialog popup is positioned in the middle of the screen
    file_dialog_size = ImVec2(
        window_width * file_dialog_width_fraction,
        window_height * file_dialog_height_fraction
    );
    file_dialog_position = ImVec2(
        window_width * 0.5f - file_dialog_size[WIDTH_INDEX] * 0.5f,
        window_height * 0.5f - file_dialog_size[HEIGHT_INDEX] * 0.5f
    );

    // Recompute the window sizes upon the application resizing
    window_resized = false;
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
    }

    // Set the backgroud colour
    glClearColor(clear_color.x * clear_color.w, clear_color.y * clear_color.w, clear_color.z * clear_color.w, clear_color.w);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

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

void GUI::cursor_callback(GLFWwindow* window, double xpos, double ypos) {
    GUI* app = static_cast<GUI*>(glfwGetWindowUserPointer(window));
    if (!app) return;

    if (!app->car_visualized || !app->mouse_control) {
        return;
    }

    if (app->first_mouse_movement)
    {
        app->last_x = xpos;
        app->last_y = ypos;
        app->first_mouse_movement = false;
    }

    float xoffset = xpos - app->last_x;
    float yoffset = app->last_y - ypos; 
    app->last_x = xpos;
    app->last_y = ypos;

    app->car_model->camera->ProcessMouseMovement(xoffset, yoffset);
}

void GUI::process_input(GLFWwindow *window)
{
    if (!car_visualized) {
        return;
    }

    // Relinquish orbital control of the mouse
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
        glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
        last_key_pressed = GLFW_KEY_ESCAPE;
        mouse_control = false;
    }
    // Allow orbital control for the mouse
    if (glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS) {
        glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
        last_key_pressed = GLFW_KEY_Q;
        mouse_control = true;
    }

    // When the irradiance heat map is visualized, arrow keys can be used to
    // navigate the row of the irradiance csv to visualize
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
            curr_irr_row = curr_irr_row + irr_mod;
        }
    }

    if (!mouse_control) return;

    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) {
        car_model->camera->ProcessKeyboard(Camera::CameraMovement::FORWARD, delta_time);
        last_key_pressed = GLFW_KEY_W;
    }
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) {
        car_model->camera->ProcessKeyboard(Camera::CameraMovement::BACKWARD, delta_time);
        last_key_pressed = GLFW_KEY_S;
    }
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) {
        car_model->camera->ProcessKeyboard(Camera::CameraMovement::LEFT, delta_time);
        last_key_pressed = GLFW_KEY_A;
    }
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) {
        car_model->camera->ProcessKeyboard(Camera::CameraMovement::RIGHT, delta_time);
        last_key_pressed = GLFW_KEY_D;
    }
}

void GUI::scroll_callback(GLFWwindow* window, double xpos, double ypos) {
    GUI* app = static_cast<GUI*>(glfwGetWindowUserPointer(window));
    if (!app) return;
    if (!app->car_visualized || !app->mouse_control) {
        return;
    }

    app->car_model->camera->ProcessMouseScroll(static_cast<float>(ypos));
}

void GUI::window_iconify_callback(GLFWwindow* window, int iconified)
{
    GUI* app = static_cast<GUI*>(glfwGetWindowUserPointer(window));
    if (!app) return;
    if (iconified) {
        app->is_minimized = false;
    }
    else {
        app->is_minimized = true;
    }
}
