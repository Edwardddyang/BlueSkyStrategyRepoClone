#include "gui.hpp"
#include <cstdlib>
#include "imgui_stdlib.h"
#include <sstream>
#include <iomanip>  // for std::setprecision
#include "ImGuiFileDialog.h"

/* Static declarations */
std::shared_ptr<GUI> GUI::instance = nullptr;
GLFWwindow* GUI::window = nullptr;

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

// NOTE: This code was almost entirely copied from ImGUI's opengl3 example
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
    window = glfwCreateWindow(1280, 720, app_name, nullptr, nullptr);
    if (window == nullptr) {
        std::cout << "Could not initialize glfw window" << std::endl;
        return;
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); // Enable vsync

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
}

void GUI::render() {
    // Main render loop
    while (!glfwWindowShouldClose(window))
    {
        initialize_new_frame();

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

    if (ImGui::BeginPopupModal("Guide", NULL, ImGuiWindowFlags_AlwaysAutoResize))
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
        ImGui::Begin("Visualize");
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
    std::string path; // Unused
    insert_file_dialog_button("Array Cell STL Directory", &button_size,"arraySTLDir", "Choose Directory", nullptr, path, cell_stl_folder_path);
    insert_file_dialog_button("Canopy STL File", &button_size, "canopySTLFile", "Choose File", ".stl", canopy_stl_file_path, path);

    // ImGui::SetNextItemWidth(text_field_width);
    // ImGui::InputText("Direction", &direction);
    // insert_tooltip("Direction of the nose of the car. Usually \"-x\". Confirm with the CAD of the car");

    // If we have both a canopy STL and the array cell stl folder, render them
    if (ImGui::Button("Visualize")) {
        
    }
}

void GUI::insert_file_dialog_button(const char* button_name, 
                                   ImVec2* button_size,
                                   const std::string key, 
                                   const std::string window_title,
                                   const char* filter,
                                   std::string& file_path,
                                   std::string& folder_path) {
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
            file_path = ImGuiFileDialog::Instance()->GetCurrentFileName();
        }
        ImGuiFileDialog::Instance()->Close();
    }
}

void GUI::initialize_new_frame() {
    glfwPollEvents();
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
    ImGuiIO& io = ImGui::GetIO();

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
    glClearColor(clear_color.x * clear_color.w, clear_color.y * clear_color.w, clear_color.z * clear_color.w, clear_color.w);
    glClear(GL_COLOR_BUFFER_BIT);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    glfwSwapBuffers(window);
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