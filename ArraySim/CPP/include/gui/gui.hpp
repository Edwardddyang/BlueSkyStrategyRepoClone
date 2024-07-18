// GUI rendering class

#pragma once

#include <iostream>
#include <stdlib.h>
#include <memory>
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include <stdio.h>
#define GL_SILENCE_DEPRECATION
#if defined(IMGUI_IMPL_OPENGL_ES2)
#include <GLES2/gl2.h>
#endif
#include <GLFW/glfw3.h> // Will drag system OpenGL headers
// [Win32] Our example includes a copy of glfw3.lib pre-compiled with VS2010 to maximize ease of testing and compatibility with old VS compilers.
// To link with VS2010-era libraries, VS2015+ requires linking with legacy_stdio_definitions.lib, which we do using this pragma.
// Your own project should not be affected, as you are likely to link with a newer binary of GLFW that is adequate for your version of Visual Studio.
#if defined(_MSC_VER) && (_MSC_VER >= 1900) && !defined(IMGUI_DISABLE_WIN32_FUNCTIONS)
#pragma comment(lib, "legacy_stdio_definitions")
#endif

// Indexing into ImVec2 position/sizes
#define HEIGHT_INDEX 1
#define WIDTH_INDEX 0

#define FORWARD_ADJUSTMENT 0
#define BACKWARD_ADJUSTMENT 1

enum class steps {
    STEP_1,
    STEP_2,
    STEP_3,
};

class GUI {
private:
    GUI() {};
    static void glfw_error_callback(int error, const char* description);
    static std::shared_ptr<GUI> instance;
    static GLFWwindow* window;

    // Background colour
    const ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);

    // Application name that appears in the toolbar
    const char* app_name = "Gen 12 Array Simulation";

    // Window variables
    float window_width;
    float window_height;
    float prev_window_width;  // From last frame
    float prev_window_height;  // From last frame

    /* ----- Window Component Size + Locations ----- */
    // All window width and height fractions are relative to the entire window size

    // Step selection
    const float step_selection_width_fraction = 0.25;
    const float step_selection_height_fraction = 0.2;
    const ImVec2 step_selection_position = ImVec2(0.0f, 0.0f);
    ImVec2 step_selection_size;

    // Help popup
    const float help_popup_width_fraction = 0.5;
    const float help_popup_height_fraction = 0.3;
    ImVec2 help_popup_position;
    ImVec2 help_popup_size;

    // Simulation configuration
    const float sim_config_width_fraction = 0.25;
    const float sim_config_height_fraction = 0.0; // Let ImGUI auto-adjust
    ImVec2 sim_config_position;
    ImVec2 sim_config_size;

    // Step 1
    const float num_timesteps_width_fraction = 0.4;
    const float location_width_fraction = 0.4;
    const float time_field_width_fraction = 0.4;
    const float text_field_width_fraction = 0.4;
    float num_timesteps_width;
    float location_width;
    float time_field_width;
    float text_field_width;

    /* ----- Internal State ----- */
    bool window_resized = true;
    // Help popup
    bool is_help_popup_open = false;
    // Step selection
    int selected_step = 0;
    int last_selected_step;
    // Step 1
    int num_timesteps = 180;
    float location[2] = {0.0f, 0.0f};
    std::string start_time;
    std::string end_time;
    std::string utc_adjustment;
    int adjustment = FORWARD_ADJUSTMENT;
    std::string output_file_buffer;

    void render_step_selection_pane();
    void render_simulation_configuration_pane();
    void initialize_new_frame();
    void render_frame();
    void render_step_one_layout();

    // Utility functions

    // Set the size and position of the next ImGui::Begin() window
    void set_window_size_position(const ImVec2 size, const ImVec2 position) const;

    // Set the size and position of the next ImGui::Begin() window if the application
    // window resized on the last frame
    void set_window_size_position_on_resize(const ImVec2 size, const ImVec2 position) const;

    // Inserts a tooltip with icon (?) on the same line as the previous element
    void insert_tooltip(const char* tip) const;

public:
    static std::shared_ptr<GUI> get_instance();

    // Initialize backends and rendering window
    void initialize();

    // Primary render loop
    void render();

    // Cleanup tasks
    void cleanup();
};