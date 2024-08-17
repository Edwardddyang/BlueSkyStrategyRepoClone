// GUI rendering class

#pragma once

#include <iostream>
#include <stdlib.h>
#include <memory>
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include <stdio.h>
#include "model.hpp"
#include "Shader.hpp"
#include "camera.hpp"
#include <filesystem>
#include "Luts.hpp"
#include "Utilities.hpp"
#define GL_SILENCE_DEPRECATION
#if defined(IMGUI_IMPL_OPENGL_ES2)
#include <GLES2/gl2.h>
#endif
#include "glad/glad.h"
#include "irradiance_csv.hpp"
#include "sun_plane.hpp"
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
    const ImVec4 clear_color = ImVec4(0.0f, 0.0f, 0.0f, 1.00f);

    // Application name that appears in the toolbar
    const char* APP_NAME = "Gen 12 Array Simulation";

    // Window variables
    float window_width = 1280;
    float window_height = 720;
    float prev_window_width;  // From last frame
    float prev_window_height;  // From last frame

    // Keyboard and mouse I/O
    ImGuiIO io;

    // Array vertex data structures
    unsigned int VBO, VAO;

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

    // File dialog
    const float file_dialog_width_fraction = 0.5;
    const float file_dialog_height_fraction = 0.5;
    ImVec2 file_dialog_size;
    ImVec2 file_dialog_position;

    // Step 1
    const float num_timesteps_width_fraction = 0.4;
    const float location_width_fraction = 0.4;
    const float time_field_width_fraction = 0.4;
    const float text_field_width_fraction = 0.4;
    float num_timesteps_width;
    float location_width;
    float time_field_width;
    float text_field_width;

    // Step 2
    const float button_width_fraction = 0.8;
    const float button_height = 0.0; // auto
    ImVec2 button_size;

    // Visualizations
    std::shared_ptr<Model> car_model;
    glm::vec3 sun_position = glm::vec3(1.2f, 1.0f, 2.0f);
    glm::vec3 sun_diffuse = glm::vec3(0.5f, 0.5f, 0.5f); // Gray-ish

    /* ----- Internal State ----- */
    bool window_resized = true;
    // Help popup
    bool is_help_popup_open = false;
    // Step selection
    int selected_step = static_cast<int>(steps::STEP_1);
    int last_selected_step;
    // Step 1
    int num_timesteps = 180;
    float location[2] = {0.0f, 0.0f};
    std::string start_time;
    std::string end_time;
    std::string utc_adjustment;
    int adjustment = FORWARD_ADJUSTMENT;
    std::string output_file_buffer;
    // Step 2
    std::filesystem::path cell_stl_folder_path;
    std::filesystem::path canopy_stl_file_path;
    std::filesystem::path sun_positions_path;
    std::shared_ptr<SunPositionLUT> sun_position_lut;
    std::string direction;
    std::string bearing_text;
    double bearing = 0;
    std::shared_ptr<IrradianceCSV> irradiance_csv;
    // Visualization
    std::filesystem::path cell_stl_folder_path_v;
    std::filesystem::path canopy_stl_file_path_v;
    bool car_visualized = false;
    float delta_time = 0.0f;	// Time between current frame and last frame
    float last_frame = 0.0f; // Time of last frame
    double last_x;
    double last_y;
    bool first_mouse_movement;
    bool is_left_click_held;

    // Frame render functions
    void render_step_selection_pane();
    void render_simulation_configuration_pane();
    void initialize_new_frame();
    void render_frame();
    void render_step_one_layout();
    void render_step_two_layout();
    void render_step_three_layout();

    // Mouse and keyboard capture functions
    void cursor_callback(double xpos, double ypos);
    static void cursor_callback_bridge(GLFWwindow* window, double xpos, double ypos);

    void scroll_callback(double xoffset, double yoffset);
    static void scroll_callback_bridge(GLFWwindow* window, double xoffset, double yoffset);

    void mouse_button_callback(int button, int action, int mods);
    static void mouse_button_callback_bridge(GLFWwindow* window, int button, int action, int mods);

    void process_input(GLFWwindow* window);
    // Utility functions

    // Set the size and position of the next ImGui::Begin() window
    void set_window_size_position(const ImVec2 size, const ImVec2 position) const;

    // Set the size and position of the next ImGui::Begin() window if the application
    // window resized on the last frame
    void set_window_size_position_on_resize(const ImVec2 size, const ImVec2 position) const;

    // Inserts a tooltip with icon (?) on the same line as the previous element
    void insert_tooltip(const char* tip) const;

    /** Add a dialog file button
     * @param button_name: The name that appears on the button
     * @param key: A unique string indicating the file dialog
     * @param window_title: The name that appears as the title of the file dialog window
     * @param filter: The filter for types of files to select e.g. nullptr, .stl, .cpp
     * @param file_path: The selected file
     * @param folder_path: Parent folder of the selected file
     */
    void insert_file_dialog_button(const char* button_name, 
                                    ImVec2* button_size,
                                    const std::string key, 
                                    const std::string window_title,
                                    const char* filter,
                                    std::filesystem::path& file_path,
                                    std::filesystem::path& folder_path);

public:
    static std::shared_ptr<GUI> get_instance();

    // Initialize backends and rendering window
    void initialize();

    // Primary render loop
    void render();

    // Cleanup tasks
    void cleanup();
};