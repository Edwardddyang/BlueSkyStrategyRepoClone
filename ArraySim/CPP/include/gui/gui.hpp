// GUI rendering class

#pragma once

#include <iostream>
#include <stdlib.h>
#include <memory>
#include "time.hpp"
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
#include "CellIrradianceSim.hpp"
#include "sun_plane.hpp"
#include "tinycolormap.hpp"
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

/** Top level application class */
class GUI {
private:
    GUI() {};

    /** @brief Display GLFW error to stdout */
    static void glfw_error_callback(int error, const char* description);

    // Singleton object
    static std::unique_ptr<GUI> instance;

    // Application window - should be alive until the GUI object is destroyed
    static GLFWwindow* window;

    // Background colour (black)
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

    /* ----- Window Components ----- */
    // All window width and height fractions are relative to the entire window size

    // Step selection
    enum class steps {
        STEP_1,
        STEP_2,
        STEP_3,
    };
    const float step_selection_width_fraction = 0.25;
    const float step_selection_height_fraction = 0.2;
    const ImVec2 step_selection_position = ImVec2(0.0f, 0.0f);
    ImVec2 step_selection_size;
    int selected_step = static_cast<int>(steps::STEP_1);
    int last_selected_step;

    // Help popup
    const float help_popup_width_fraction = 0.5;
    const float help_popup_height_fraction = 0.3;
    ImVec2 help_popup_position;
    ImVec2 help_popup_size;
    bool is_help_popup_open = false;
    bool first_frame_help_render = false;

    // Error popup
    bool is_error_popup_open = false;
    bool first_frame_error_render = false;
    std::string error_popup_message;
    const float error_popup_width_fraction = 0.4;
    const float error_popup_height_fraction = 0.2;
    ImVec2 error_popup_position;
    ImVec2 error_popup_size;

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

    /** @brief Check input arguments to dayAzElIrr.py */
    bool check_dayAzElIrr_args();

    int num_timesteps = 180;
    float location[2] = {0.0f, 0.0f};
    int utc_adjustment_direction = FORWARD_ADJUSTMENT;
    std::string start_time_buffer;
    std::string end_time_buffer;
    std::string utc_adjustment;
    std::string output_file_buffer;

    // Step 2
    const float button_width_fraction = 0.8;
    const float button_height = 0.0; // Let ImGui auto-adjust
    ImVec2 button_size;

    std::unique_ptr<SunPositionLUT> sun_position_lut = nullptr;
    std::unique_ptr<Time> start_time = nullptr;
    std::unique_ptr<Time> end_time = nullptr;
    std::unique_ptr<CellIrradianceSim> irradiance_sim = nullptr;
    std::filesystem::path cell_stl_folder_path;
    std::filesystem::path canopy_stl_file_path;
    std::filesystem::path sun_positions_path;
    std::filesystem::path route_path;
    std::string start_route_time_buffer;
    std::string end_route_time_buffer;
    std::string direction;
    std::string bearing_buffer;
    std::string irradiance_csv_name;
    float bearing = 0.0;
    float car_speed = 0.0;
    bool precise_shadow_calculation = false;
    int sim_type = static_cast<int>(CellIrradianceSim::SimType::STATIC);

    /** @brief Check input arguments for generating the irradiance csv */
    bool check_irrCsv_args();

    // Visualizations
    const float irr_row_width_fraction = 0.3;
    float irr_row_width;

    std::shared_ptr<Model> car_model = nullptr;
    std::shared_ptr<CellIrradianceCsv> irradiance_csv = nullptr;
    std::vector<std::vector<glm::vec3>> cell_colours; // Colour for each cell mesh on a colour gradient
    std::pair<double, double> irradiance_limits;  // Irradiance min, max limits in the irradiance csv
    size_t num_irr_csv_rows;
    size_t num_array_cells;
    int curr_irr_row = 0;  // The row of the irradiance csv to visualize
    int curr_cell_idx = -1;  // The cell to be highlighted
    std::string time_of_day;
    std::string azimuth;
    std::string bearing_label;
    std::string coordinate_label;
    std::string elevation;
    std::string irradiance;
    std::string cell_irradiance;
    std::filesystem::path cell_stl_folder_path_v;
    std::filesystem::path canopy_stl_file_path_v;
    std::filesystem::path sun_positions_path_v;
    std::filesystem::path irradiance_csv_file_path;
    std::filesystem::path metadata_csv_file_path;

    bool show_dynamic_params = false;
    bool car_visualized = false;  // If the car is visualized at all
    bool irradiance_visualized = false;  // If the irradiance CSV is being visualized as well

    // The following variables are used to manage the <-, -> key presses when navigating between the
    // irradiances rows for different sun positions during shadow visualization
    int last_key_pressed = GLFW_KEY_UNKNOWN;  // Remember key press of the last frame
    float key_press_duration;  // Time from first frame of the current key press. This is used to provide a 
                                // smoother experience when navigating between irradiance rows. Otherwise,
                                // since frame loading is very fast, a "single" arrow key press could result
                                // in many increments or decrements of curr_irr_row
    const float key_press_duration_threshold = 0.2;
    float key_press_start;  // The starting timestamp for when the current key press

    // The following variables are used to manage the camera movement during visualization
    float delta_time = 0.0f;  // Time between current frame and last frame
    float last_frame = 0.0f;  // Time of last frame
    double last_x;  // x position of the mouse in the last frame
    double last_y;  // y position of the mouse in the last frame
    bool first_mouse_movement;  // Set to true upon visualization to set last_x and last_y for the first time
    bool mouse_control = false;  // If the mouse is controlling the visualization camera

    /** @brief Check input arguments for visualizing the car */
    bool check_visualization_args();

    /** @brief Check input argument for visualization the irradiance map on the car */
    bool check_irradiance_visualization_args();

    /* ----- App Internal State ----- */
    bool window_resized = true;
    bool is_minimized = false;  // If the app is minimized

    /* ----- Frame render functions ----- */

    /** @brief Render the pane to select the simulation step */
    void render_step_selection_pane();
    /** @brief Render the simulation configuration window depending on the selected step */
    void render_simulation_configuration_pane();
    /** @brief Render error message popup */
    void render_error_message_popup();
    /** @brief Setup the new frame. Calculate pane dimensions and sizes */
    void initialize_new_frame();
    /** @brief Calls ImGui render functions */
    void render_frame();
    /** @brief Render the step one simulation configuration pane */
    void render_step_one_layout();
    /** @brief Render the step two simulation configuration pane */
    void render_step_two_layout();
    /** @brief Render the visualization configuration pane */
    void render_step_three_layout();

    /* ----- Mouse/Keyboard capture functions ----- */

    /** @brief Handle cursor movement during visualization
     * @param xpos: Current x position of the mouse in pixels
     * @param ypos: Current y position of the mouse in pixels
     */
    static void cursor_callback(GLFWwindow* window, double xpos, double ypos);

    /** @brief Handle scroll movement during visualization
     * @param xoffset: Current x (horizontal) scroll
     * @param yoffset: Current y (vertical) scroll
     */
    static void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);

    /** @brief Handle keyboard press during visualization.
     * 
     * Note: Esc to stop camera movement, q to enter camera movement, <- to decrement curr_irr_row
     * -> to increment curr_irr_row
     * 
     * @param window: application window pointer
     */
    void process_input(GLFWwindow* window);

    /** @brief Handle window minimization
     * @param window: Application window pointer
     * @param iconified: 1 if the window is minimized and 0 if not
     */
    static void window_iconify_callback(GLFWwindow* window, int iconified);

    /* --------- Utility render functions ----------------*/

    /** @brief Set the size and position of the next window created by ImGui::Begin()
     * 
     * @param size: The [width, height] of the window in pixels
     * @param position: The [x,y] pixel location of the center of the window
     */
    void set_window_size_position(const ImVec2 size, const ImVec2 position) const;

    /** @brief Set the size and position of the next window created by ImGui::Begin() if the window
     * has been resized since the last frame render
     * 
     * @param size: The [width, height] of the window in pixels
     * @param position: The [x,y] pixel location of the center of the window
     */
    void set_window_size_position_on_resize(const ImVec2 size, const ImVec2 position) const;

    /** @brief Insert a tooltip with icon (?) on the same line as the previous element
     * 
     * @param tip: The message displayed by the tooltip when hovered
    */
    void insert_tooltip(const char* tip) const;

    /** @brief Render underlined text
     * 
     * @param text: The text to underline
     */
    void render_underlined_text(const char* text);

    /** @brief Add a dialog file button
     * 
     * @param button_name: The name that appears on the button
     * @param button_size: Button width/height
     * @param key: A unique string to identify this file dialog
     * @param window_title: The name that appears as the title of the file dialog window
     * @param filter: The filter for types of files to select e.g. nullptr, .stl, .cpp
     * @param file_path: Reference variable to store the selected file's absolute path
     * @param folder_path: Reference variable to store the selected file's absolute directory path
     */
    void insert_file_dialog_button(const char* button_name, 
                                    ImVec2* button_size,
                                    const std::string key, 
                                    const std::string window_title,
                                    const char* filter,
                                    std::filesystem::path& file_path,
                                    std::filesystem::path& folder_path);

public:
    /** @brief Retrieve the singleton instance of the application */
    static std::unique_ptr<GUI>& get_instance();

    /** @brief Initialize the application */
    void initialize();

    /** @brief Primary render loop */
    void render();

    /** @brief Cleanup application resources */
    void cleanup();
};
