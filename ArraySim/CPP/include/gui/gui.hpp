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

    // Window locations
    const ImVec2 top_left_position = ImVec2(0.0f, 0.0f);

    // Window variables
    float window_width;
    float window_height;

    void initialize_new_frame();

    /* ------ Window Panes ------ */
    // All width and height fractions are relative to the entire window size

    // Step selection
    float step_selection_width_fraction = 0.15;
    float step_selection_height_fraction = 0.2;

    float help_popup_width_fraction = 0.5;
    float help_popup_height_fraction = 0.3;

    /* ----- Internal State ----- */
    bool is_help_popup_open = false;
    int selected_step = 0;

    void render_step_selection_pane();
    void render_simulation_configuration_pane();

public:
    static std::shared_ptr<GUI> get_instance();

    // Initialize backends and rendering window
    void initialize();

    // Primary render loop
    void render();

    // Cleanup tasks
    void cleanup();
};