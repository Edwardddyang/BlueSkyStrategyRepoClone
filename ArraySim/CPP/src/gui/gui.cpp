#include "gui.hpp"

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
    while (!glfwWindowShouldClose(window))
    {
        initialize_new_frame();

        /* Step selection window pane */
        render_step_selection_pane();

        /* Simulation configuration pane */
        render_simulation_configuration_pane();

        // Rendering
        ImGui::End();
        ImGui::Render();
        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClearColor(clear_color.x * clear_color.w, clear_color.y * clear_color.w, clear_color.z * clear_color.w, clear_color.w);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(window);
    }
}

void GUI::render_step_selection_pane() {
    ImGui::SetNextWindowPos(top_left_position);
    ImGui::SetNextWindowSize(ImVec2(
                            window_width*step_selection_width_fraction, window_height*step_selection_height_fraction
                            ), ImGuiCond_Always);

    ImGui::Begin("Steps");

    ImGui::RadioButton("Step 1", &selected_step, 0);
    ImGui::SameLine();
    ImGui::TextDisabled("(?)");
    if (ImGui::IsItemHovered())
    {
        ImGui::BeginTooltip();
        ImGui::Text("Create a timestamp CSV");
        ImGui::EndTooltip();
    }
    ImGui::RadioButton("Step 2", &selected_step, 1);
    ImGui::SameLine();
    ImGui::TextDisabled("(?)");
    if (ImGui::IsItemHovered())
    {
        ImGui::BeginTooltip();
        ImGui::Text("Create effective irradiance csv");
        ImGui::EndTooltip();
    }
    ImGui::RadioButton("Visualization", &selected_step, 2);
    ImGui::SameLine();
    ImGui::TextDisabled("(?)");
    if (ImGui::IsItemHovered())
    {
        ImGui::BeginTooltip();
        ImGui::Text("Visualizations of the sun path csv (step 1) or the array heat map (step 2)");
        ImGui::EndTooltip();
    }

    // Help popup
    if (ImGui::Button("Help")) {
        ImGui::OpenPopup("Guide");
        is_help_popup_open = true;
    }

    if (is_help_popup_open) {
        float x_window_size = window_width * help_popup_width_fraction;
        float y_window_size = window_height * help_popup_height_fraction;
        ImGui::SetNextWindowSize(ImVec2(x_window_size,y_window_size), ImGuiCond_Always);
        ImVec2 center_position = ImVec2(
            window_width * 0.5f - x_window_size * 0.5f,
            window_height * 0.5f - y_window_size * 0.5f
        );
        ImGui::SetNextWindowPos(center_position, ImGuiCond_Always);
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
    if (selected_step == static_cast<int>(steps::STEP_1)) {
        ImGui::Begin("Step 1 - Generate Simulation CSV");
        ImGui::End();
    }
}

void GUI::initialize_new_frame() {
    glfwPollEvents();
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
    ImGuiIO& io = ImGui::GetIO();
    window_width = io.DisplaySize.x;
    window_height = io.DisplaySize.y;
}
void GUI::cleanup() {
    // Cleanup
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();
}