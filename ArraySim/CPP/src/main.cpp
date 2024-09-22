#include "gui.hpp"

int main(int, char**)
{
    const char* array_root = std::getenv("ARRAY_ROOT");
    if (array_root == nullptr) {
        std::cout << "No ARRAY_ROOT environment variable detected. Set it to the full path to gen12_strategy/ArraySim. Exiting." << std::endl;
        return 0;    
    }
    std::unique_ptr<GUI>& renderer = GUI::get_instance();

    renderer->initialize();

    renderer->render();

    renderer->cleanup();

    return 0;
}
