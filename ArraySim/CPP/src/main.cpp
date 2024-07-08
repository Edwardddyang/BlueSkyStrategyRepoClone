#include "gui.hpp"

int main(int, char**)
{
    std::shared_ptr<GUI> renderer = GUI::get_instance();

    renderer->initialize();

    renderer->render();

    renderer->cleanup();

    return 0;
}
