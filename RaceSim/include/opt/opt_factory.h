/* Generate optimizers */

#ifndef OPT_FACTORY_H
#define OPT_FACTORY_H

#include <base_opt.h>
#include <route.h>
#include <sim.h>

enum class optimizer 
{
    CONSTANT = 0,
};

class Opt_Factory {
private:
    static std::unordered_map<std::string, optimizer> config_to_optimizer;
    static std::string DEFAULT_OPTIMIZER;
public:
    static Optimizer* get_optimizer(std::string opt_type, Route route, Sim simulator);
};

#endif