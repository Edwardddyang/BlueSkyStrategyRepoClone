#include <gtest/gtest.h>
#include "v1_car.h"
#include "units.h"
#include <base_car.h>

TEST(v1CarTest,  ElectricLossTest) {
    CONFIG_FILE_PATH = "../data/config/wsc_config.yaml";

    V1_Car TestCar = V1_Car();
    
    
    double energy_loss = TestCar.compute_electric_loss(300);
    double true_energy_loss = 0.0016667;
    EXPECT_NEAR(energy_loss, true_energy_loss, 0.0000001);

}