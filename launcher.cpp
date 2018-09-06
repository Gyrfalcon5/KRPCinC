#include <iostream>
#include <chrono>
#include <thread>
#include <krpc.hpp>
#include <krpc/services/space_center.hpp>
#include <vector>
#include "launcherFunctions.hpp"

int main()
{
    krpc::Client conn = krpc::connect("Launch into orbit", "Endeavor-Mint.local", 50000, 50001);
    krpc::services::SpaceCenter space_center(&conn);
    auto vessel = space_center.active_vessel();
    
    float target_altitude = 80000;
    float fairing_deploy_alt = 40000;

    // Set up telemetry streams
    auto ut = space_center.ut_stream();
    auto altitude = vessel.flight().mean_altitude_stream();
    auto apoapsis = vessel.orbit().apoapsis_altitude_stream();
    auto engines = vessel.parts().engines();
    auto apoTimeStream = vessel.orbit().time_to_apoapsis_stream();

    // Pre-launch setup
    vessel.control().set_sas(false);
    vessel.control().set_rcs(false);
    vessel.control().set_throttle(1);

    // Countdown
    std::cout << "3..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::cout << "2..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::cout << "1..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::cout << "Launch!" << std::endl;

    // Activate first stage
    vessel.control().activate_next_stage();
    vessel.auto_pilot().engage();
    vessel.auto_pilot().target_pitch_and_heading(90, 90); 

    // Prep staging algorithm
    std::vector<krpc::services::SpaceCenter::Engine> active_engines;
    for(auto engine : engines)
    {
        if (engine.active())
            active_engines.push_back(engine);
    }

    // Main ascent loop
    double turn_angle = 0;
    double time_set_point = 55; // seconds
    double errorIntegral = 0.0;
    double pGain = -0.05;
    double iGain = 0;
    double dGain = -0.5;
    double prevError = 0;
    auto start = std::chrono::steady_clock::now();

    std::thread (deploy_fairings, fairing_deploy_alt).detach();

    while (apoapsis() < (target_altitude * 0.9)) 
    {
        double error = time_set_point - apoTimeStream();
        auto end = std::chrono::steady_clock::now();
        double sampleResolution = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() / 1000.0;
        errorIntegral += error * sampleResolution;
        double errorDerivative = (error - prevError) / sampleResolution;
        start = end;
        prevError = error;

        turn_angle += pGain * error + iGain * errorIntegral + dGain * errorDerivative;
        turn_angle = std::max(std::min(turn_angle, 90.0), 0.0);

        vessel.auto_pilot().target_pitch_and_heading(90.0 - turn_angle, 90.0);

        std::cout << "Turn Angle: " << turn_angle << std::endl;

        for (auto engine : active_engines)
        {
            if (!engine.has_fuel())
            {
                vessel.control().activate_next_stage();
                active_engines.clear();
                auto engines = vessel.parts().engines();
                for (auto engine : engines)
                {
                    if(engine.active())
                        active_engines.push_back(engine);
                }
                break;
            }
        }
        if(vessel.thrust() < 1.0)
        {
            vessel.control().activate_next_stage();
            active_engines.clear();
            auto engines = vessel.parts().engines();
            for (auto engine : engines)
            {
                if(engine.active())
                    active_engines.push_back(engine);
            }
        }

        
        // If you don't have this it gets confused for some reason
        std::this_thread::sleep_for(std::chrono::milliseconds(10));



 
    }
    
    vessel.control().set_throttle(0);

    pGain = 0.01;
    double throttle = 0.0;
    while (altitude() < 70000.0)
    {
        throttle = (target_altitude - apoapsis()) * pGain;
        vessel.control().set_throttle(throttle);

        std::cout << "Maintenance Throttle: " << throttle << std::endl;

        // Wait to prevent confusion
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    vessel.control().set_throttle(0);

    apoapsis_circularize_node(space_center);
    execute_node(space_center);
}   
