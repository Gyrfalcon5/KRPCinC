#include <krpc.hpp>
#include <krpc/services/space_center.hpp>
#include <krpc/services/krpc.hpp>
#include <iostream>
#include <chrono>
#include <thread>
#include <vector>
#include <math.h>
#include <tuple>

float vis_viva(float radius, float semi_major_axis, krpc::services::SpaceCenter::Vessel vessel)
{
    float mu = vessel.orbit().body().gravitational_parameter();
    return sqrt(mu*((2.0/radius)-(1.0/semi_major_axis)));
}

void apoapsis_circularize_node(krpc::services::SpaceCenter space_center)
{
    auto vessel = space_center.active_vessel();
    auto ut = space_center.ut_stream();

    float r = vessel.orbit().apoapsis();
    float a1 = vessel.orbit().semi_major_axis();
    float a2 = r;

    float v1 = vis_viva(r, a1, vessel);
    float v2 = vis_viva(r, a2, vessel);
    float delta_v = v2 - v1;
    vessel.control().add_node(ut() + vessel.orbit().time_to_apoapsis(), delta_v, 0.0, 0.0);
}

void execute_node(krpc::services::SpaceCenter space_center)
{
    auto vessel = space_center.active_vessel();
    auto ut = space_center.ut_stream();
    auto node = vessel.control().nodes()[0];
    float delta_v = node.delta_v();
    auto remaining_burn = node.remaining_delta_v_stream();
    
    // Calculate burn time using rocket equation
    float F = vessel.available_thrust();
    float Isp = vessel.specific_impulse() * 9.82; // 9.82 m/sec^2
    float m0 = vessel.mass();
    float m1 = m0 / exp(delta_v / Isp);
    float flow_rate = F / Isp;
    float burn_time = (m0 - m1) / flow_rate;

    // Orient ship for burn
    std::cout << "Orienting ship for burn" << std::endl;
    std::cout << "The burn will take " << burn_time << " seconds" << std::endl;
    vessel.auto_pilot().set_reference_frame(node.reference_frame());
    vessel.auto_pilot().set_target_direction(std::make_tuple(0.0, 1.0, 0.0));
    vessel.auto_pilot().disengage();
    vessel.auto_pilot().engage();
    vessel.auto_pilot().wait();

    std::cout << "Ready to burn" << std::endl;

    while ((node.time_to() - (burn_time / 2.0)) > 0)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        continue;
    }

    std::cout << "Burning" << std::endl;
    vessel.control().set_throttle(1.0);
    auto burn_start = ut();

    // Physics accurate burn waiting
    while (ut() < (burn_start + burn_time))
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        continue;
    }

    vessel.control().set_throttle(0.0);
    node.remove();
}

void deploy_fairings(float altitude)
{
    auto conn = krpc::connect("Fairing Deployment", "Endeavor-Mint.local");
    krpc::services::KRPC krpc(&conn);
    krpc::services::SpaceCenter space_center(&conn);
    auto vessel = space_center.active_vessel();
    auto flight = vessel.flight();

    auto mean_altitude = flight.mean_altitude_call();

    typedef krpc::services::KRPC::Expression Expr;
    auto expr = Expr::greater_than(conn, Expr::call(conn, mean_altitude), Expr::constant_double(conn, altitude));

    auto event = krpc.add_event(expr);
    event.acquire();
    event.wait();
    std::cout << "Deploying fairings" << std::endl;
    auto fairings = vessel.parts().fairings();
    for (auto fairing : fairings)
        fairing.jettison();
    event.release();
} 




