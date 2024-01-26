#include <tuple>
#include <string>
#include <sstream>
#include <algorithm>

#include "arwain/arwain.hpp"
#include "arwain/serial.hpp"
#include "arwain/logger.hpp"
#include "arwain/exceptions.hpp"
#include "arwain/transmit_lora.hpp"
#include "arwain/uwb_reader.hpp"
#include "arwain/service_manager.hpp"

#include "uubla/utils.hpp"

UublaWrapper::UublaWrapper()
{
    ServiceManager::register_service(this, service_name);
    init();
}

UublaWrapper::~UublaWrapper()
{
    ServiceManager::unregister_service(service_name);
}

void UublaWrapper::core_setup()
{
    uubla = std::make_unique<UUBLA::Network>();
}

void UublaWrapper::run_idle()
{
    sleep_ms(10);
}

void UublaWrapper::run()
{
    // TODO Currently configured to act as an UUBLA master; need to generalize.
    if (!arwain::config.use_uwb_positioning || arwain::config.node_id != 2)
    {
        return;
    }

    while (mode != arwain::OperatingMode::Terminate)
    {
        switch (mode)
        {
            case arwain::OperatingMode::Inference:
                run_inference();
                break;
            default:
                run_idle();
                break;
        }
    }
}

void UublaWrapper::setup_inference()
{
    uubla->force_plane(true);
    uubla->set_ewma_gain(0.1);
    // uubla->add_node_callback = inform_new_uubla_node; // Replaced with event registrations.
    // uubla->remove_node_callback = inform_remove_uubla_node; // Replaced with event registrations.
    serial_reader_th = std::jthread{serial_reader_fn, uubla.get(), "port", 115200};
}

bool UublaWrapper::cleanup_inference()
{
    serial_reader_th.request_stop();
    return true;
}

void UublaWrapper::run_inference()
{
    setup_inference();
    
    // TODO: Get the framerate (30) from somewhere sensible, or hard code?
    IntervalTimer<std::chrono::milliseconds> timer{1000/30};

    while (mode != arwain::OperatingMode::Terminate)
    {
        uubla->process_queue();
        uubla->solve_map();
        uubla->process_callbacks();
        arwain::Events::new_uwb_position_event.invoke({get_own_position(), 0}); // TODO This 0 needs to be a dt
        timer.await();
    }

    cleanup_inference();
}

bool UublaWrapper::init()
{
    core_setup();
    job_thread = std::jthread{std::bind_front(&UublaWrapper::run, this)};
    return true;
}

bool UublaWrapper::join()
{
    while (!job_thread.joinable())
    {
        sleep_ms(1);
    }
    if (job_thread.joinable())
    {
        job_thread.join();
        return true;
    }
    return false;
}

double UublaWrapper::get_distance(const int position)
{
    // TODO Do we still need this function?  Library doesn't support it.
    // return uubla->get_distance(position);
    return 0;
}

Vector3 UublaWrapper::get_own_position() const
{
    // return get_node_position(own_id); // TODO
    return {0, 0, 0};
}

Vector3 UublaWrapper::get_node_position(const std::string& node_name) const
{
    return uubla->get_node_position(node_name);
}
