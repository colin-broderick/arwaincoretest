#include <tuple>
#include <string>
#include <sstream>
#include <algorithm>
#include <thread>

#include "arwain/arwain.hpp"
#include "arwain/serial.hpp"
#include "arwain/logger.hpp"
#include "arwain/exceptions.hpp"
#include "arwain/transmit_lora.hpp"
#include "arwain/uwb_reader.hpp"
#include "arwain/service_manager.hpp"
#include "arwain/hybrid_positioner.hpp"
#include "arwain/velocity_prediction.hpp"
#include "arwain/ws_messenger.hpp"

#include "uubla/utils.hpp"
#include "uubla/events.hpp"
#include "uubla/network.hpp"

#define PUBLISH_HYBRID_POSITIONS

std::string state_string(UUBLA::Network* uubla);
void pin_thread(std::jthread& th, int core_number);

UublaWrapper::UublaWrapper()
{
    ServiceManager::register_service(this, service_name);

    // The websocket server exists to receive position data from the Unity front end.
    // Upon reception, data is fed to the UUBLA instance running here and there it is
    // used for solving for tag position.
    core_setup();
    job_thread = std::jthread{std::bind_front(&UublaWrapper::run, this)};
}

UublaWrapper::~UublaWrapper()
{
    UUBLA::run_flag = false;
    ServiceManager::unregister_service(service_name);
}

void UublaWrapper::core_setup()
{
    m_uubla = std::make_unique<UUBLA::Network>();
    m_uubla->force_plane(arwain::config.force_z_zero);
}

void UublaWrapper::run_idle()
{
    sleep_ms(10);
}

void UublaWrapper::run()
{
    // if (!arwain::config.use_uwb_positioning || !(arwain::config.node_id < 10))
    // {
    //     return;
    // }

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
    m_uubla->set_ewma_gain(0.1);
    // m_uubla->add_node_callback = inform_new_uubla_node; // Replaced with event registrations.
    // m_uubla->remove_node_callback = inform_remove_uubla_node; // Replaced with event registrations.
    serial_reader_th = std::jthread{serial_reader_fn, m_uubla.get(), arwain::config.uubla_serial_port, 115200};
    pin_thread(serial_reader_th, 3);
}

bool UublaWrapper::cleanup_inference()
{
    UUBLA::run_flag = false;
    return true;
}

void publish_inertial_on_uwb()
{
    auto inf = ServiceManager::get_service<PositionVelocityInference>(PositionVelocityInference::service_name);
    if (inf)
    {
        auto pos = inf->get_position();
        UUBLA::add_to_send_queue({pos, arwain::config.node_id});
    }
}

void UublaWrapper::run_inference()
{
    setup_inference();

    // TODO: Get the framerate (30?) from somewhere sensible, or hard code?
    IntervalTimer<std::chrono::milliseconds> timer{1000 / (15 /* hz */)};

    auto last_count = timer.count();
    auto frame_counter = 0;
    while (mode == arwain::OperatingMode::Inference)
    {
        frame_counter++;

        if (arwain::config.use_uwb_positioning)
        {
            m_uubla->process_queue();
            m_uubla->solve_map();
            auto now_count = timer.count();
            // TODO The maths below is because interval timer.count doesn't return the milisecond count as it should.
            // Consider also the timing in other inetval timer locations before making changes.
            arwain::Events::new_uwb_position_event.invoke({get_own_position(), (now_count - last_count) / 1000.0 / 1000.0 / 1000.0});
            last_count = now_count;
        }

        auto messenger = ServiceManager::get_service<WsMessenger>(WsMessenger::service_name);
        if (messenger && frame_counter % 3 == 0)
        {
            messenger->send_dash_message(state_string(m_uubla.get()));
        }

        // We still want to publish positions over WebSocket, even if we aren't using UWB positioning.
        if (messenger)
        {
            messenger->publish_positions_on_websocket(*m_uubla);
        }

        // publish_inertial_on_uwb();
        timer.await();
    }

    cleanup_inference();
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
    // return m_uubla->get_distance(position);
    return 0;
}

Vector3 UublaWrapper::get_own_position() const
{
    return m_uubla->get_node_position(
        UUBLA::Node::name_from_int(
            arwain::config.node_id
        )
    );
}

Vector3 UublaWrapper::get_node_position(const std::string& node_name) const
{
    return m_uubla->get_node_position(node_name);
}

bool UublaWrapper::network_contains(int node_id)
{
    auto str_node_id = UUBLA::Node::name_from_int(node_id);
    return m_uubla->get_nodes().count(str_node_id) > 0;

}

void UublaWrapper::fix_node_at(int node_id, const Vector3& new_position)
{
    auto str_node_id = UUBLA::Node::name_from_int(node_id);
    auto n = m_uubla->get_nodes().at(str_node_id);
    n.fix_position();
    n.set_position(new_position);
}
