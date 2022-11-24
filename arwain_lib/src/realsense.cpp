#include "realsense.hpp"
#include "exceptions.hpp"
#include "arwain_thread.hpp"
#include "logger.hpp"

CameraController::CameraController()
{
    init();
}

void CameraController::setup_data_collection()
{
    camera_position_log.open(arwain::folder_date_string + "/position_t265.txt");
    camera_position_log << "time x y z\n";
}

void CameraController::cleanup_data_collection()
{
    camera_position_log.close();
}

void CameraController::core_setup()
{
    camera = new T265;
}

void CameraController::run_data_collection()
{
    setup_data_collection();

    while (arwain::system_mode == arwain::OperatingMode::DataCollection)
    {
        Vector3 pos = Vector3::from_array(camera->get_position());
        camera_position_log << std::chrono::system_clock::now().time_since_epoch().count() << " "
                            << pos.x << " "
                            << pos.y << " "
                            << pos.z << "\n";
    }
    
    cleanup_data_collection();
}

void CameraController::run_idle()
{
    sleep_ms(10);
}

void CameraController::run()
{
    while (arwain::system_mode != arwain::OperatingMode::Terminate)
    {
        switch (arwain::system_mode)
        {
            case arwain::OperatingMode::DataCollection:
                run_data_collection();
                break;
            default:
                run_idle();
                break;
        }
    }
}

bool CameraController::init()
{
    if (!arwain::config.use_rs2)
    {
        return false;
    }
    core_setup();
    job_thread = ArwainThread{&CameraController::run, "arwain_cmra_th", this};
    return true;
}

void CameraController::join()
{
    delete camera;
    if (job_thread.joinable())
    {
        job_thread.join();
    }
}
