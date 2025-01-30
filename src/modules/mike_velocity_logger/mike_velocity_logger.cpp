#include "mike_velocity_logger.hpp"

extern "C" __EXPORT int mike_velocity_logger_main(int argc, char *argv[]) {
    return MikeVelocityLogger::main(argc, argv);
}

MikeVelocityLogger::MikeVelocityLogger():
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
}

MikeVelocityLogger::~MikeVelocityLogger()
{
	perf_free(_loop_perf);
}

void MikeVelocityLogger::integrate_accel_and_publish(const vehicle_imu_s &imu) {


    dt = imu.delta_velocity_dt * 1e-6f; // Convert microseconds to seconds

    vx += imu_axes_directions[0] * scaling_factor_imu * imu.delta_velocity[0] * dt;
    vy += imu_axes_directions[1] * scaling_factor_imu * imu.delta_velocity[1] * dt;
    vz += imu_axes_directions[2] * scaling_factor_imu * imu.delta_velocity[2] * dt;

    integrated_accel_s msg{};
    msg.timestamp = hrt_absolute_time();
    msg.vx = vx;
    msg.vy = vy;
    msg.vz = vz;

    _integrated_accel_pub.publish(msg);

}

void MikeVelocityLogger::Run() {

    if (should_exit()) {
		ScheduleClear();
		return;
	}

	perf_begin(_loop_perf);

    if (_vehicle_imu_sub.update(&_vehicle_imu)) {
        _vehicle_status_sub.update(&_vehicle_status);

        if (_vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
            integrate_accel_and_publish(_vehicle_imu);
        }
    }
    perf_end(_loop_perf);
    //ScheduleNow();
    ScheduleDelayed(100000); // Run at 10 Hz
}

int MikeVelocityLogger::task_spawn(int argc, char *argv[]) {
    MikeVelocityLogger *instance = new MikeVelocityLogger();

    if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;
        instance->ScheduleNow();
	}

    return PX4_OK;

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int MikeVelocityLogger::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int MikeVelocityLogger::print_usage(const char *reason)
{
    if (reason) {
        PX4_WARN("%s", reason);
    }
    PX4_INFO("Usage: mike_velocity_logger {start|stop|status}");
    return 0;
}




