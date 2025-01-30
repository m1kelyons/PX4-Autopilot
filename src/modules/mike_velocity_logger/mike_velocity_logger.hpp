#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/vehicle_imu.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/integrated_accel.h>

using matrix::Vector3f;
using uORB::SubscriptionData;
using namespace time_literals;

class MikeVelocityLogger: public ModuleBase<MikeVelocityLogger>, public px4::ScheduledWorkItem
{
public:
	MikeVelocityLogger();
	~MikeVelocityLogger();

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

private:
	uORB::Publication<integrated_accel_s> _integrated_accel_pub{ORB_ID(integrated_accel)};
	uORB::Subscription _vehicle_imu_sub{ORB_ID(vehicle_imu)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	perf_counter_t	_loop_perf;
	vehicle_imu_s _vehicle_imu{};
	vehicle_status_s _vehicle_status{};

	float vx = 0.0f, vy = 0.0f, vz = 0.0f;
	float dt = 1000.0f; // Integration delta time in microseconds
	float scaling_factor_imu = 1.0e6f; // Convert to correct units
	float imu_axes_directions[3] = {-1.0f, -1.0f, -1.0f}; // IMU installation direction of axes

	void Run() override;
	void integrate_accel_and_publish(const vehicle_imu_s &imu);
};
