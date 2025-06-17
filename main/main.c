

static const char *TAG = "app";

static drv8308_t motor;
static abp_sensor_t pressure_sensor;
static float kp = 1.0f, ki = 0.1f, kd = 0.01f;
static float integral = 0, prev_error = 0;
static float setpoint = 50.0f; // target pressure

static void control_task(void *arg)
{

