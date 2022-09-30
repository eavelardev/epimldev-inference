#include "ei_run_classifier.h"
#include <driver/gpio.h>
#include <driver/ledc.h>
#include "esp_timer.h"
#include "i2c_conf.h"
#include "lis3dh.h"

#define CONVERT_G_TO_MS2        9.80665f
#define INFERENCE_SAMPLING_TIME 200 // > 25ms (min inference time) 
#define ALERT_INTERVAL          500
#define SEIZURE_THRESHOLD       0.9
#define MAX_SEIZURE_VALID_TIME  (8)  // 8*500=4000ms
#define MAX_FILTER_ACTIVATION   (3)  // 3*200=600ms

#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY               (4095) // Set duty to 50%. ((2 ** 13) - 1) * 50% = 4095

#define BUZZER_PIN              (25) // Define the output GPIO
#define BUZZER_FREQUENCY        (4000) // Frequency in Hertz. Set frequency at 4 kHz

#define VIBRATOR_PIN    GPIO_NUM_26
#define BUTTON_PIN      GPIO_NUM_0       

typedef enum {
    IMU_STOPPED,
    IMU_WAITING,
    IMU_SAMPLING,
    IMU_DATA_READY
} inference_state_t;

esp_timer_handle_t imu_timer;
esp_timer_handle_t alert_timer;

static inference_state_t state = IMU_STOPPED;
static uint64_t last_inference_ts = 0;
static bool debug_mode = false;
static float samples_circ_buff[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE];
static int samples_wr_index = 0;
static int last_samples_wr_index = 0;

static bool seizure_detected = false;
static bool seizure_detected_state = false;

static int send_alert_wifi = 0;
static bool seizure_validated = false;
static bool seizure_validated_state = false;
static int seizure_validation_counter = 0;
static int filter_activation_counter = 0;

static float imu_data[EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME];
LIS3DH lis3dh;

bool inertial_init(void) {

    i2c_bus = i2c_bus_create(I2C_MASTER_NUM, &conf);

    lis3dh.begin(i2c_bus, LIS3DH_ADDRESS_UPDATED);

    if(lis3dh.isConnection() == false) {
        printf("ERR: failed to connect to inertial sensor!\n");
        return false;
    }

    ei_sleep(100);
    lis3dh.setFullScaleRange(LIS3DH_RANGE_2G);
    lis3dh.setOutputDataRate(LIS3DH_DATARATE_100HZ);

    return true;
}

static void periodic_timer_callback(void* arg)
{
    lis3dh.getAcceleration(&imu_data[0], &imu_data[1], &imu_data[2]);
    imu_data[0] *= CONVERT_G_TO_MS2;
    imu_data[1] *= CONVERT_G_TO_MS2;
    imu_data[2] *= CONVERT_G_TO_MS2;

    for(int i = 0; i < EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME; i++)
        samples_circ_buff[samples_wr_index++] = imu_data[i];  

    if(samples_wr_index == EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
        /* start from beginning of the circular buffer */
        samples_wr_index = 0;

        if(state == IMU_SAMPLING)
            state = IMU_DATA_READY;
    }

    if(last_samples_wr_index != samples_wr_index) {
        last_samples_wr_index = samples_wr_index;

        if(state == IMU_WAITING)
            state = IMU_DATA_READY;
    }
}

void buzzer_stop(){
    ESP_ERROR_CHECK(ledc_stop(LEDC_MODE, LEDC_CHANNEL, 0));
}

void buzzer_start(){
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
}

static void alert_callback(void* arg)
{
    if(seizure_detected == true)
    {
        send_alert_wifi = 1;
        ei_printf("seizure_detected\n");
        if (seizure_detected_state == false) {
            gpio_set_level(VIBRATOR_PIN, 1);
            buzzer_start();
            seizure_detected_state = true;
        }
        else {
            gpio_set_level(VIBRATOR_PIN, 0);
            buzzer_stop();
            seizure_detected_state = false;
        }

        seizure_validation_counter++;
        
        if (seizure_validation_counter == MAX_SEIZURE_VALID_TIME)
        {
            state = IMU_SAMPLING;
            samples_wr_index = 0;
            last_samples_wr_index = 0;
            seizure_validated = true;
            send_alert_wifi = 2;
            seizure_detected = false;
            seizure_validation_counter = 0;
        }
    } else if (seizure_validated == true) {
        ei_printf("seizure_validated\n");
        if (seizure_validated_state == false)
        {
            buzzer_start();
            seizure_validated_state = true;
        } else {
            buzzer_stop();
            seizure_validated_state = false;
        }
    }
}

void create_timer_alert(void)
{
    const esp_timer_create_args_t periodic_alert_args = {
        .callback = &alert_callback,
        .name = "Alert"
    };

    ESP_ERROR_CHECK(esp_timer_create(&periodic_alert_args, &alert_timer));    
    ESP_ERROR_CHECK(esp_timer_start_periodic(alert_timer, ALERT_INTERVAL * 1000));
}

static void display_results(ei_impulse_result_t* result)
{
    // ei_printf("\nPredictions (DSP: %d ms., Classification: %d ms., Anomaly: %d ms.): \n",
        // result->timing.dsp, result->timing.classification, result->timing.anomaly);
    for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {          
        if (strcmp(result->classification[ix].label, "seizure") == 0) {
            // ei_printf("    %s: \t%f", result->classification[ix].label, result->classification[ix].value);
            if (result->classification[ix].value >= SEIZURE_THRESHOLD) {
                filter_activation_counter++;

                if ((filter_activation_counter == MAX_FILTER_ACTIVATION) && (seizure_validated == false)){
                    seizure_detected = true;
                    filter_activation_counter = 0;
                } 
            }else {
                filter_activation_counter = 0;
            }
        } 
    }
#if EI_CLASSIFIER_HAS_ANOMALY == 1
    // ei_printf("    anomaly score: %f\r\n", result->anomaly);
#endif
}

void ei_run_impulse(void)
{
    signal_t signal;

    // shift circular buffer, so the newest data will be the first
    // if samples_wr_index is 0, then roll is immediately returning
    numpy::roll(samples_circ_buff, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, (-samples_wr_index));

    // Create a data structure to represent this window of data
    int err = numpy::signal_from_buffer(samples_circ_buff, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);
    if (err != 0) {
        ei_printf("ERR: signal_from_buffer failed (%d)\n", err); 
    }

    // run the impulse: DSP, neural network and the Anomaly algorithm
    ei_impulse_result_t result = { 0 };
    EI_IMPULSE_ERROR ei_error;

    ei_error = run_classifier(&signal, &result, debug_mode);

    if (ei_error != EI_IMPULSE_OK) {
        ei_printf("Failed to run impulse (%d)", ei_error);
        return;
    }

    display_results(&result);

    state = IMU_WAITING;
}

void start_imu_sampling(void)
{ 
    const esp_timer_create_args_t periodic_timer_args = {
        .callback = &periodic_timer_callback,
        .name = "IMU sampler"
    };

    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &imu_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(imu_timer, EI_CLASSIFIER_INTERVAL_MS * 1000));
}

void stop_imu_sampling(void)
{
    ESP_ERROR_CHECK(esp_timer_stop(imu_timer));
    ESP_ERROR_CHECK(esp_timer_delete(imu_timer));
}

void gpio_init()
{
    gpio_pad_select_gpio(BUTTON_PIN);
    gpio_set_direction(BUTTON_PIN, GPIO_MODE_INPUT);

    gpio_pad_select_gpio(VIBRATOR_PIN);
    gpio_set_direction(VIBRATOR_PIN, GPIO_MODE_OUTPUT);
}

static void buzzer_init()
{
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .duty_resolution  = LEDC_DUTY_RES,
        .timer_num        = LEDC_TIMER,
        .freq_hz          = BUZZER_FREQUENCY,  // Set output frequency
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .gpio_num       = BUZZER_PIN,
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = LEDC_TIMER,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0,
        .flags = {
            .output_invert = 0
        }
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

extern "C" int app_main()
{
    /* Disable Leds */
    // gpio_pad_select_gpio(GPIO_NUM_4);
    // gpio_reset_pin(GPIO_NUM_4);
    // gpio_set_direction(GPIO_NUM_4, GPIO_MODE_OUTPUT);
    // gpio_set_level(GPIO_NUM_4, 1);  

    gpio_init();
    buzzer_init();

    /* Setup the inertial sensor */
    if (inertial_init() == false) {
        printf("Inertial sensor initialization failed\r\n");
        return 0;
    }

    const float sample_length = 1000.0f * static_cast<float>(EI_CLASSIFIER_RAW_SAMPLE_COUNT) /
                        (1000.0f / static_cast<float>(EI_CLASSIFIER_INTERVAL_MS));
    
    ei_printf("\nInferencing settings:\n");
    ei_printf("\tInterval: %d ms.\n", (int)EI_CLASSIFIER_INTERVAL_MS);
    ei_printf("\tFrame size: %d\n", EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE);
    ei_printf("\tSample length: %d ms.\n", (int)sample_length);
    ei_printf("\tNo. of classes: %d\n\n", sizeof(ei_classifier_inferencing_categories) /
                                            sizeof(ei_classifier_inferencing_categories[0]));

    state = IMU_SAMPLING;
    start_imu_sampling();
    create_timer_alert();
    last_inference_ts = ei_read_timer_ms();   

    while(true)
    {
        if (state != IMU_DATA_READY){
            ei_sleep(1);
        }else if (ei_read_timer_ms() < (last_inference_ts + INFERENCE_SAMPLING_TIME)) {
            ei_sleep(1);
        }else {
            int actual_inference_sampling_time = ei_read_timer_ms() - last_inference_ts;
            last_inference_ts = ei_read_timer_ms();

            if(seizure_detected == false) {
                int tic = ei_read_timer_ms();
                ei_run_impulse();
                int toc = ei_read_timer_ms();
                
                int inference_time = toc - tic;
                // ei_printf("Sampling time: %d ms\n", actual_inference_sampling_time);
                // ei_printf("Inference time: %d ms\n\n", inference_time);
            }
            
            ei_printf("Signal: %d\n", send_alert_wifi);
        }  

        if (gpio_get_level(BUTTON_PIN) == 0 && (seizure_detected == true || seizure_validated == true))
        {
            gpio_set_level(VIBRATOR_PIN, 0);
            buzzer_stop();
            send_alert_wifi = 0;
            seizure_detected = false;
            seizure_detected_state = false;
            seizure_validated = false;
            seizure_validated_state = false;
            seizure_validation_counter = 0;
            state = IMU_SAMPLING;
            samples_wr_index = 0;
            last_samples_wr_index = 0;
        }
    }

    stop_imu_sampling();

    return 0;
}
