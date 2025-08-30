/*

Send a PWM step to the motor and get the current angular position and velocity of the motor (DC6V210RPM - Reducer 20:1).
The data is send/received through serial port.

This script uses the 3.X version of the Arduino ESP32 core.

Author: Juan M. Gandarias
email: jmgandarias@uma.es

*/

#define ENCODER_A 26
#define ENCODER_B 27

#define PWM_CW_PIN 32
#define PWM_CCW_PIN 33

#define LED_PIN 13

// timer variables
hw_timer_t *timer = NULL;
int timer_frequency = 1e6;
volatile bool timer_activated = false;

// PWM configuration
const int frequency = 1000;
const int resolution = 10;
const int pwm_max = (1 << resolution) - 1; // 1023 for 10-bit
volatile int32_t cmd_PWM = 0;

// Encoder counter
volatile int counter = 0;
volatile int last_count = 0;

// Store the last time
volatile uint32_t last_time = 0;

// Convert to radians
const float pulsesPerRevolution = 880.0;
const float radiansPerPulse = 2 * PI / pulsesPerRevolution;

// Experiment control
int experiment_time = 5e3; // time in milliseconds
bool start_experiment = false;

void IRAM_ATTR ISRENCODER_A()
{
    if (digitalRead(ENCODER_A) == digitalRead(ENCODER_B))
    {
        counter++;
    }
    else
    {
        counter--;
    }
}

void IRAM_ATTR ISRENCODER_B()
{
    if (digitalRead(ENCODER_A) == digitalRead(ENCODER_B))
    {
        counter--;
    }
    else
    {
        counter++;
    }
}

// Callback ISR timer interrupt
void IRAM_ATTR timerInterrupt()
{
    timer_activated = true;
}

void setup()
{
    Serial.begin(2000000);
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    ledcAttach(PWM_CW_PIN, frequency, resolution);
    ledcAttach(PWM_CCW_PIN, frequency, resolution);

    pinMode(ENCODER_A, INPUT_PULLUP);
    pinMode(ENCODER_B, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(ENCODER_A), ISRENCODER_A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_B), ISRENCODER_B, CHANGE);

    timer = timerBegin(timer_frequency);          // Initializes the timer at timer_frequency
    timerAttachInterrupt(timer, &timerInterrupt); // Set the ISR associated to the timer
    // Establish the alarm every 1ms
    timerAlarm(timer, 1e3, true, 0);
    timerStop(timer); // The timer will start when the button is pressed
}

void loop()
{

    if (!start_experiment)
    {
        if (Serial.available() > 0)
        {
            // read the incoming byte:
            int incoming_byte = Serial.read();
            if (incoming_byte == '1')
                start_experiment = true;

            if (start_experiment)
            {
                digitalWrite(LED_PIN, HIGH);
                timerRestart(timer);
            }
        }
    }

    if (timer_activated)
    {
        long delta_pos_ppr = counter - last_count;
        uint32_t current_time = timerReadMillis(timer);
        float elapsed_time = (current_time - last_time) / 1000.0; // convert ms to seconds
        float delta_pos = delta_pos_ppr * radiansPerPulse;

        // Calculate position
        float pos = counter * radiansPerPulse;

        // Calculate velocity
        float vel = 0.0f;
        if (elapsed_time > 0.0f)
        {
            vel = delta_pos / elapsed_time;
        }

        if (current_time < experiment_time / 2)
        {
            int pwm = 0;
            ledcWrite(PWM_CW_PIN, 0);
            ledcWrite(PWM_CCW_PIN, 0);
            Serial.printf("%d;%.4f;%.4f;%d\n", pwm, pos, vel, current_time);
        }
        else if (current_time < experiment_time)
        {
            int pwm = 1;
            ledcWrite(PWM_CW_PIN, pwm_max);
            ledcWrite(PWM_CCW_PIN, 0);
            Serial.printf("%d;%.4f;%.4f;%d\n", pwm, vel, current_time);
        }
        else
        {
            ledcWrite(PWM_CW_PIN, 0);
            ledcWrite(PWM_CCW_PIN, 0);
            Serial.println("END");
            timerStop(timer);
            digitalWrite(LED_PIN, LOW);
            start_experiment = false;
            last_count = 0;
            last_time = 0;
            counter = 0;
        }
        timer_activated = false;

        // Update last values
        last_count = counter;
        last_time = current_time;
    }
}
