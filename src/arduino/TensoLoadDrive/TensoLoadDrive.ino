#include <HX711.h>

// Using HX711 for load cell reading
#define DT  3
#define SCK 2

// Using L298N motor driver
#define PIN_IN1 7
#define PIN_IN2 6
#define PIN_ENA 9   // Motor PWM control pin

HX711 scale;

float loadTask_gram = 2000;
float loadForce_gram = 0;

const int dead_zone = 1;

const float p_coef = 20;
const float i_coef = 1.1;
const float d_coef = 0.5;
float error = 0;
float prev_error = 0;
float g_integr = 0;
float control_output = 0;

unsigned long timestamp = 0;
unsigned long control_system_dt = 13; // 12,5 ms means 80 Hz

void setup()
{
    // Serial for monitoring
    Serial.begin(115200);

    // Initialize HX711
    scale.begin(DT, SCK);
    scale.set_scale();
    scale.tare();

    pinMode(PIN_IN1, OUTPUT);
    pinMode(PIN_IN2, OUTPUT);

    // Send motor to upper limit
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, HIGH);
    analogWrite(PIN_ENA, 1000);
    delay(5000);

    // Set zero on load cell when shaft does not touch it
    scale.set_scale();
    scale.tare();
}

// Average filter of HX711 results
// Max hardware frequency is 80 Hz
#define FILTER_SIZE 1

float read_weight_from_adc()
{
    float units = 0;
    for (int i = 0; i < FILTER_SIZE; i++) 
    {
        units += scale.get_units(); 
    }
    units = units / FILTER_SIZE;
    return units;
}

void loop() 
{
    if (millis() - timestamp > control_system_dt)
    {
        doCalculations();  
  
        Serial.print("e:");
        Serial.print(error);
        Serial.print(", u:");
        Serial.print(control_output);
        Serial.print(", x / 10:");
        Serial.println(loadForce_gram / 10);
    }

    if (Serial.available())
    {
        loadTask_gram = Serial.parseFloat();
    }
}

void doCalculations()
{
    // Units to grams with a calbration result divider
    loadForce_gram = read_weight_from_adc() / 467;

    // Error in %
    error = (loadTask_gram - loadForce_gram) * 100 / loadTask_gram; 
    
    // Integral with saturation
    g_integr += error;
    if (g_integr > 80)
    {
        g_integr = 80;
    } 
    else if (g_integr < -80) 
    {
        g_integr = -80;      
    }

    control_output = p_coef * error 
                     + d_coef * (error - prev_error) / control_system_dt 
                     + i_coef * g_integr;

    // Dead zone and control sign (direction) processing
    if (control_output < -1 * dead_zone)
    {
      digitalWrite(PIN_IN1, LOW);
      digitalWrite(PIN_IN2, HIGH);
      control_output = -1 * control_output;
    }
    else if (control_output > dead_zone)
    {
      digitalWrite(PIN_IN1, HIGH);
      digitalWrite(PIN_IN2, LOW); 
    }

    if (control_output > 255)
      control_output = 255;

    if (error < 1 && error > -1)
        control_output = 0;       

    analogWrite(PIN_ENA, control_output);

    prev_error = error;
}
