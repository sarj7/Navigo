//===============================================================================
// Advanced Pedestrian Dead Reckoning System with Multi-Feature Step Detection
//===============================================================================

#include "mbed.h"
#include "MMA7660.h"
#include "C12832_lcd.h"
#include <cmath>

//-------------------------------------------------------------------------------
// Hardware Definitions and Initialization
//-------------------------------------------------------------------------------
C12832_LCD lcd;                         // On-board LCD display
MMA7660 MMA(p28, p27);                  // I2C Accelerometer
DigitalOut connectionLed(LED1);         // Accelerometer connection indicator
DigitalOut stepLed(LED2);               // Step detection indicator
DigitalOut modeLed(LED3);               // System mode indicator
DigitalOut errorLed(LED4);              // Error indicator

//-------------------------------------------------------------------------------
// Configuration Parameters 
//-------------------------------------------------------------------------------
const float CALIBRATION_TIME = 3.0;     // Seconds for initial calibration phase
const int WINDOW_SIZE = 20;             // Moving average window size for smoothing

// System operation modes
enum PDR_Mode { 
    CALIBRATING,    // Initial calibration in progress
    READY,          // Calibrated and ready to detect steps
    ACTIVE          // Actively detecting steps and calculating metrics
};

//-------------------------------------------------------------------------------
// Main Program
//-------------------------------------------------------------------------------
int main()
{
    //===========================================================================
    // Variable Initialization
    //===========================================================================
    float ax, ay, az;                    // Raw accelerometer readings for each axis
    float baseline = 0;                  // Baseline (gravity) acceleration magnitude
    
    // Signal processing buffer
    float accelWindow[WINDOW_SIZE] = {0};    // Buffer for moving average filter
    int windowIndex = 0;                     // Current index in moving average window
    
    // Step detection variables
    int stepCount = 0;                   // Total valid steps counted
    bool inStep = false;                 // Step detection state
    float peakValue = 0;                 // Peak acceleration during step
    float stepThreshold = 0.15f;         // Threshold to start detecting a step
    
    // Timer variables
    Timer stepTimer;                     // Measures time between steps
    Timer calibrationTimer;              // Times the calibration phase
    
    // System state
    PDR_Mode currentMode = CALIBRATING;  // Start in calibration mode
    
    //===========================================================================
    // Initialization
    //===========================================================================
    stepTimer.start();
    calibrationTimer.start();
    
    // Initialize display
    lcd.cls();
    lcd.locate(0, 0);
    lcd.printf("Step Counter v1.0");
    lcd.locate(0, 10);
    lcd.printf("Calibrating...");
    
    // Check accelerometer connection
    if (!MMA.testConnection()) {
        // Handle connection error
        errorLed = 1;
        lcd.locate(0, 20);
        lcd.printf("Accelerometer Error!");
        while(1) { wait(0.5); }  // Halt in error state
    }
    
    connectionLed = 1;  // Indicate accelerometer connection OK
    
    //===========================================================================
    // Calibration Phase - Measure baseline acceleration (gravity)
    //===========================================================================
    float calibSum = 0;
    int calibSamples = 0;
    
    while(calibrationTimer.read() < CALIBRATION_TIME) {
        // Read accelerometer data
        ax = MMA.x();
        ay = MMA.y();
        az = MMA.z();
        
        // Calculate acceleration magnitude
        float accel = sqrt(ax*ax + ay*ay + az*az);
        
        // Add to calibration sum
        calibSum += accel;
        calibSamples++;
        
        // Show calibration progress
        int progressWidth = (int)(127 * calibrationTimer.read() / CALIBRATION_TIME);
        lcd.rect(0, 20, progressWidth, 25, 1);
        
        wait(0.01);  // Short delay
    }
    
    // Calculate baseline acceleration and set initial threshold
    baseline = calibSum / calibSamples;
    
    // Transition to READY mode
    currentMode = READY;
    modeLed = 1;  // Indicate ready state
    
    // Update display for READY state
    lcd.cls();
    lcd.locate(0, 0);
    lcd.printf("Ready - Start walking");
    
    //===========================================================================
    // Main Processing Loop
    //===========================================================================
    while(1) {
        // Read accelerometer data for all axes
        ax = MMA.x();
        ay = MMA.y();
        az = MMA.z();
        
        // Calculate acceleration magnitude and remove baseline
        float accel = sqrt(ax*ax + ay*ay + az*az) - baseline;
        
        // Apply simple low-pass filter to reduce noise
        float filteredAccel = 0;
        accelWindow[windowIndex] = accel;
        windowIndex = (windowIndex + 1) % WINDOW_SIZE;
        
        for (int i = 0; i < WINDOW_SIZE; i++) {
            filteredAccel += accelWindow[i];
        }
        filteredAccel /= WINDOW_SIZE;
        
        // Step detection state machine for more accuracy
        if (!inStep) {
            // Looking for start of step (rising edge)
            if (filteredAccel > stepThreshold && stepTimer.read() > 0.25) {
                inStep = true;
                peakValue = filteredAccel;
                currentMode = ACTIVE;
                
                // When step starts, LED on
                stepLed = 1;
            }
        } else {
            // In step - track peak and look for completion
            
            // Update peak if we find a higher value
            if (filteredAccel > peakValue) {
                peakValue = filteredAccel;
            }
            
            // Detect end of step (when acceleration drops significantly)
            if (filteredAccel < (peakValue * 0.4) && stepTimer.read() > 0.35) {
                // Valid step detected
                stepCount++;
                
                // Update display immediately
                lcd.cls();
                wait_ms(50);  // Allow the LCD to clear fully
                
                lcd.locate(0, 0);
                lcd.printf("STEP COUNT");
                lcd.locate(0, 10);
                lcd.printf("%d", stepCount);
                
                // Show step strength
                lcd.locate(0, 20);
                lcd.printf("Strength: %.2f", peakValue);
                
                wait_ms(50);  // Allow the text to render
                
                // Adaptive threshold - adjust based on peak value
                stepThreshold = peakValue * 0.3;
                if (stepThreshold < 0.1f) stepThreshold = 0.1f;
                if (stepThreshold > 0.5f) stepThreshold = 0.5f;
                
                // Reset for next step
                inStep = false;
                stepTimer.reset();
                
                // Toggle LED for visual feedback
                stepLed = 0;
                
                // Short delay after detection to avoid double-counting
                wait(0.2);  // Reduced wait time to detect steps that occur closer together
            }
            
            // Timeout for step detection - prevent getting stuck in "in step" state
            if (stepTimer.read() > 1.5) {
                inStep = false;
                stepTimer.reset();
                stepLed = 0;
            }
        }
        
        // Add a jerk-based detection method to catch steps that the main algorithm might miss
        // Calculate jerk (rate of change of acceleration)
        static float prevAccel = 0;
        float jerk = fabs(filteredAccel - prevAccel) / 0.02;
        prevAccel = filteredAccel;
        
        // If we detect a strong jerk and not already in a step, this might be a step too
        if (!inStep && jerk > 0.8 && stepTimer.read() > 0.25) {
            // Use jerk detection as a secondary method
            stepCount++;
            
            // Visual feedback
            stepLed = !stepLed;
            
            // Update display
            lcd.cls();
            wait_ms(50);
            lcd.locate(0, 0);
            lcd.printf("STEP COUNT");
            lcd.locate(0, 10);
            lcd.printf("%d", stepCount);
            lcd.locate(0, 20);
            lcd.printf("Jerk: %.2f", jerk);
            
            wait_ms(50);
            
            // Reset step timer
            stepTimer.reset();
            
            // Small delay to avoid duplicates
            wait(0.2);
        }
        
        // Lower threshold if we haven't detected steps in a while
        if (stepTimer.read() > 3.0) {
            // If no steps for 3 seconds, gradually lower threshold to become more sensitive
            stepThreshold = max(0.05f, stepThreshold * 0.95f);
        }
        
        // Sample at a consistent rate
        wait(0.02);
    }
}

