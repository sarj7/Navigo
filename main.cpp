//===============================================================================
// Advanced Pedestrian Dead Reckoning System with Multi-Feature Step Detection
//===============================================================================

#include "mbed.h"
#include "MMA7660.h"
#include "C12832_lcd.h"
#include <cmath>
#include <algorithm>

//-------------------------------------------------------------------------------
// Hardware Definitions and Initialization
//-------------------------------------------------------------------------------
C12832_LCD lcd;                         // On-board LCD display
MMA7660 MMA(p28, p27);                  // I2C Accelerometer
DigitalOut connectionLed(LED1);         // Accelerometer connection indicator
DigitalOut stepLed(LED2);               // Step detection indicator

//-------------------------------------------------------------------------------
// Configuration Parameters 
//-------------------------------------------------------------------------------
const float CALIBRATION_TIME    = 3.0f;    // Calibration duration (seconds)
const float STEP_THRESHOLD_INIT = 0.20f;   // Lower initial threshold
const float MIN_STEP_INTERVAL   = 0.5f;    // Minimum interval between steps (seconds)
const float ALPHA               = 0.2f;    // Increase smoothing factor for a sharper response

//-------------------------------------------------------------------------------
// Main Program
//-------------------------------------------------------------------------------
int main() {
    // Variables for calibration and filtering
    float ax, ay, az;
    float baseline = 0.0f;
    float calibSum = 0.0f;
    int calibSamples = 0;
    
    // Step detection variables
    int stepCount = 0;
    bool inStep = false;
    float peakValue = 0.0f;
    float stepThreshold = STEP_THRESHOLD_INIT;
    float smoothAccel = 0.0f;
    
    // Timers
    Timer calibTimer, stepTimer;
    calibTimer.start();
    stepTimer.start();
    
    // Startup display and calibration message
    lcd.cls();
    lcd.locate(0, 0);
    lcd.printf("Step Counter v1.0");
    lcd.locate(0, 10);
    lcd.printf("Calibrating...");
    
    // Verify accelerometer connection
    if (!MMA.testConnection()) {
        while (1) { wait(0.5); }
    }
    connectionLed = 1;
    
    // Calibration phase: measure the baseline (gravity magnitude)
    while (calibTimer.read() < CALIBRATION_TIME) {
        ax = MMA.x(); 
        ay = MMA.y(); 
        az = MMA.z();
        float accel = sqrt(ax*ax + ay*ay + az*az);
        calibSum += accel;
        calibSamples++;
        // Display calibration progress (progress bar)
        int progressWidth = (int)(127 * calibTimer.read() / CALIBRATION_TIME);
        lcd.rect(0, 20, progressWidth, 25, 1);
        wait(0.01);
    }
    baseline = calibSum / calibSamples;
    
    // Ready message
    lcd.cls();
    lcd.locate(0, 0);
    lcd.printf("Ready - Walk");
    
    // Main processing loop
    while (true) {
        // Read accelerometer and subtract baseline
        ax = MMA.x(); 
        ay = MMA.y(); 
        az = MMA.z();
        float rawAccel = sqrt(ax*ax + ay*ay + az*az) - baseline;
        
        // Exponential smoothing
        smoothAccel = ALPHA * rawAccel + (1.0f - ALPHA) * smoothAccel;
        
        // Step detection state machine
        if (!inStep) {
            // Detect rising edge with minimum step interval condition
            if (smoothAccel > stepThreshold && stepTimer.read() > MIN_STEP_INTERVAL) {
                inStep = true;
                peakValue = smoothAccel;
                stepLed = 1;
            }
        } else {
            // Update peak during the step
            if (smoothAccel > peakValue) {
                peakValue = smoothAccel;
            }
            // Detect falling edge: use a fraction of the peak value to register the step end
            if (smoothAccel < peakValue * 0.7f && stepTimer.read() > MIN_STEP_INTERVAL) {
                stepCount++;
                // Update display with step count and peak value
                lcd.cls();
                lcd.locate(0, 0);
                lcd.printf("STEP COUNT");
                lcd.locate(0, 10);
                lcd.printf("%d", stepCount);
                lcd.locate(0, 20);
                lcd.printf("Peak: %.2f", peakValue);
                
                // Adaptive threshold adjustment
                float newThreshold = std::min(peakValue * 0.4f, 0.35f);
                stepThreshold = std::max(STEP_THRESHOLD_INIT, newThreshold);
                
                // Reset state for next step
                inStep = false;
                stepTimer.reset();
                stepLed = 0;
                wait(0.3f); // Delay to avoid duplicate counts
            }
            // Timeout: reset step detection if it hangs
            if (stepTimer.read() > 2.0f) {
                inStep = false;
                stepTimer.reset();
                stepLed = 0;
            }
        }
        // Control sampling rate
        wait(0.01);
    }
}

