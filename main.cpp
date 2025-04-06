//===============================================================================
// Advanced Pedestrian Dead Reckoning System with Multi-Feature Step Detection
// 
// This application detects steps using the onboard accelerometer and implements
// a sophisticated pedestrian dead reckoning system that:
//  1. Calibrates to determine the baseline acceleration (gravity)
//  2. Uses multiple features (magnitude, jerk, variance) to detect steps
//  3. Employs adaptive thresholding to accommodate different walking patterns
//  4. Calculates distance, speed, and cadence metrics
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
// PDR Configuration Parameters 
//-------------------------------------------------------------------------------
// Algorithm tuning parameters - adjust these based on testing
const float CALIBRATION_TIME = 3.0;     // Seconds for initial calibration phase
const int WINDOW_SIZE = 20;             // Moving average window size for smoothing
const int HISTORY_SIZE = 100;           // Acceleration history buffer size
const float MIN_STEP_INTERVAL = 0.25;   // Minimum time between steps (seconds)
const float MAX_STEP_INTERVAL = 2.0;    // Maximum time between steps (seconds)
const float ADAPTIVE_ALPHA = 0.05;      // Learning rate for adaptive threshold
const float JERK_THRESHOLD_MULT = 1.2;  // Jerk threshold multiplier

// Step analysis parameters
const int PATTERN_SIZE = 5;             // Number of steps to use for pattern analysis
const float MIN_CONFIDENCE = 0.6;       // Minimum confidence to count a valid step
const float DEFAULT_STEP_LENGTH = 0.7;  // Default step length in meters

//-------------------------------------------------------------------------------
// Data Structures
//-------------------------------------------------------------------------------
// System operation modes
enum PDR_Mode { 
    CALIBRATING,    // Initial calibration in progress
    READY,          // Calibrated and ready to detect steps
    ACTIVE,         // Actively detecting steps and calculating metrics
    ERROR           // Error state
};

// Structure to hold feature data for step detection
struct StepFeatures {
    float accelMagnitude;       // Magnitude of acceleration
    float jerk;                 // Rate of change of acceleration (derivative)
    float accelVariance;        // Short-term variance of acceleration
    float stepConfidence;       // Confidence score for step detection (0-1)
};

//-------------------------------------------------------------------------------
// Main Program
//-------------------------------------------------------------------------------
int main()
{
    //===========================================================================
    // Variable Initialization
    //===========================================================================
    
    // Core sensor variables
    float ax, ay, az;                    // Raw accelerometer readings for each axis
    float baseline = 0;                  // Baseline (gravity) acceleration magnitude
    float dynamicThreshold = 0;          // Adaptive step detection threshold
    float prevDynamicThreshold = 0;      // Previous threshold for smoothing
    float prevAccel = 0;                 // Previous acceleration value for jerk calculation
    float prevJerk = 0;                  // Previous jerk value for comparison
    StepFeatures features;               // Current step features
    
    // Signal processing buffers
    float accelWindow[WINDOW_SIZE] = {0};    // Buffer for moving average filter
    float accelHistory[HISTORY_SIZE] = {0};  // History buffer for pattern analysis
    float featureHistory[PATTERN_SIZE][3];   // Store features of recent steps [peakStrength, jerkStrength, confidence]
    int windowIndex = 0;                     // Current index in moving average window
    int historyIndex = 0;                    // Current index in history buffer
    int patternIndex = 0;                    // Current index for pattern analysis
    
    // Step detection state variables
    bool inStep = false;                 // Flag for step detection state machine
    float localMin = 0, localMax = 0;    // For tracking peaks and valleys
    int stepCount = 0;                   // Total valid steps counted
    float stepLengths[PATTERN_SIZE];     // Individual step lengths
    float avgStepLength = DEFAULT_STEP_LENGTH; // Average step length in meters
    float totalDistance = 0;             // Estimated total distance traveled
    float cadence = 0;                   // Steps per minute
    float speed = 0;                     // Walking speed in m/s
    
    // Timer variables
    Timer stepTimer;                     // Measures time between steps
    Timer calibrationTimer;              // Times the calibration phase
    Timer displayTimer;                  // Controls display update frequency
    Timer bounceTimer;                   // For bounce (false step) detection
    
    // System state management
    PDR_Mode currentMode = CALIBRATING;  // Start in calibration mode
    bool bounceDetected = false;         // Flag for bounce detection
    bool stepDetected = false;           // Flag for step detection
    
    //===========================================================================
    // Initialization
    //===========================================================================
    
    // Initialize all timers
    stepTimer.start();
    calibrationTimer.start();
    displayTimer.start();
    bounceTimer.start();
    
    // Initialize step length array
    for (int i = 0; i < PATTERN_SIZE; i++) {
        stepLengths[i] = DEFAULT_STEP_LENGTH;
    }
    
    // Initialize feature history
    for (int i = 0; i < PATTERN_SIZE; i++) {
        for (int j = 0; j < 3; j++) {
            featureHistory[i][j] = 0;
        }
    }
    
    // Initialize display
    lcd.cls();
    lcd.locate(0, 0);
    lcd.printf("PDR System v3.0");
    lcd.locate(0, 10);
    lcd.printf("Calibrating...");
    
    // Check accelerometer connection
    if (!MMA.testConnection()) {
        // Handle connection error
        currentMode = ERROR;
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
    dynamicThreshold = baseline * 0.1;  // Initial threshold is 10% of baseline
    prevDynamicThreshold = dynamicThreshold;
    
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
        //=======================================================================
        // 1. Data Acquisition and Pre-processing
        //=======================================================================
        
        // Read accelerometer data for all axes
        ax = MMA.x();
        ay = MMA.y();
        az = MMA.z();
        
        // Calculate acceleration magnitude and remove baseline
        float accel = sqrt(ax*ax + ay*ay + az*az) - baseline;
        
        // Calculate jerk (rate of change of acceleration)
        // Assuming 50Hz sampling rate (0.02s between samples)
        float jerk = fabs(accel - prevAccel) / 0.02;
        
        // Update circular buffers for signal processing
        // Moving average window
        accelWindow[windowIndex] = accel;
        windowIndex = (windowIndex + 1) % WINDOW_SIZE;
        
        // History buffer for pattern analysis
        accelHistory[historyIndex] = accel;
        historyIndex = (historyIndex + 1) % HISTORY_SIZE;
        
        //=======================================================================
        // 2. Signal Processing
        //=======================================================================
        
        // Apply moving average filter to acceleration data
        float filteredAccel = 0;
        for (int i = 0; i < WINDOW_SIZE; i++) {
            filteredAccel += accelWindow[i];
        }
        filteredAccel /= WINDOW_SIZE;
        
        // Calculate short-term variance (over 10 samples)
        // This helps identify the variation in acceleration pattern
        float variance = 0;
        int varWindowStart = (historyIndex - 10 + HISTORY_SIZE) % HISTORY_SIZE;
        for (int i = 0; i < 10; i++) {
            int idx = (varWindowStart + i) % HISTORY_SIZE;
            variance += (accelHistory[idx] - filteredAccel) * (accelHistory[idx] - filteredAccel);
        }
        variance /= 10;
        
        // Update feature structure
        features.accelMagnitude = filteredAccel;
        features.jerk = jerk;
        features.accelVariance = variance;
        
        //=======================================================================
        // 3. Dynamic Threshold Adjustment
        //=======================================================================
        
        // Find min/max values in recent history
        float histMin = 100.0, histMax = -100.0;
        for (int i = 0; i < HISTORY_SIZE; i++) {
            histMin = min(histMin, accelHistory[i]);
            histMax = max(histMax, accelHistory[i]);
        }
        
        // Update dynamic threshold with exponential smoothing
        float newThreshold = 0.35 * (histMax - histMin);
        dynamicThreshold = (ADAPTIVE_ALPHA * newThreshold) + 
                           ((1 - ADAPTIVE_ALPHA) * prevDynamicThreshold);
        prevDynamicThreshold = dynamicThreshold;
        
        // Ensure threshold doesn't drop too low
        if (dynamicThreshold < 0.05) dynamicThreshold = 0.05;
        
        //=======================================================================
        // 4. Step Detection (State Machine)
        //=======================================================================
        
        stepDetected = false;  // Reset step detection flag
        
        if (!inStep) {
            // LOOKING FOR STEP START
            // Detect start of a potential step when:
            // 1. Acceleration exceeds threshold AND
            // 2. Jerk (acceleration change rate) is significant
            if (filteredAccel > dynamicThreshold && 
                jerk > (prevJerk * JERK_THRESHOLD_MULT)) {
                
                // Transition to in-step state
                inStep = true;
                localMax = filteredAccel;  // Initialize max value
                
                // Initial confidence based on timing
                float timeSinceLastStep = stepTimer.read();
                if (timeSinceLastStep >= MIN_STEP_INTERVAL && 
                    timeSinceLastStep <= MAX_STEP_INTERVAL) {
                    // Timing suggests a valid step pattern
                    features.stepConfidence = 0.7;
                } else {
                    // Unusual timing, lower initial confidence
                    features.stepConfidence = 0.3;
                }
            }
        } else {
            // IN STEP - TRACKING PEAK AND LOOKING FOR COMPLETION
            
            // Update peak if we find a higher value
            if (filteredAccel > localMax) {
                localMax = filteredAccel;
            } 
            // Detect step completion when:
            // 1. Acceleration drops significantly from peak (60% drop) AND
            // 2. Minimum time has passed since entering step state
            else if (filteredAccel < (localMax * 0.6) && 
                     stepTimer.read() > MIN_STEP_INTERVAL) {
                
                // Step cycle complete - we've found a step
                stepDetected = true;
                inStep = false;  // Exit step state
                
                // Calculate step confidence using multiple features
                float peakStrength = localMax / dynamicThreshold;  // How strong was the peak relative to threshold
                float jerkStrength = jerk / prevJerk;              // How sharp was the acceleration change
                float varStrength = variance / 0.01;               // How much variance in the signal (normalized)
                
                // Weighted confidence score (0-1)
                features.stepConfidence = (0.5 * peakStrength) + 
                                          (0.3 * jerkStrength) + 
                                          (0.2 * varStrength);
                
                // Store step features for pattern analysis
                featureHistory[patternIndex][0] = peakStrength;
                featureHistory[patternIndex][1] = jerkStrength;
                featureHistory[patternIndex][2] = features.stepConfidence;
                patternIndex = (patternIndex + 1) % PATTERN_SIZE;
                
                // Reset step timer for next step
                stepTimer.reset();
            }
        }
        
        //=======================================================================
        // 5. Step Validation and False Positive Rejection
        //=======================================================================
        
        // Bounce detection: Reject steps that occur too quickly after another
        // (false positives from device movement or signal noise)
        if (bounceTimer.read() < 0.1 && stepDetected) {
            bounceDetected = true;
            stepDetected = false;  // Invalidate the step
            errorLed = 1;  // Visual feedback for rejected step
        } else if (stepDetected) {
            // Valid timing - reset bounce detection
            bounceTimer.reset();
            bounceDetected = false;
            errorLed = 0;
        }
        
        //=======================================================================
        // 6. Metrics Calculation for Valid Steps
        //=======================================================================
        
        // Process valid steps (high confidence and passed bounce detection)
        if (stepDetected && features.stepConfidence > MIN_CONFIDENCE) {
            // Increment step counter
            stepCount++;
            
            // Calculate step length based on step features
            // More confident steps with stronger acceleration tend to be longer
            float stepMultiplier = min(max(features.stepConfidence * 0.5, 0.8), 1.2);
            stepLengths[patternIndex] = avgStepLength * stepMultiplier;
            
            // Update average step length using rolling average
            float sumLengths = 0;
            for (int i = 0; i < PATTERN_SIZE; i++) {
                sumLengths += stepLengths[i];
            }
            avgStepLength = sumLengths / PATTERN_SIZE;
            
            // Update total distance
            totalDistance += stepLengths[patternIndex];
            
            // Calculate walking cadence (steps per minute)
            cadence = 60.0 / stepTimer.read();
            
            // Calculate walking speed (meters per second)
            speed = (stepLengths[patternIndex] / stepTimer.read());
            
            // Visual feedback - toggle LED with each step
            stepLed = !stepLed;
            
            // Transition to ACTIVE mode if not already
            if (currentMode != ACTIVE) {
                currentMode = ACTIVE;
                modeLed = 0;  // Turn off ready LED in active mode
            }
        }
        
        //=======================================================================
        // 7. Display Update (periodic refresh)
        //=======================================================================
        
        if (displayTimer.read() > 0.5) {  // Update every 0.5 seconds
            displayTimer.reset();
            
            // Clear display for update
            lcd.cls();
            
            // Top line - mode and step count
            lcd.locate(0, 0);
            switch(currentMode) {
                case ACTIVE:
                    lcd.printf("Active - %d steps", stepCount);
                    break;
                case READY:
                    lcd.printf("Ready - Start walking");
                    break;
                case ERROR:
                    lcd.printf("Error - Check sensor");
                    break;
                default:
                    lcd.printf("PDR System v3.0");
            }
            
            // Middle line - distance and speed
            lcd.locate(0, 10);
            lcd.printf("%.1fm (%.1fm/s)", totalDistance, speed);
            
            // Bottom line - cadence and threshold
            lcd.locate(0, 20);
            lcd.printf("%.0f steps/min T:%.2f", cadence, dynamicThreshold);
        }
        
        //=======================================================================
        // 8. Store Values for Next Iteration
        //=======================================================================
        
        prevAccel = accel;
        prevJerk = jerk;
        
        // Small delay for consistent timing (approximately 50Hz sampling)
        wait(0.02);
    }
}

