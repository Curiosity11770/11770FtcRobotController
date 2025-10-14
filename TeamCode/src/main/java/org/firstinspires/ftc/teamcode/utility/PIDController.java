
package org.firstinspires.ftc.teamcode.utility;

import com.qualcomm.robotcore.util.Range;


public class PIDController {
    //PID Control Class
    /*

     * Proportional Integral Derivative Controller

     */
    double Kp, Ki, Kd;
    double lastError = 0;
    double integralSum = 0;
    double integralCap = 0.25;
    public double maxOut = 0.4;
    public double minOut = 0.1;
    double errorMargin = 1;
    double previousFilterEstimate = 0;
    double currentFilterEstimate = 0;
    double a = 0.5;
    public boolean targetReached;

    public PIDController(double kpIn, double kiIn, double kdIn, double maxOutIn){
        Kp = kpIn;
        Ki = kiIn;
        Kd = kdIn;
        maxOut = maxOutIn;
    }

    public double calculate(double reference, double currentPosition){

        // calculate the error
        double error = reference - currentPosition;

        //check if target is reached
        targetReached = Math.abs(error) < errorMargin;

        // rate of change of the error
        double errorChange = (error - lastError);

        //Low pass filter, reduces noise in signal
        currentFilterEstimate = (a*previousFilterEstimate)+(1-a)*errorChange;
        previousFilterEstimate = currentFilterEstimate;

        double derivative = currentFilterEstimate;

        // sum of all error over time
        integralSum = integralSum + (error);

        //anti-windup caps integralSum
        if(integralSum*Ki > integralCap){
            integralSum = integralCap/Ki;
        }else if(integralSum*Ki < -integralCap){
            integralSum = -integralCap/Ki;
        }

        double out = (Kp * error) + (Ki * integralSum) + (Kd * derivative);

        out = Range.clip(out, -maxOut, maxOut);

        lastError = error;

        return out;
    }

    public double calculate(double error){
        //check if target is reached
        targetReached = Math.abs(error) < errorMargin;

        // rate of change of the error
        double errorChange = (error - lastError);

        //Low pass filter, reduces noise in signal
        currentFilterEstimate = (a*previousFilterEstimate)+(1-a)*errorChange;
        previousFilterEstimate = currentFilterEstimate;

        double derivative = currentFilterEstimate;

        // sum of all error over time
        integralSum = integralSum + (error);

        //anti-windup caps integralSum
        if(integralSum*Ki > integralCap){
            integralSum = integralCap/Ki;
        }else if(integralSum*Ki < -integralCap){
            integralSum = -integralCap/Ki;
        }

        double out = (Kp * error) + (Ki * integralSum) + (Kd * derivative);

        out = Range.clip(out, -maxOut, maxOut);

        lastError = error;

        return out;
    }

    public boolean targetReached(double reference, double currentPosition){
        double error = reference-currentPosition;
        if(Math.abs(error) > errorMargin){
            return false;
        }else{
            return true;
        }
    }

    void reset(){
        integralSum = 0;
        previousFilterEstimate = 0;
        currentFilterEstimate = 0;
    }

    // Add a setter to allow updating gains
    public void setGains(double Kp, double Ki, double Kd, double maxOutput) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.maxOut = maxOutput;
    }
}
