package frc.robot.core.utils;

import com.kauailabs.navx.frc.AHRS;

public class PID {
    private boolean isEnabled = false;

    private double P;
    private double I;
    private double D;

    private double error;
    private double prevError = 0;
    private double totalError = 0;
    private double errorRate;
    
    //stores the time (ms) of the last update
    private double prevTime = 0;

    private double period;

    // private double maxOutput;
    // private double minOutput;
    

    public double target;
    public PID (double kP, double kI, double kD, boolean isEnabled) {
        P = kP;
        I = kI;
        D = kD;
        this.isEnabled = isEnabled;


    }
    public PID (double kP, double kI, double kD) {
        this(kP, kI, kP, false);

    }


    public void setTarget(double target) {
        this.target = target;
    }
    public void enable() {
        isEnabled = true;
    }
     public void disable() {
        isEnabled = false;
    }
    public void reset() {
        error = 0;
        totalError = 0;
        errorRate = 0;
        prevError = 0;
        prevTime = 0;
    }

    public void update(double input) {
        
        period = System.currentTimeMillis() - prevTime;

        //Sets prevTime to the current time for the next call of update()
        prevTime = System.currentTimeMillis();
        
        //Updating the error
        error = target - input;

        //Finding the total error (integral of error)
        totalError += (error * period);
        
        //Finding the rate at which the error is changing (derivative of error)
        errorRate = (error - prevError) / period;

        //Same as above with error
        prevError = error;
    }

     public double calculate() {
        //Formula for PID
        //P * error + I * (integral of error) + D * (derivative of error)
        if (isEnabled) {
            return error * P + totalError * I + errorRate  * D;
        }
        else {
            return 0.0;
        }
    }

//"eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee" - Isaac Liu


}