// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Utils;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants.RobotStateConstants;

/** Custom DOF PID Control Solution */
public class PIDController {
    private double kP;
    private double kI;
    private double kD;
    private double tolerance = 0.05;
    private double positionError = 0;
    private double totalError = 0;
    private double velocityError = 0;
    private double prevError = 0;

    /**Sets PID Controller without kI or kD */
    public PIDController(double kP) {
        this.kP = kP;
        this.kI = 0.0;
        this.kD = 0.0;
    }

    /**Sets PID Controller without kD */
    public PIDController(double kP, double kI) {
        this.kP = kP;
        this.kI = kI;
        this.kD = 0.0;
    }

    /**Sets PID Controller */
    public PIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    /**Updates Position Tolerance Value */
    public void setTolerance(double tolerance){
        this.tolerance = tolerance;
    }

    /**Updates kP Value */
    public void setP(double kP){
        this.kP = kP;
    }

    /**Updates kI Value */
    public void setI(double kI){
        this.kI = kI;
    }

    /**Updates kD Value */
    public void setD(double kD){
        this.kD = kD;
    }

    /**Updates all PID Values */
    public void setPID(double kP, double kI, double kD){
        this.kP = kP;
    }

    /**Gets Tolerance Value */
    public double getTolerance(){
        return tolerance;
    }

    /**Gets kP Value */
    public double getP(){
        return kP;
    }

    /**Gets kI Value */
    public double getI(){
        return kI;
    }

    /**Gets kD Value */
    public double getD(){
        return kD;
    }

    /** Calculates Voltage Needed from Measurement, Setpoint, and Max Setpoint Value
     * PARAMETERS NEED THE SAME UNITS
     * @param measurement Current Measurement (Behavior or Location) of System 
     * @param setpoint The Ideal Behavior or Location of the System
     * @param maxValue The Max Value the Setpoint Could Ever Achieve
     * @return Returns Voltage*/ 
    public double calculateForVoltage(double measurement, double setpoint, double maxValue){
        positionError = setpoint - measurement;
        velocityError = (positionError -  prevError) / RobotStateConstants.LOOP_PERIODIC_SEC; //Velocity Error = Slope = Change in X/ Change in Y = (Where we are - Where we were)/Time Passed -> Derivative
        prevError = positionError;
        totalError = totalError + (positionError * RobotStateConstants.LOOP_PERIODIC_SEC); //Total Error = Area OVER Graph (Under Error Graph, Hence Intergral) = Total Error + (New Error * How Long we had that New Error)
        return 
        ((setpoint * 12) / maxValue) //raw movement value/voltage output
        //  + (Math.abs(setpoint - measurement) < tolerance ? TODO:Is this how we implement tolerance control? If not, how so?
         + (kP * positionError) //Calculates for Position Error 
         + (kI * totalError) //Calculates for Total Error
         + (kD * velocityError) //Calculates for Velocity Error
        //   : 0) TODO:Is this how we implement tolerance control? If not, how so? Idea: Keep Current/Working kP + kI + kD values until tolerance drops again
         ; 
    }

}
