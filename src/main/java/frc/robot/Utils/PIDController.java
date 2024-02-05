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
    private double tolerance = 0.05; //percentage
    private double positionError = 0;
    private double totalError = 0;
    private double velocityError = 0;
    private double prevError = 0;
    private double setpoint = 0;
    private boolean atSetpoint = false;

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

    /**Updates Tolerance As A Decimal Percentage */
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
        this.kI = kI;
        this.kD = kD;
    }

    /**Gets Tolerance As A Decimal Percentage */
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

    /**Gets all PID values in an array [kP, kI, kD] */
    public double[] getPID(){
        double[] pid = {kP, kI, kD};
        return pid;
    }

    /** Calculates voltage from measurement, setpoint, and max setpoint value,
     * PARAMETERS NEED THE SAME UNITS
     * @param measurement Current Measurement (Behavior or Location) of System 
     * @param setpoint The Ideal Behavior or Location of the System
     * @param maxValue The Max Value the Setpoint Could Ever Achieve
     * @return Returns Voltage*/ 
    public double calculateForVoltage(double measurement, double setpoint, double maxValue){
        //updates setpoint so accessible within class
        this.setpoint = setpoint;

        //PROPORTIONAL: Current error
        positionError = setpoint - measurement;

        //DERIVATIVE: Velocity error = change in position / time 
        velocityError = (positionError -  prevError) / RobotStateConstants.LOOP_PERIODIC_SEC; 

        //update previous error
        prevError = positionError;

        //INTEGRAL: Total Error = Area Under Graph = position error * time, accumulated for each periodic loop 
        totalError = totalError + (positionError * RobotStateConstants.LOOP_PERIODIC_SEC); 

        // raw voltage output + PID tuning = calculated voltage
        double desiredVoltage =  ((setpoint * RobotStateConstants.BATTERY_VOLTAGE) / maxValue) 
        + (kP * positionError)
        + (kI * totalError)
        + (kD * velocityError); 
        
        //  + (Math.abs(setpoint - measurement) < tolerance ? TODO:Is this how we implement tolerance control? If not, how so?
        //   : 0) TODO:Is this how we implement tolerance control? If not, how so? Idea: Keep Current/Working kP + kI + kD values until tolerance drops again

        //updates atSetpoint
        atSetpoint = atSetpoint(desiredVoltage); 

        return desiredVoltage;
    }

    /** TODO: my attempt at tolerance. To my understanding as long as the calculated output is within "tolerance" value of 
     * the setpoint, they should return a boolean, and then use the boolean to test other stuff*/
    public boolean atSetpoint(double voltage) {
        if ((voltage >= setpoint - tolerance) && (voltage <= setpoint + tolerance)) {
            return true;
        }
        return false;
    }
}
