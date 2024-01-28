// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.otbIntake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants.OTBIntakeConstants;

/** Runs the real life OTBIntake with CANSpark Speed Controllers and NEO motor */
public class OTBIntakeIOSparkMax implements OTBIntakeIO{
    private CANSparkMax OTBIntakeMotor;
    private RelativeEncoder OTBIntakeEncoder;

    /** Creates the motor and encoder for the OTB Intake */
    public OTBIntakeIOSparkMax(){
        System.out.println("[Init] Creating UTBIntakeIO");
        OTBIntakeMotor = new CANSparkMax(OTBIntakeConstants.OTB_INTAKE_CANID, MotorType.kBrushless);
        OTBIntakeEncoder = OTBIntakeMotor.getEncoder();
    }

    /** Updates the values for the OTB Intake */
    public void updateInputs(OTBIntakeIOInputs inputs) {
        inputs.rollerVelocityRadPerSec = 
            Units.rotationsToRadians(OTBIntakeEncoder.getPosition()) / OTBIntakeConstants.GEAR_RATIO; // Converts rotaions to Radians and then divides it by the gear ratio
        inputs.rollerAppliedVolts = 
            OTBIntakeMotor.getAppliedOutput()*OTBIntakeMotor.getBusVoltage();  // Applied voltage of the OTBIntake
        inputs.rollerCurrentAmps = 
            new double[] {OTBIntakeMotor.getOutputCurrent()};  // Amps used by intake
    }

    /** Sets the voltage of the OTB Intake motor
     * @param voltage 
     */
    public void setOTBIntakeVoltage(double voltage){
        OTBIntakeMotor.setVoltage(voltage);
    }

    /** Sets the OTB Intake to a percent of its max speed
     * @param percent
     */
    public void setOTBIntakePercentSpeed(double percent){
        OTBIntakeMotor.set(percent);
    }
}

