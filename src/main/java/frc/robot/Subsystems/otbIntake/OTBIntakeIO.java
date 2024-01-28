// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.otbIntake;

import org.littletonrobotics.junction.AutoLog;

/** These are all the input/output values that you can log for OTB Intake */
public interface OTBIntakeIO {

    @AutoLog
    public static class OTBIntakeIOInputs {
        // Rollers
        public double rollerVelocityRadPerSec = 0.0;
        public double rollerAppliedVolts = 0.0;
        public double[] rollerCurrentAmps = new double[] {};
    }

    public default void updateInputs(OTBIntakeIOInputs inputs){}

    public default void setOTBIntakeVoltage(double volts) {}

    public default void setOTBIntakePercentSpeed(double percent){}

}
