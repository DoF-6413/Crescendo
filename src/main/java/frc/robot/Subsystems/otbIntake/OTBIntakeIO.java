// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.otbIntake;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface OTBIntakeIO {

    @AutoLog
    public static class OTBIntakeIOInputs {
        // Actuator
        public double drivePositionRad = 0.0;
        public double driveVelocityRadPerSec = 0.0;
        public double driveAppliedVolts = 0.0;
        public double[] driveCurrentAmps = new double[] {};
        public double[] driveTempCelcius = new double[] {};

        // Rollers
        public double rollerMotorRPM = 0.0;
        public double rollerAppliedVolts = 0.0;
        public double[] rollerCurrentAmps = new double[] {};
        public double[] rollerTempCelcius = new double[] {};
    }

}
