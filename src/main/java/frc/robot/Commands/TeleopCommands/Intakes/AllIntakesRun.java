// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.TeleopCommands.Intakes;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Subsystems.actuator.*;
import frc.robot.Subsystems.feeder.Feeder;
import frc.robot.Subsystems.otbIntake.*;
import frc.robot.Subsystems.utbintake.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AllIntakesRun extends ParallelCommandGroup {
  private double otbIntakePercentSpeed;
  private double utbIntakePercentSpeed;
  private double feederRPM;
  private double actuatorPosition;

  /** Lowers OTB Intake and runs both Intakes to intake/outtake depending on isInwards */
  public AllIntakesRun(
      Actuator actuator, OTBIntake otbIntake, UTBIntake utbIntake, Feeder feeder, boolean stop) {

    if (stop) {
      otbIntakePercentSpeed = 0;
      utbIntakePercentSpeed = 0;
      feederRPM = 0;
      actuatorPosition = ActuatorConstants.MIN_ANGLE_RADS;
    } else {
      otbIntakePercentSpeed = -0.30;
      utbIntakePercentSpeed = -1.0;

      feederRPM = 1500;
      actuatorPosition = ActuatorConstants.MAX_ANGLE_RADS;
    }

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        Commands.runOnce(
            () -> {
              actuator.setSetpoint(actuatorPosition);
            },
            actuator),
        new InstantCommand(() -> otbIntake.setPercentSpeed(otbIntakePercentSpeed), otbIntake),
        new InstantCommand(
            () -> utbIntake.setUTBIntakePercentSpeed(utbIntakePercentSpeed), utbIntake),
        new InstantCommand(() -> feeder.setSetpoint(feederRPM), feeder));
  }
}
