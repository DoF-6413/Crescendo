// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.AutonomousCommands.PathPlannerCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.actuator.Actuator;
import frc.robot.Subsystems.actuator.ActuatorConstants;
import frc.robot.Subsystems.otbIntake.OTBIntake;
import frc.robot.Subsystems.utbintake.UTBIntake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PickUp extends SequentialCommandGroup {
  private double otbIntakePercentSpeed;
  private double utbIntakePercentSpeed;
  private double actuatorPosition;
  /** Creates a new pickUp. */
  public PickUp(Actuator actuator, OTBIntake otb, UTBIntake utb, boolean stop) {

    if (stop) {
      otbIntakePercentSpeed = 0;
      utbIntakePercentSpeed = 0;
      actuatorPosition = ActuatorConstants.MIN_ANGLE_RADS;
    } else {
      otbIntakePercentSpeed = -0.20;
      utbIntakePercentSpeed = -1.0;
      actuatorPosition = ActuatorConstants.MAX_ANGLE_RADS;
    }

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ParallelCommandGroup(
            new InstantCommand(() -> actuator.setSetpoint(actuatorPosition)),
            new InstantCommand(() -> otb.setPercentSpeed(otbIntakePercentSpeed)),
            new InstantCommand(() -> utb.setUTBIntakePercentSpeed(utbIntakePercentSpeed))));
  }
}
