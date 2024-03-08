// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.TeleopCommands.IntakesPosition;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Subsystems.actuator.*;
import frc.robot.Subsystems.otbIntake.*;
import frc.robot.Subsystems.utbintake.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AllIntakesOut extends ParallelCommandGroup {
  /** Creates a new AllIntake. */
  private double otbIntakeSpeed;

  private double utbIntakeSpeed;

  public AllIntakesOut(
      Actuator actuator, OTBIntake otbIntake, UTBIntake utbIntake, boolean isInwards) {
    if (isInwards) {
      otbIntakeSpeed = -0.50;
      utbIntakeSpeed = -1.0;
    } else {
      otbIntakeSpeed = 0.50;
      utbIntakeSpeed = -1.0;
    }
    addRequirements();

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        Commands.runOnce(
            () -> {
              actuator.setActuatorSetpoint(ActuatorConstants.MAX_ANGLE_RADS);
            },
            actuator),
        new InstantCommand(() -> otbIntake.setOTBIntakePercentSpeed(otbIntakeSpeed), otbIntake),
        new InstantCommand(() -> utbIntake.setUTBIntakePercentSpeed(utbIntakeSpeed), utbIntake));
  }
}
