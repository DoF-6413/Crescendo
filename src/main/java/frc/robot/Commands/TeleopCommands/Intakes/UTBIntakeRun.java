// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.TeleopCommands.Intakes;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Subsystems.feeder.Feeder;
import frc.robot.Subsystems.feeder.FeederConstants;
import frc.robot.Subsystems.utbintake.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class UTBIntakeRun extends ParallelCommandGroup {
  /** Creates a new AllIntake. */
  private double utbIntakePercentSpeed;

  private double feederRPM;

  /** Lowers OTB Intake and runs both Intakes to intake/outtake depending on isInwards */
  public UTBIntakeRun(UTBIntake utbIntake, Feeder feeder, boolean isInwards, boolean stop) {
    if (stop) {
      feederRPM = 0;
      utbIntakePercentSpeed = 0;
    } else if (isInwards) {
      feederRPM = FeederConstants.INTAKE_RPM;
      utbIntakePercentSpeed = UTBIntakeConstants.INTAKE_PERCENT_SPEED;
    } else {
      feederRPM = FeederConstants.OUTTAKE_RPM;
      utbIntakePercentSpeed = UTBIntakeConstants.OUTTAKE_PERCENT_SPEED;
    }

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new InstantCommand(() -> utbIntake.setPercentSpeed(utbIntakePercentSpeed), utbIntake),
        new InstantCommand(() -> feeder.setSetpoint(feederRPM), feeder));
  }
}
