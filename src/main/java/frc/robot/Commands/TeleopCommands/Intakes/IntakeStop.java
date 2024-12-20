// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.TeleopCommands.Intakes;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Subsystems.actuator.Actuator;
import frc.robot.Subsystems.actuator.ActuatorConstants;
import frc.robot.Subsystems.otbroller.OTBRoller;
import frc.robot.Subsystems.utbintake.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeStop extends ParallelCommandGroup {
  /** Raises OTB Intake and stops all intakes */
  public IntakeStop(Actuator actuator, OTBRoller otbRoller, UTBIntake utbIntake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addRequirements();
    addCommands(
        Commands.runOnce(() -> actuator.setSetpoint(ActuatorConstants.MAX_ANGLE_RADS), actuator),
        Commands.runOnce(() -> otbRoller.setPercentSpeed(0.0), otbRoller),
        Commands.runOnce(() -> utbIntake.setPercentSpeed(0.0), utbIntake));
  }
}
