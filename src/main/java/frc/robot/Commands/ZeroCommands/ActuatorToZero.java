// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.ZeroCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.actuator.Actuator;
import frc.robot.Subsystems.actuator.ActuatorConstants;

public class ActuatorToZero extends Command {
  /** Creates a new ZeroActuator. */
  private Actuator actuator;

  public ActuatorToZero(Actuator actuator) {
    this.actuator = actuator;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(actuator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    actuator.actuatorPIDEnable(false);
    actuator.setCurrentLimit(10);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    actuator.setActuatorPercentSpeed(-0.2);
    ;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    actuator.setActuatorPercentSpeed(0.0);
    actuator.setCurrentLimit(ActuatorConstants.CUR_LIM_A);
    actuator.actuatorPIDEnable(true);
    actuator.zeroPosition();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return actuator.getOutputCurrent() > 10;
  }
}
