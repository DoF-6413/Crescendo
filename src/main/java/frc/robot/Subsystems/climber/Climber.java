// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {

  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  /** Creates a new Climber. */
  public Climber(ClimberIO io) {
    System.out.println("[Init] Creating Climber");
    this.io = io;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    this.updateInputs();
    Logger.processInputs("Climber", inputs);
  }

  /** Updates the set of loggable inputs for the Climber */
  public void updateInputs() {
    io.updateInputs(inputs);
  }

  /**
   * Sets the voltage of the Climber motor
   *
   * @param volts [-12 to 12]
   */
  public void setClimberVoltage(double volts) {
    io.setClimberVoltage(volts);
  }

  /**
   * Sets the Climber motor to a percentage of its maximum speed
   *
   * @param percent [-1 to 1]
   */
  public void setClimberPercentSpeed(double percent) {
    io.setClimberPercentSpeed(percent);
  }

  /**
   * Sets the Brake Mode for the Climber (Brake means motor holds position, Coast means easy to
   * move)
   *
   * @param enable if enable, it sets brake mode, else it sets coast mode
   */
  public void setClimberBrakeMode(Boolean enable) {
    io.setClimberBrakeMode(enable);
  }
}
