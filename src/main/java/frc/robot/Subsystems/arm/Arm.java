// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {

  private final ArmIO io;
  private final ArmIOInputsAutoLogged armInputs = new ArmIOInputsAutoLogged();
  private final PIDController armPIDController;
  private final ShuffleboardTab armTab = Shuffleboard.getTab("Arm");

  /** Creates a new Arm, the Subsystem that moves the Shooter from Up and Down */
  public Arm(ArmIO io) {
    System.out.println("[Init] Creating arm");
    this.io = io;
    armPIDController = new PIDController(ArmConstants.KP, ArmConstants.KI, ArmConstants.KD);
    armPIDController.setSetpoint(0);
    armPIDController.setTolerance(Units.degreesToRadians(1));
    armPIDController.disableContinuousInput();

    // TODO: Delete once final PID Numbers are Decided
    // armkp = armTab.add("armkp", 0.0).getEntry();
    // armki = armTab.add("armki", 0.0).getEntry();
    // armkd = armTab.add("armkd", 0.0).getEntry();
    // armSetpointSetter = armTab.add("armSetpoint", 0.0).getEntry();
  }

  
  @Override
  public void periodic() {
    // updates the inputs
    this.updateInputs();
    // log the inputs
    Logger.processInputs("Arm", armInputs);

    // Updates Arm Speed based on PID Control
    setArmPercentSpeed(armPIDController.calculate(armInputs.armPositionRad));

    // // TODO: Delete once final PID Numbers are Decided
    // if (ArmConstants.KP != armkp.getDouble(0.0)
    //     || ArmConstants.KI != armki.getDouble(0.0)
    //     || ArmConstants.KD != armkd.getDouble(0.0)) {
    //   updatePIDController();
    // }

    // if (armSetpoint != armSetpointSetter.getDouble(0.0)) {
    //   updateSetpoint();
    // }
  }

  /** Updates the Outputs of the Motors based on What Mode we are In */
  public void updateInputs() {
    io.updateInputs(armInputs);
  }

  /**
   * Sets the Arm motor to a percent of its maximum speed
   *
   * @param percent [-1 to 1]
   */
  public void setArmPercentSpeed(double percent) {
    io.setArmPercentSpeed(percent);
  }
  /**
   * Sets the voltage of the Arm motor
   *
   * @param volts [-12 to 12]
   */
  public void setArmMotorVoltage(double volts) {

    io.setArmVoltage(volts);
  }

  /**
   * Sets the Brake Mode for the Arm (Brake means motor holds position, Coast means easy to move)
   *
   * @param enable if enable, it sets brake mode, else it sets coast mode
   */
  public void setBrakeMode(boolean enable) {
    io.setBrakeMode(enable);
  }

  /**
   * Updates the angle that the wrist should be at using the WPI PID controller
   *
   * @param setpoint Radians [RANGE]
   */
  public void setSetpoint(double setpoint) {
    armPIDController.setSetpoint(setpoint);
  }

  /** Returns whether the arm is at its setpoint or not */
  public boolean atSetpoint() {
    return armPIDController.atSetpoint();
  }
}
