// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotStateConstants;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {

  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  // private final ProfiledPIDController armPIDController;
  private final PIDController armPIDController;
  private SimpleMotorFeedforward armFeedforward;

  private static boolean isPIDEnabled = true;
  private static boolean isTestingEnabled = false;

  public Arm(ArmIO io) {
    System.out.println("[Init] Creating Arm");
    this.io = io;

    // Initalizing the Arm PID Contoller
    armPIDController = new PIDController(ArmConstants.KP, ArmConstants.KI, ArmConstants.KD);
    // armPIDController =
    //     new ProfiledPIDController(
    //         ArmConstants.KP,
    //         ArmConstants.KI,
    //         ArmConstants.KD,
    //         new TrapezoidProfile.Constraints(
    //             ArmConstants.MAX_VELOCITY, ArmConstants.MAX_ACCELERATION));
    // armPIDController.setGoal(ArmConstants.DEFAULT_POSITION_RAD);
    armPIDController.setSetpoint(ArmConstants.DEFAULT_POSITION_RAD);
    armPIDController.setTolerance(ArmConstants.ANGLE_TOLERANCE);
    armPIDController.disableContinuousInput();

    // Initalizing the Arm Feedforward Controller
    armFeedforward = new SimpleMotorFeedforward(ArmConstants.KS, ArmConstants.KV, ArmConstants.KA);

    // Puts the adjustable PID and FF values onto the SmartDashboard for the test mode
    SmartDashboard.putNumber("armkP", 1.0);
    SmartDashboard.putNumber("armkI", 0.0);
    SmartDashboard.putNumber("armkD", 0.0);
    SmartDashboard.putNumber("armkS", 0.0);
    SmartDashboard.putNumber("armkV", 0.0);
    SmartDashboard.putNumber("armkA", 0.0);
    SmartDashboard.putNumber("armMaxAcceleration", 0.0);
  }

  @Override
  public void periodic() {
    // updates the inputs
    this.updateInputs();
    // log the inputs
    Logger.processInputs("Arm", inputs);

    // Updates Arm Speed based on PID Control
    if (isPIDEnabled) {
      setPercentSpeed(
          armPIDController.calculate(inputs.armAbsolutePositionRad)
              + (armFeedforward.calculate(inputs.armVelocityRadPerSec)
                  / RobotStateConstants
                      .BATTERY_VOLTAGE)); // Feedforward divided by 12 since it returns a voltage
    }

    if (isTestingEnabled) {
      testPIDFValues();
    }
  }

  /** Updates the Outputs of the Motors based on What Mode we are In */
  public void updateInputs() {
    io.updateInputs(inputs);
  }

  /**
   * Sets the Arm motor to a percent of its maximum speed
   *
   * @param percent -1 to 1
   */
  public void setPercentSpeed(double percent) {
    io.setPercentSpeed(percent);
  }

  /**
   * Sets the voltage of the Arm motor
   *
   * @param volts -12 to 12
   */
  public void setMotorVoltage(double volts) {
    io.setMotorVoltage(volts);
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
   * Updates the angle that the arm should be at using the WPI PID controller
   *
   * @param goal Angle (Radians)
   */
  public void setGoal(double goal) {
    armPIDController.setSetpoint(goal);
    SmartDashboard.putNumber(
        "armSetpointDeg", Units.radiansToDegrees(armPIDController.getSetpoint()));
    // armPIDController.setGoal(goal);
  }

  /** Returns the goal/setpoint of the Arm PID contoller */
  public double getGoal() {
    return armPIDController.getSetpoint();
    // return armPIDController.getGoal().position;
  }

  /** Returns whether the arm is at its goal or not */
  public boolean atGoal() {
    return armPIDController.atSetpoint();
    // return armPIDController.atGoal();
  }

  /**
   * @return The current position of the Absolute Encoder on the Arm
   */
  public double getPositionDeg() {
    return inputs.armAbsolutePositionDeg;
  }

  /**
   * Changes the angle goal of the Arm
   *
   * @param increment Angle (Radians)
   */
  public void incrementArmGoal(double increment) {
    this.setGoal(armPIDController.getSetpoint() + increment);
    // armPIDController.setGoal(armPIDController.getGoal().position + increment);
  }

  /**
   * @param enabled True = Enable, False = Disable
   */
  public void enablePID(boolean enabled) {
    isPIDEnabled = enabled;
  }

  /**
   * @param enabled True = Enable, False = Disable
   */
  public void enableTesting(boolean enabled) {
    isTestingEnabled = enabled;
  }

  /** Updates the PID values for the Arm from ShuffleBoard */
  public void updatePIDController(double kp, double ki, double kd) {
    ArmConstants.KP = kp;
    ArmConstants.KI = ki;
    ArmConstants.KD = kd;
    armPIDController.setPID(ArmConstants.KP, ArmConstants.KI, ArmConstants.KD);
  }

  /** Updates the FF values from Shuffleboard */
  public void updateFFController(double ks, double kv, double ka) {
    ArmConstants.KS = ks;
    ArmConstants.KV = kv;
    ArmConstants.KA = ka;
    armFeedforward = new SimpleMotorFeedforward(ArmConstants.KS, ArmConstants.KV, ArmConstants.KA);
  }

  /** Updates the Trapezoidal Constraints from the ShuffleBoard */
  // public void updateTrapezoidalConstraints() {
  //   ArmConstants.MAX_VELOCITY = armMaxVelocity.getDouble(0.0);
  //   ArmConstants.MAX_ACCELERATION = armMaxAcceleration.getDouble(0.0);
  //   armPIDController.setConstraints(
  //       new TrapezoidProfile.Constraints(ArmConstants.MAX_VELOCITY,
  // ArmConstants.MAX_ACCELERATION));
  // }

  /** Updates PID and FF values from the SmartDashboard during testing mode */
  public void testPIDFValues() {
    if (ArmConstants.KP != SmartDashboard.getNumber("armkP", 1.0)
        || ArmConstants.KI != SmartDashboard.getNumber("armkI", 0.0)
        || ArmConstants.KD != SmartDashboard.getNumber("armkD", 0.0)) {
      updatePIDController(
          SmartDashboard.getNumber("armkP", 1.0),
          SmartDashboard.getNumber("armkI", 0.0),
          SmartDashboard.getNumber("armkD", 0.0));
    }
    if (ArmConstants.KS != SmartDashboard.getNumber("armkS", 0.0)
        || ArmConstants.KV != SmartDashboard.getNumber("armkV", 0.0)
        || ArmConstants.KA != SmartDashboard.getNumber("armkA", 0.0)) {
      updateFFController(
          SmartDashboard.getNumber("armkS", 0.0),
          SmartDashboard.getNumber("armkV", 0.0),
          SmartDashboard.getNumber("armkA", 0.0));
    }
    // if (ArmConstants.MAX_ACCELERATION != SmartDashboard.getNumber("armMaxAcceleration", 0.0)) {
    // updateTrapezoidalConstraints(SmartDashboard.getNumber("armMaxAcceleration", 0.0))
    // }
  }
}
