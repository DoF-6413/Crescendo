// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.wrist;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.RobotStateConstants;
import frc.robot.Constants.ShootingInterpolationConstants;
import org.littletonrobotics.junction.Logger;

public class Wrist extends SubsystemBase {
  private final WristIO io;
  private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();
  private final ProfiledPIDController wristPIDController;
  private SimpleMotorFeedforward wristFeedforward;
  private double speedScalar = 1;

  /** Used to toggle PID calculations for the angle */
  private static boolean isPIDEnabled = true;

  /** Used to toggle test features */
  private static boolean isTestingEnabled = false;

  /** Creates a new Wrist, the second joint of the entire Arm mechanism */
  public Wrist(WristIO io) {
    System.out.println("[Init] Creating Wrist");
    this.io = io;

    /** Creates a new Profiled PID controller for the Wrist */
    wristPIDController =
        new ProfiledPIDController(
            WristConstants.KP,
            WristConstants.KI,
            WristConstants.KD,
            new TrapezoidProfile.Constraints(
                WristConstants.MAX_VELOCITY, WristConstants.MAX_ACCELERATION));
    wristPIDController.setGoal(WristConstants.DEFAULT_POSITION_RAD);
    wristPIDController.setTolerance(WristConstants.ANGLE_TOLERANCE);
    wristPIDController.disableContinuousInput();

    /** Creates a new Feedforward Contoller for the Wrist */
    wristFeedforward =
        new SimpleMotorFeedforward(WristConstants.KS, WristConstants.KV, WristConstants.KA);

    /** Developer tools */
    // Puts adjustable PPID and FF values onto the SmartDashboard for the testing mode
    SmartDashboard.putNumber("wristkP", 0.8);
    SmartDashboard.putNumber("wristkI", 0.0);
    SmartDashboard.putNumber("wristkD", 0.01);
    SmartDashboard.putNumber("wristkS", 0.2);
    SmartDashboard.putNumber("wristkV", 0.0001);
    SmartDashboard.putNumber("wristkA", 0.0);
    SmartDashboard.putNumber("wristMaxAcceleration", 0.0);
  }

  @Override
  public void periodic() {
    /** Periodically updates inputs and logs them */
    this.updateInputs();
    Logger.processInputs("Wrist", inputs);

    if (isPIDEnabled) {
      setPercentSpeed(
          this.speedScalar
              * (wristPIDController.calculate(inputs.wristAbsolutePositionRad)
                  + (wristFeedforward.calculate(inputs.wristVelocityRadPerSec)
                      / RobotStateConstants
                          .BATTERY_VOLTAGE))); // Feedforward divided by 12 since it returns a
      // voltage
    }

    if (isTestingEnabled) {
      testPIDFValues();
    }
  }

  /** Updates the set of loggable inputs for the Wrist */
  public void updateInputs() {
    io.updateInputs(inputs);
  }

  /**
   * Sets Wrist to a percentage of its maximum speed
   *
   * @param percent -1 to 1
   */
  public void setPercentSpeed(double percent) {
    io.setPercentSpeed(percent);
  }

  /**
   * Sets voltage of the Wrist
   *
   * @param volts -12 to 12
   */
  public void setVoltage(double volts) {
    io.setVoltage(volts);
  }

  /**
   * Sets brake mode of the Wrist
   *
   * @param enable Sets brake mode if true, coast if false
   */
  public void setBrakeMode(boolean enable) {
    io.setBrakeMode(enable);
  }

  /**
   * Updates the angle that the Wrist should be at using the WPI PID controller
   *
   * @param goal Angle (Radians)
   */
  public void setGoal(double goal) {
    wristPIDController.setGoal(goal);
    SmartDashboard.putNumber(
        "WristSetpointDeg", Units.radiansToDegrees(wristPIDController.getGoal().position));
  }

  /** Returns the angle setpoint of the Wrist */
  public double getGoal() {
    return wristPIDController.getGoal().position;
  }

  /** Returns whether the Wrist is at it's angle goal or not */
  public boolean atGoal() {
    return wristPIDController.atSetpoint();
  }

  /**
   * @return The current position of the Absolute Encoder on the Wrist
   */
  public double getPositionDeg() {
    return inputs.wristAbsolutePositionDeg;
  }

  /**
   * Sets the value of the speed multiplier for the Wrist
   *
   * @param scalar Multiplier value of the Wrist speed
   */
  public void setSpeedScalar(double scalar) {
    this.speedScalar = scalar;
  }

  /**
   * Changes the angle goal of the Wrist
   *
   * @param increment Angle (Radians)
   */
  public void incrementWristGoal(double increment) {
    this.setGoal(wristPIDController.getGoal().position + increment);
  }

  /**
   * Calculates the angle of the Wrist based on the robot's distance away from the SPEAKER
   *
   * <p>Data collected to create the line of best fit equation used to calculate Wrist angle can be
   * found at {@link ShootingInterpolationConstants}
   *
   * @param robotPose Robot's current pose
   */
  public void autoAlignWrist(Pose2d robotPose) {
    // X Distance from the center of the SPEAKER
    double deltaX = 0.0;
    if (RobotStateConstants.getAlliance().get() == Alliance.Red) {
      deltaX = Math.abs(robotPose.getX() - FieldConstants.RED_SPEAKER_X);
    } else if (RobotStateConstants.getAlliance().get() == Alliance.Blue) {
      deltaX = Math.abs(robotPose.getX() - FieldConstants.BLUE_SPEAKER_X);
    }
    // Y Distance from the center of the SPEAKER
    double deltaY = Math.abs(robotPose.getY() - FieldConstants.SPEAKER_Y);
    // Hypotenuse of the x and y components to find the actual distance from the center of the
    // SPEAKER
    double speakerDist = Math.hypot(deltaX, deltaY);

    // Defaults angle to 0 degrees if robot's distance is outside the range of tested values in the
    // lookup table
    if (speakerDist
        > ShootingInterpolationConstants.LOOKUP_TABLE_X_M_VS_THETA_DEG[0][
            ShootingInterpolationConstants.LOOKUP_TABLE_X_M_VS_THETA_DEG[0].length - 1]) {
      this.setGoal(0);
    } else {
      this.setGoal(Units.degreesToRadians(-9.37857 * speakerDist + 37.1616));
    }
  }

  /**
   * Toggles whether the PID controller is used to for setting the voltage of the Wrist based on an
   * angle setpoint
   *
   * @param enabled True = Enable, False = Disable
   */
  public void enablePID(boolean enabled) {
    isPIDEnabled = enabled;
  }

  /**
   * Toggles whether the PID values are used from Constants or SmartDashboard inputs
   *
   * @param enabled True = Enable, False = Disable
   */
  public void enableTesting(boolean enabled) {
    isTestingEnabled = enabled;
  }

  /** Updates the PID values from SmartDashboard */
  public void updatePIDController(double kp, double ki, double kd) {
    WristConstants.KP = kp;
    WristConstants.KI = ki;
    WristConstants.KD = kd;
    wristPIDController.setPID(WristConstants.KP, WristConstants.KI, WristConstants.KD);
  }

  /** Updates the Trapezoidal Constraints from the SmartDashboard */
  public void updateTrapezoidalConstraints(double maxAcc) {
    WristConstants.MAX_ACCELERATION = maxAcc;
    wristPIDController.setConstraints(
        new TrapezoidProfile.Constraints(
            WristConstants.MAX_VELOCITY, WristConstants.MAX_ACCELERATION));
  }

  /** Updates the PID values from SmartDashboard */
  public void updateFFController(double ks, double kv, double ka) {
    WristConstants.KS = ks;
    WristConstants.KV = kv;
    WristConstants.KA = ka;
    wristFeedforward =
        new SimpleMotorFeedforward(WristConstants.KS, WristConstants.KV, WristConstants.KA);
  }

  /** Updates PPID and FF values from the SmartDashboard during testing mode */
  public void testPIDFValues() {
    if (WristConstants.KP != SmartDashboard.getNumber("wristkP", 0.8)
        || WristConstants.KI != SmartDashboard.getNumber("wristkI", 0.0)
        || WristConstants.KD != SmartDashboard.getNumber("wristkD", 0.01)) {
      updatePIDController(
          SmartDashboard.getNumber("wristkP", 0.8),
          SmartDashboard.getNumber("wristkI", 0.0),
          SmartDashboard.getNumber("wristkD", 0.01));
    }
    if (WristConstants.KS != SmartDashboard.getNumber("wristkS", 0.2)
        || WristConstants.KV != SmartDashboard.getNumber("wristkV", 0.0001)
        || WristConstants.KA != SmartDashboard.getNumber("wristkA", 0.0)) {
      updateFFController(
          SmartDashboard.getNumber("wristkS", 0.2),
          SmartDashboard.getNumber("wristkV", 0.0001),
          SmartDashboard.getNumber("wristkA", 0.0));
    }
    if (WristConstants.MAX_ACCELERATION != SmartDashboard.getNumber("wristMaxAcceleration", 0.0)) {
      updateTrapezoidalConstraints(SmartDashboard.getNumber("armMaxAcceleration", 0.0));
    }
  }
}
