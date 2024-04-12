// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotStateConstants;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  // Creates the PID & FF Contollers for both shooter motors
  private final PIDController topShooterPIDController;
  private final PIDController bottomShooterPIDController;
  private SimpleMotorFeedforward topShooterFeedforward;
  private SimpleMotorFeedforward bottomShooterFeedforward;

  // The desired RPM for the shooter
  private double setpointRPM = 0.0;

  private static boolean isPIDEnabled = true;
  private static boolean isTestingEnabled = false;

  public Shooter(ShooterIO io) {
    System.out.println("[Init] Creating Shooter");
    this.io = io;

    // Initializes the Shooter PID Contollers
    topShooterPIDController =
        new PIDController(ShooterConstants.KP, ShooterConstants.KI, ShooterConstants.KD);
    bottomShooterPIDController =
        new PIDController(ShooterConstants.KP, ShooterConstants.KI, ShooterConstants.KD);
    topShooterPIDController.setSetpoint(setpointRPM);
    bottomShooterPIDController.setSetpoint(setpointRPM);
    // Sets the tolerance of the setpoint
    topShooterPIDController.setTolerance(ShooterConstants.RPM_TOLERANCE);
    bottomShooterPIDController.setTolerance(ShooterConstants.RPM_TOLERANCE);

    // Initalizes the Shooter FF Contollers
    topShooterFeedforward =
        new SimpleMotorFeedforward(ShooterConstants.KS, ShooterConstants.KV, ShooterConstants.KA);
    bottomShooterFeedforward =
        new SimpleMotorFeedforward(ShooterConstants.KS, ShooterConstants.KV, ShooterConstants.KA);
    // topShooterFeedforward.maxAchievableAcceleration(
    //     RobotStateConstants.BATTERY_VOLTAGE, inputs.topShooterMotorRPM);
    // bottomShooterFeedforward.maxAchievableAcceleration(
    //     RobotStateConstants.BATTERY_VOLTAGE, inputs.bottomShooterMotorRPM);

    // Puts adjustable PID and FF values onto the SmartDashboard for testing mode
    SmartDashboard.putNumber("shooterkP", 0.0025);
    SmartDashboard.putNumber("shooterkI", 0.0);
    SmartDashboard.putNumber("shooterkD", 0.00002);
    SmartDashboard.putNumber("shooterkS", 0.0);
    SmartDashboard.putNumber("shooterkV", 0.0225);
    SmartDashboard.putNumber("shooterkA", 0.8);
  }

  @Override
  public void periodic() {
    this.updateInputs();
    Logger.processInputs("Shooter", inputs);

    if (isPIDEnabled) {
      // Sets the voltage of the Shooter Motors using PID
      setTopVoltage(
          topShooterPIDController.calculate(inputs.topShooterMotorRPM)
              + (topShooterFeedforward.calculate(inputs.topShooterMotorRPM)
                  / RobotStateConstants.BATTERY_VOLTAGE));
      setBottomVoltage(
          bottomShooterPIDController.calculate(inputs.bottomShooterMotorRPM)
              + (bottomShooterFeedforward.calculate(inputs.bottomShooterMotorRPM)
                  / RobotStateConstants.BATTERY_VOLTAGE));
    }

    SmartDashboard.putNumber("shooterSetpoint", topShooterPIDController.getSetpoint());
    SmartDashboard.putBoolean("BothAtSetpoint", bothAtSetpoint());

    if (isTestingEnabled) {
      testPIDFFValues();
    }
  }

  /** Updates the set of loggable inputs for both Shooter Motors */
  public void updateInputs() {
    io.updateInputs(inputs);
  }

  /**
   * Sets the Brake Mode for the Shooter (Brake means motor holds position, Coast means easy to
   * move)
   *
   * @param enable if enable, it sets brake mode, else it sets coast mode
   */
  public void setBrakeMode(boolean enable) {
    io.setBrakeMode(enable);
  }

  /**
   * Sets BOTH Shooter Motors at a percentage of its max speed.
   *
   * <p>A positve number spins the Top Shooter Motor CCW and the Bottom Shooter Motor CW and vice
   * versa for a negative number
   *
   * @param percent -1 to 1
   */
  public void setBothPercentSpeed(double percent) {
    io.setBothPercentSpeed(percent);
  }

  /**
   * Sets BOTH Shooter Motors at a percentage of its max speed.
   *
   * <p>A positve number spins the Top Shooter Motor CCW and CCW for a negative number
   *
   * @param percent -1 to 1
   */
  public void setTopPercentSpeed(double percent) {
    io.setTopPercentSpeed(percent);
  }

  /**
   * Sets BOTH Shooter Motors at a percentage of its max speed.
   *
   * <p>A positve number spins the Motor CW and CCW for a negative number
   *
   * @param percent -1 to 1
   */
  public void setBottomPercentSpeed(double percent) {
    io.setBottomPercentSpeed(percent);
  }

  /**
   * Sets BOTH Shooter Motors at the specified Voltage
   *
   * @param volts -12 to 12
   */
  public void setBothsVoltage(double volts) {
    io.setBothVoltage(volts);
  }

  /**
   * Sets the voltage of the Top Shooter Motor
   *
   * @param volts -12 to 12
   */
  public void setTopVoltage(double volts) {
    io.setTopVoltage(volts);
  }

  /**
   * Sets the voltage of the Bottom Shooter Motor
   *
   * @param volts -12 to 12
   */
  public void setBottomVoltage(double volts) {
    io.setBottomVoltage(volts);
  }

  /** Returns whether BOTH Shooter motors are at their setpoint */
  public boolean bothAtSetpoint() {
    return bottomShooterPIDController.atSetpoint() && topShooterPIDController.atSetpoint();
  }

  /**
   * Sets the PID setpoint of the Shooter
   *
   * @param setpoint RPM
   */
  public void setSetpoint(double setpoint) {
    topShooterPIDController.setSetpoint(setpoint);
    bottomShooterPIDController.setSetpoint(setpoint);
  }

  /**
   * Sets the range the RPM of the Shooter motors can be within the setpoint
   *
   * @param tolerance RPM
   */
  public void setTolerance(double tolerance) {
    topShooterPIDController.setTolerance(tolerance);
    bottomShooterPIDController.setTolerance(tolerance);
  }

  public double getSetpoint() {
    return topShooterPIDController.getSetpoint();
  }

  public double returnDesiredAngle(double x) {
    double closestX, closestTheta;
    if (ShooterConstants.LOOKUP_TABLE_X_M_VS_THETA_DEG[0][5] > x) {
      int i = 1;
      closestX = ShooterConstants.LOOKUP_TABLE_X_M_VS_THETA_DEG[0][i];
      closestTheta = ShooterConstants.LOOKUP_TABLE_X_M_VS_THETA_DEG[1][i];

      // do closest theta is 0 if you are 5 m away of the speaker(you cant
      // shoot)//TODO:when we change the max change the 5 to the new max

      // since the table is sorted, find the index of the first value where the distance value
      // exceeds
      while (ShooterConstants.LOOKUP_TABLE_X_M_VS_THETA_DEG[0][i] < x) {
        i++;
      }

      // finds which x value in the table the actual distance is closer to and return that
      if (Math.abs(ShooterConstants.LOOKUP_TABLE_X_M_VS_THETA_DEG[0][i - 1] - x)
          < Math.abs(ShooterConstants.LOOKUP_TABLE_X_M_VS_THETA_DEG[0][i] - x)) {
        closestX = ShooterConstants.LOOKUP_TABLE_X_M_VS_THETA_DEG[0][i - 1];
        closestTheta =
            (ShooterConstants.LOOKUP_TABLE_X_M_VS_THETA_DEG[1][i - 1]
                    + ShooterConstants.LOOKUP_TABLE_X_M_VS_THETA_DEG[2][i - 1])
                / 2;
      } else {
        closestX = ShooterConstants.LOOKUP_TABLE_X_M_VS_THETA_DEG[0][i];
        closestTheta =
            (ShooterConstants.LOOKUP_TABLE_X_M_VS_THETA_DEG[1][i]
                    + ShooterConstants.LOOKUP_TABLE_X_M_VS_THETA_DEG[2][i])
                / 2;
      }

      // returns the closest Theta based on the lookup table
      return closestTheta;
    } else {
      closestTheta = 0;
      return closestTheta;
    }
  }

  /**
   * @param enable True = Enable, False = Disable
   */
  public void enablePID(boolean enable) {
    isPIDEnabled = enable;
  }

  /**
   * @param enable True = Enable, False = Disable
   */
  public void enableTesting(boolean enable) {
    isTestingEnabled = enable;
  }

  /** Updates the PID values for the Shooter from ShuffleBoard */
  public void updatePIDController(double kp, double ki, double kd) {
    ShooterConstants.KP = kp;
    ShooterConstants.KI = ki;
    ShooterConstants.KD = kd;
    topShooterPIDController.setPID(ShooterConstants.KP, ShooterConstants.KI, ShooterConstants.KD);
    bottomShooterPIDController.setPID(
        ShooterConstants.KP, ShooterConstants.KI, ShooterConstants.KD);
  }

  /** Updates the Feedforward values for the Shooter from ShuffleBoard */
  public void updateFFController(double ks, double kv, double ka) {
    ShooterConstants.KS = ks;
    ShooterConstants.KV = kv;
    ShooterConstants.KA = ka;
    topShooterFeedforward =
        new SimpleMotorFeedforward(ShooterConstants.KS, ShooterConstants.KV, ShooterConstants.KA);
    bottomShooterFeedforward =
        new SimpleMotorFeedforward(ShooterConstants.KS, ShooterConstants.KV, ShooterConstants.KA);
  }

  public void testPIDFFValues() {
    if (ShooterConstants.KP != SmartDashboard.getNumber("shooterkP", 0.0025)
        || ShooterConstants.KI != SmartDashboard.getNumber("shooterkI", 0.0)
        || ShooterConstants.KD != SmartDashboard.getNumber("shooterkD", 0.00002)) {
      updatePIDController(
          SmartDashboard.getNumber("shooterkP", 0.0025),
          SmartDashboard.getNumber("shooterkI", 0.0),
          SmartDashboard.getNumber("shooterkD", 0.00002));
    }

    if (ShooterConstants.KS != SmartDashboard.getNumber("shooterkS", 0.0)
        || ShooterConstants.KV != SmartDashboard.getNumber("shooterkV", 0.0225)
        || ShooterConstants.KA != SmartDashboard.getNumber("shooterkA", 0.8)) {
      updateFFController(
          SmartDashboard.getNumber("shooterkS", 0.0),
          SmartDashboard.getNumber("shooterkV", 0.0225),
          SmartDashboard.getNumber("shooterkA", 0.8));
    }
  }
}
