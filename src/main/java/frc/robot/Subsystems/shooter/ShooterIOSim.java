package frc.robot.Subsystems.shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants.RobotStateConstants;
import frc.robot.Utils.PIDController;

public class ShooterIOSim implements ShooterIO {
  // Creating flywheels
  private final FlywheelSim topShooterFlywheel =
      new FlywheelSim(
          DCMotor.getFalcon500(1), ShooterConstants.GEAR_RATIO, ShooterConstants.MOI_KG_M2);

  private final FlywheelSim bottomShooterFlywheel =
      new FlywheelSim(
          DCMotor.getFalcon500(1), ShooterConstants.GEAR_RATIO, ShooterConstants.MOI_KG_M2);

  private PIDController topShooterPID;
  private PIDController bottomShooterPID;

  public ShooterIOSim() {
    System.out.println("[Init] Creating ShooterIOSim");
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    // Updates the Shooter motors periodically
    topShooterFlywheel.update(RobotStateConstants.LOOP_PERIODIC_SEC);
    bottomShooterFlywheel.update(RobotStateConstants.LOOP_PERIODIC_SEC);

    // Updates logged inputs of the simulated Shooter Flywheels
    inputs.topShooterMotorRPM = topShooterFlywheel.getAngularVelocityRPM();
    inputs.topShooterAppliedVolts = 0.0;
    inputs.topShooterCurrentAmps = new double[] {Math.abs(topShooterFlywheel.getCurrentDrawAmps())};

    inputs.bottomShooterMotorRPM = bottomShooterFlywheel.getAngularVelocityRPM();
    inputs.bottomShooterAppliedVolts = 0.0;
    inputs.bottomShooterCurrentAmps =
        new double[] {Math.abs(bottomShooterFlywheel.getCurrentDrawAmps())};
  }

  @Override
  public void setBothShooterMotorPercentSpeed(double percent) {
    topShooterFlywheel.setInputVoltage(RobotStateConstants.BATTERY_VOLTAGE * percent);
    bottomShooterFlywheel.setInputVoltage(RobotStateConstants.BATTERY_VOLTAGE * percent);
  }

  @Override
  public void setBothShooterMotorsVoltage(double volts) {
    topShooterFlywheel.setInputVoltage(volts);
    bottomShooterFlywheel.setInputVoltage(volts);
  }

  @Override
  public void setTopShooterMotorVoltage(double volts) {
    topShooterFlywheel.setInputVoltage(volts);
  }

  @Override
  public void setBottomShooterMotorVoltage(double volts) {
    bottomShooterFlywheel.setInputVoltage(volts);
  }
}
