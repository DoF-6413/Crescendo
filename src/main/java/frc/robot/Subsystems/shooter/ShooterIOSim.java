package frc.robot.Subsystems.shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants.RobotStateConstants;
import frc.robot.Constants.ShooterConstants;

public class ShooterIOSim implements ShooterIO {
  // Creating flywheels
  private FlywheelSim topShooterFlywheel =
      new FlywheelSim(
          DCMotor.getFalcon500(1),
          ShooterConstants.SHOOTER_GEAR_RATIO,
          ShooterConstants.SHOOTER_MOI_KG_M2);

  private FlywheelSim bottomShooterFlywheel =
      new FlywheelSim(
          DCMotor.getFalcon500(1),
          ShooterConstants.SHOOTER_GEAR_RATIO,
          ShooterConstants.SHOOTER_MOI_KG_M2);

  public ShooterIOSim() {
    System.out.println("[Init] Creating ShooterIOSim");
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    // Updates the Shooter motors periodically
    topShooterFlywheel.update(RobotStateConstants.LOOP_PERIODIC_SEC);
    bottomShooterFlywheel.update(RobotStateConstants.LOOP_PERIODIC_SEC);

    // Updates logged inputs, RPM and current. Voltage and temp aren't updated because ideally (like
    // in a simulation), they would be constant
    inputs.topShooterMotorRPM = topShooterFlywheel.getAngularVelocityRPM();
    inputs.topShooterCurrentAmps = new double[] {Math.abs(topShooterFlywheel.getCurrentDrawAmps())};
    inputs.bottomShooterMotorRPM = bottomShooterFlywheel.getAngularVelocityRPM();
    inputs.bottomShooterCurrentAmps =
        new double[] {Math.abs(bottomShooterFlywheel.getCurrentDrawAmps())};
  }

  // TODO: Update below methods to implement PID
  @Override
  public void setBothShooterMotorPercentSpeed(double percent) {
    // Sets the speed based on a percentage of the voltage
    topShooterFlywheel.setInputVoltage(ShooterConstants.APPLIED_VOLTS * percent);
    bottomShooterFlywheel.setInputVoltage(ShooterConstants.APPLIED_VOLTS * percent);
  }

  @Override
  public void setBothShooterMotorsVoltage(double volts) {
    // Sets voltage
    topShooterFlywheel.setInputVoltage(volts);
    bottomShooterFlywheel.setInputVoltage(volts);
  }

  @Override
  public void setBottomShooterMotorVoltage(double volts) {
    // Sets voltage based on PID
    bottomShooterFlywheel.setInputVoltage(volts);
  }
}
