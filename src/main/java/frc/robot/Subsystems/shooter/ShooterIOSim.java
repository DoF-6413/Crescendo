package frc.robot.Subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants.RobotStateConstants;
import frc.robot.Constants.ShooterConstants;

public class ShooterIOSim implements ShooterIO {
  // Creating flywheels
  private FlywheelSim topShooterFlywheel =
      new FlywheelSim(
          DCMotor.getFalcon500(1),
          ShooterConstants.GEAR_RATIO,
          ShooterConstants.SHOOTER_J_KG_METERS_SQUARED);

  private FlywheelSim bottomShooterFlywheel =
      new FlywheelSim(
          DCMotor.getFalcon500(1),
          ShooterConstants.GEAR_RATIO,
          ShooterConstants.SHOOTER_J_KG_METERS_SQUARED);

  // PID and FeedForward Contollers for the top shooter flywheel
  // private final PIDController topController =
  //     new PIDController(
  //         ShooterConstants.SHOOTER_KP, ShooterConstants.SHOOTER_KI, ShooterConstants.SHOOTER_KD);
  // private SimpleMotorFeedforward topFF =
  //     new SimpleMotorFeedforward(ShooterConstants.SHOOTER_KS, ShooterConstants.SHOOTER_KV);

  // PID and FeedForward Contollers for the bottom shooter flywheel
  // private final PIDController bottomController =
  //     new PIDController(
  //         ShooterConstants.SHOOTER_KP, ShooterConstants.SHOOTER_KI, ShooterConstants.SHOOTER_KD);
  // private SimpleMotorFeedforward bottomFF =
  //     new SimpleMotorFeedforward(ShooterConstants.SHOOTER_KS, ShooterConstants.SHOOTER_KV);

  // private double topSetpointRPM = 2500;
  // private double bottomSetpointRPM = 3000;

  // private double topAppliedVolts = 0;

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
    // inputs.topShooterAppliedVolts = topAppliedVolts;

    inputs.bottomShooterMotorRPM = bottomShooterFlywheel.getAngularVelocityRPM();
    inputs.bottomShooterCurrentAmps =
        new double[] {Math.abs(bottomShooterFlywheel.getCurrentDrawAmps())};

    // if (topSetpointRPM >= Math.abs(topShooterFlywheel.getAngularVelocityRPM())) {
    //   topAppliedVolts =
    //       topController.calculate(topShooterFlywheel.getAngularVelocityRPM(), topSetpointRPM);
    //   topShooterFlywheel.setInputVoltage(
    //       MathUtil.clamp(ShooterConstants.APPLIED_VOLTS, -12.0, 12.0));
    }

    // if (bottomSetpointRPM >= Math.abs(bottomShooterFlywheel.getAngularVelocityRPM())) {
    //   ShooterConstants.APPLIED_VOLTS =
    //       bottomController.calculate(
    //           bottomShooterFlywheel.getAngularVelocityRPM(),
    //           bottomSetpointRPM + bottomFF.calculate(bottomSetpointRPM));
    //   bottomShooterFlywheel.setInputVoltage(
    //       MathUtil.clamp(ShooterConstants.APPLIED_VOLTS, -12.0, 12.0));
    // }
  

  @Override
  public void setShooterMotorPercentSpeed(double percent) {
    // sets voltage
    topShooterFlywheel.setInputVoltage(ShooterConstants.APPLIED_VOLTS * percent);
    // bottomShooterFlywheel.setInputVoltage(ShooterConstants.APPLIED_VOLTS * percent);
  }
}
 
