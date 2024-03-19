package frc.robot.Subsystems.climber;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants.RobotStateConstants;

public class ClimberIOSim implements ClimberIO {

  private final ElevatorSim leftClimberSim;
  private final ElevatorSim rightClimberSim;

  /**
   * This is a simulation for the climber. It uses the elavtor simulation because it extends up and
   * down
   */
  public ClimberIOSim() {
    System.out.println("[Init] Creating ClimberIOSim");
    leftClimberSim =
        new ElevatorSim(
            DCMotor.getNEO(1),
            ClimberConstants.GEAR_RATIO,
            ClimberConstants.CARRIAGE_MASS_KG,
            ClimberConstants.DRUM_RADIUS_M,
            ClimberConstants.MIN_HEIGHT_M,
            ClimberConstants.MAX_HEIGHT_M,
            ClimberConstants.SIMULATE_GRAVITY,
            ClimberConstants.STARTING_HEIGHT_M);

    rightClimberSim =
        new ElevatorSim(
            DCMotor.getNEO(1),
            ClimberConstants.GEAR_RATIO,
            ClimberConstants.CARRIAGE_MASS_KG,
            ClimberConstants.DRUM_RADIUS_M,
            ClimberConstants.MIN_HEIGHT_M,
            ClimberConstants.MAX_HEIGHT_M,
            ClimberConstants.SIMULATE_GRAVITY,
            ClimberConstants.STARTING_HEIGHT_M);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    // Updates inputs periodaclly
    leftClimberSim.update(RobotStateConstants.LOOP_PERIODIC_SEC);
    rightClimberSim.update(RobotStateConstants.LOOP_PERIODIC_SEC);

    inputs.leftClimberPositionMeters = leftClimberSim.getPositionMeters();
    inputs.leftClimberVelocityMetersPerSecond = leftClimberSim.getVelocityMetersPerSecond();
    inputs.leftClimberAppliedVolts = 0.0;
    inputs.leftClimberCurrentAmps = new double[] {leftClimberSim.getCurrentDrawAmps()};

    inputs.rightClimberPositionMeters = rightClimberSim.getPositionMeters();
    inputs.rightClimberVelocityMetersPerSecond = rightClimberSim.getVelocityMetersPerSecond();
    inputs.rightClimberAppliedVolts = 0.0;
    inputs.rightClimberCurrentAmps = new double[] {rightClimberSim.getCurrentDrawAmps()};
  }

  @Override
  public void setBothClimberVoltage(double volts) {
    leftClimberSim.setInputVoltage(volts);
    rightClimberSim.setInputVoltage(volts);
  }

  @Override
  public void setBothClimberPercentSpeed(double percent) {
    leftClimberSim.setInputVoltage(RobotStateConstants.BATTERY_VOLTAGE * percent);
    rightClimberSim.setInputVoltage(RobotStateConstants.BATTERY_VOLTAGE * percent);
  }

  @Override
  public void setLeftClimberPercentSpeed(double percent) {
    leftClimberSim.setInputVoltage(RobotStateConstants.BATTERY_VOLTAGE * percent);
  }

  @Override
  public void setLeftClimberVoltage(double volts) {
    leftClimberSim.setInputVoltage(volts);
  }

  @Override
  public void setRightClimberPercentSpeed(double percent) {
    rightClimberSim.setInputVoltage(RobotStateConstants.BATTERY_VOLTAGE * percent);
  }

  @Override
  public void setRightClimberVoltage(double volts) {
    rightClimberSim.setInputVoltage(volts);
  }

  @Override
  public double getRightClimberPose() {
    return rightClimberSim.getPositionMeters();
  }

  @Override
  public double getLeftClimberPose() {
    return leftClimberSim.getPositionMeters();
  }
}
