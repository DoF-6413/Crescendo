package frc.robot.Subsystems.climber;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.RobotStateConstants;

public class ClimberIOSim implements ClimberIO {

  private final ElevatorSim leftClimberSim =
      new ElevatorSim(
          DCMotor.getNEO(1),
          ClimberConstants.CLIMBER_GEAR_RATIO,
          ClimberConstants.CLIMBER_CARRIAGE_MASS_KG,
          ClimberConstants.CLIMBER_DRUM_RADIUS_M,
          ClimberConstants.CLIMBER_MIN_HEIGHT_M,
          ClimberConstants.CLIMBER_MAX_HEIGHT_M,
          ClimberConstants.CLIMBER_SIMULATE_GRAVITY,
          ClimberConstants.CLIMBER_STARTING_HEIGHT_M);
  private final ElevatorSim rightClimberSim =
      new ElevatorSim(
          DCMotor.getNEO(1),
          ClimberConstants.CLIMBER_GEAR_RATIO,
          ClimberConstants.CLIMBER_CARRIAGE_MASS_KG,
          ClimberConstants.CLIMBER_DRUM_RADIUS_M,
          ClimberConstants.CLIMBER_MIN_HEIGHT_M,
          ClimberConstants.CLIMBER_MAX_HEIGHT_M,
          ClimberConstants.CLIMBER_SIMULATE_GRAVITY,
          ClimberConstants.CLIMBER_STARTING_HEIGHT_M);

  public ClimberIOSim() {
    System.out.println("[Init] Creating ClimberIOSim");
    leftClimberSim.update(RobotStateConstants.LOOP_PERIODIC_SEC);
  }

  public void updateInputs(ClimberIOInputs inputs) {
    inputs.leftClimberPositionRad =
        leftClimberSim.getPositionMeters()
            / ClimberConstants.CLIMBER_GEAR_RATIO; // TODO: Update math
    inputs.leftClimberVelocityRadPerSec =
        leftClimberSim.getVelocityMetersPerSecond()
            / ClimberConstants.CLIMBER_GEAR_RATIO; // TODO: Update math
    inputs.leftClimberAppliedVolts = 0.0;
    inputs.leftClimberCurrentAmps = new double[] {leftClimberSim.getCurrentDrawAmps()};

    inputs.rightClimberPositionRad = rightClimberSim.getPositionMeters(); // TODO: Update math
    inputs.rightClimberVelocityRadPerSec =
        rightClimberSim.getVelocityMetersPerSecond()
            / ClimberConstants.CLIMBER_GEAR_RATIO; // TODO: Update math
    inputs.rightClimberAppliedVolts = 0.0;
    inputs.rightClimberCurrentAmps = new double[] {rightClimberSim.getCurrentDrawAmps()};
  }

  @Override
  public void setBothClimberMotorsVoltage(double volts) {
    leftClimberSim.setInputVoltage(volts);
    rightClimberSim.setInputVoltage(volts);
  }

  @Override
  public void setBothClimberMotorsPercentSpeed(double percent) {
    leftClimberSim.setInputVoltage(percent);
    rightClimberSim.setInputVoltage(percent);
  }
}