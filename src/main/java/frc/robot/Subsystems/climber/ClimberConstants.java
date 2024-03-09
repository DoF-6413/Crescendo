package frc.robot.Subsystems.climber;

import edu.wpi.first.math.util.Units;

public class ClimberConstants {

  /** ID of the Climber Motor on the CAN BUS */
  public static final int MOTOR_ID = 17;
  /** Sets the inversion of the Climber Motor */
  public static final boolean IS_INVERTED = true;
  /** Gear Ratio of 30:1 on the Climber Motor */
  public static final double GEAR_RATIO = 30;

  public static final double SHAFT_RADIUS_M = Units.inchesToMeters(1.5);
  public static final int CUR_LIM_A = 40;
}
