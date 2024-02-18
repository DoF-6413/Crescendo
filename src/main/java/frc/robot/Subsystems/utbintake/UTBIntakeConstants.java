package frc.robot.Subsystems.utbintake;

  public final class UTBIntakeConstants {
    // PID Constants for the wrist; TODO: finalize PID values once they are determined + add 'final' modifiers

    /**
     * represents the proportional constant, multiplied by the current error
     */
    public static double KP = 0.0;

    /**
     * represents the integral constant, multiplied by the total error
     */
    public static double KI = 0.0;

    /**
     * represents the derivative constant, multiplied by the change in error
     */
    public static double KD = 0.0;

     /** the RPM of the UTB intake can be within 1% of the setpoint */
    public static final double TOLERANCE_PERCENT = 0.01;

    public static final int CAN_ID = 0; // TODO: Update later
    public static final int GEAR_RATIO = 2; // 2:1 Gear Ratio
    public static final double UTB_MOI_KG_M2 = 0.0001929765;

  }
