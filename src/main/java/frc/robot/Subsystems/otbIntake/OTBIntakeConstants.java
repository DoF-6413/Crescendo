package frc.robot.Subsystems.otbIntake;

public class OTBIntakeConstants {
    public static final int CAN_ID = 0; // TODO: update this Id value please !!!
    public static final double GEAR_RATIO = 2.0;
    public static final int SMART_CURRENT_LIMIT_AMPS = 40; // TODO: Update

    // PID Constants  TODO: Tune, update, finalize
    public static double KP = 0.0;
    public static double KI = 0.0;
    public static double KD = 0.0;
    /** The RPM of the OTB Intake can be within 1% of the setpoint */
    public static final double TOLERANCE = 0.01; 

    // Sim constants for the OTB Intake Rollers
    /** The moment of inertia for the OTB Intake Sim */
    public static final double MOI_KG_M2 = 0.0000023411;
}
