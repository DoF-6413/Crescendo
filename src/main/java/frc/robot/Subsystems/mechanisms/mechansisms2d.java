package frc.robot.Subsystems.mechanisms;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.shooter.Shooter;

public class mechansisms2d extends SubsystemBase {
    private final Mechanism2d shooter;
    public Mechanism2d(Shooter shooter) {
        this.shooter = shooter;
    }
}