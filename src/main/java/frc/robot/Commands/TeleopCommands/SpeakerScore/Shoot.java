// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.TeleopCommands.SpeakerScore;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.ZeroCommands.ArmToZero;
import frc.robot.Commands.ZeroCommands.EndEffectorToZero;
import frc.robot.Subsystems.arm.Arm;
import frc.robot.Subsystems.feeder.Feeder;
import frc.robot.Subsystems.feeder.FeederConstants;
import frc.robot.Subsystems.shooter.Shooter;
import frc.robot.Subsystems.wrist.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Shoot extends SequentialCommandGroup {
  /** Creates a new Shoot. */
  public Shoot(Feeder feeder, Shooter shooter, Arm arm, Wrist wrist, CommandXboxController xbox) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    Command feedAndShootDirection(CommandXboxController xbox, Feeder feeder){
      if(xbox.rightBumper().getAsBoolean()){
        return Commands.runOnce(()-> {}
    
        );
      }
      else if(xbox.leftTrigger().getAsBoolean() || xbox.leftBumper().getAsBoolean()){
        return Commands.runOnce(()-> {
          feeder.setSetpoint(FeederConstants.SPEAKER_RPM);
        }
    
        );
      }
      else {
        return Commands.runOnce(()-> {}
    
        );
      }
    }

    addCommands(
              new feedAndShootDirection(xbox, feeder),
            new WaitCommand(0.3),
            new EndEffectorToZero(shooter, feeder),
            new ArmToZero(wrist, arm)
            
            );
  }


}

