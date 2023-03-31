// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DriveEncoderCommand;
import frc.robot.commands.RampClimbNavX;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.WristSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoRampClimbExit extends SequentialCommandGroup {
  /** Creates a new AutoRampClimb. */
  public AutoRampClimbExit(DriveSubsystem drive, ArmSubsystem arm, WristSubsystem wrist) {

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      
      // Ramp Climber: Substitute with RampClimbNavX when finished
      new DriveTimeTestCommand(drive, -.18, 0, 0, 5),
      new WaitCommand(.5),
      new DriveTimeTestCommand(drive, .18, 0, 0, 2.8),
      new RampClimbNavX(drive)
      
    );
  }
}