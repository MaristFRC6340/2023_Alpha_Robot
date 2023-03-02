// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ArmDownCommand;
import frc.robot.commands.ArmLengthCommand;
import frc.robot.commands.ArmUpCommand;
import frc.robot.commands.IntakeTimeCommand;
import frc.robot.commands.WristSetPositionCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.WristSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoBlueHighCone extends SequentialCommandGroup {
  /** Creates a new AutoBlueHighCone. */
  public AutoBlueHighCone(DriveSubsystem drive, ArmSubsystem arm, WristSubsystem wrist) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ArmUpCommand(arm, 2),
      new ArmLengthCommand(arm, -.5, 1.75),
      new WristSetPositionCommand(wrist, 33, 2),
      new IntakeTimeCommand(arm, 0.8, 1),
      new WristSetPositionCommand(wrist, 5),
      new ArmLengthCommand(arm, .5, 1.95),
      new ArmDownCommand(arm, 0.1),
      new DriveTimeTestCommand(drive, -.3, 0, 0, 3)
    );
  }
}
