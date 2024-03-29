// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autos;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.IntakeTimeCommand;
import frc.robot.commands.WristSetPositionCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.WristSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoBlueOne extends SequentialCommandGroup {
  /** Creates a new AutoBlueOne. */
  public AutoBlueOne(ArmSubsystem arm, WristSubsystem wrist, DriveSubsystem drive) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    // Test Sequence of Placing Object then moving backwards
    addCommands(
    new WristSetPositionCommand(wrist, 20),
    new IntakeTimeCommand(arm, -0.4, 2),
    new WaitCommand(1),

    Commands.parallel(
      new WristSetPositionCommand(wrist, 0),
      new DriveTimeTestCommand(drive, -.15, 0, 0, 6)
    )

    );
  }
}
