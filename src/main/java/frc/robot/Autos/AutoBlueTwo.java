// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ArmDownCommand;
import frc.robot.commands.ArmUpCommand;
import frc.robot.commands.IntakeTimeCommand;
import frc.robot.commands.WristSetPositionCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.WristSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoBlueTwo extends SequentialCommandGroup {
  /** Creates a new AutoBlueTwo. */
  public AutoBlueTwo(ArmSubsystem arm, WristSubsystem wrist, DriveSubsystem drive) {

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ArmUpCommand(arm),
      new WaitCommand(1),
      new WristSetPositionCommand(wrist, 50),
      new WaitCommand(2),
      new IntakeTimeCommand(arm, 0.4, 2),
      new WaitCommand(1),
      new WristSetPositionCommand(wrist, 10),
      new WaitCommand(1),
      new ArmDownCommand(arm)
      // new WaitCommand(1),
      // new DriveTimeTestCommand(drive, -.15, 0, 0, 6)
    );
  }
}
