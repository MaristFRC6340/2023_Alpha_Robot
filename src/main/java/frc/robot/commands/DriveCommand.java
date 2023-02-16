// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveCommand extends CommandBase {

  // Field for DriveSubsystem
  private final DriveSubsystem m_robotDrive;


  /** Creates a new DriveCommand. */
  public DriveCommand(DriveSubsystem drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_robotDrive = drive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  double leftX = 0;
  double leftY = 0;
  double rightX = 0;


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Updated Drive Command

    leftX = Robot.getDriveControlJoystick().getRawAxis(0);
    leftY = Robot.getDriveControlJoystick().getRawAxis(1);
    rightX = Robot.getDriveControlJoystick().getRawAxis(4);


    m_robotDrive.drive(
                MathUtil.applyDeadband(-leftY*DriveConstants.SpeedMultiplier, 0.06),
                MathUtil.applyDeadband(-leftX*DriveConstants.SpeedMultiplier, 0.06),
                MathUtil.applyDeadband(-rightX*DriveConstants.SpeedMultiplier, 0.06),
                false);

                
    if(Robot.getDriveControlJoystick().getRawButton(9)){ // Left Stick Button
      m_robotDrive.zeroHeading();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Todo: Set motors to stop

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
