// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class ArmUpWristSetPositionCommand extends CommandBase {
  private final ArmSubsystem arm;
  private WristSubsystem wrist;
  private double newRotation; 

  private double duration;
  private long startTime;
  private long endTime;
  /** Creates a new ArmUpCommand. */
  
  public ArmUpWristSetPositionCommand(ArmSubsystem arm, WristSubsystem wrist, double duration, double rotation) {
    this.arm = arm;
    this.wrist = wrist;
    this.duration = duration;
    newRotation = rotation;
    addRequirements(wrist);
  }
  
  public ArmUpWristSetPositionCommand(ArmSubsystem arm, WristSubsystem wrist, double rotation) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    this.wrist = wrist;
    duration = 1;
    newRotation = rotation;
    addRequirements(wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = System.currentTimeMillis();
    endTime = startTime + (long)(duration * 1000);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.armUp();
    wrist.setWristReference(newRotation);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    long currentTime = System.currentTimeMillis();
    if (currentTime < endTime) {
      return false;
    }
    else {
      return true;
    }
  }
}
