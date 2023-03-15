// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystem;

public class ForwardRampClimbBangBang extends CommandBase {


  private DriveSubsystem  m_drive;


  private int count = 0; 
  private int frameCount = 0; 
  private double totalDeltaX;
  private double avgDeltaX;
  private int arraySize = 25;
  private double deltaXValues [] = new double[arraySize];

  private double rateOfChange = 0;
  private double oldDeltaX = 0;

  private double oldDeltaY = 0;

  /** Creates a new RampClimbBangBang. */
  public ForwardRampClimbBangBang(DriveSubsystem drive) {

    m_drive = drive;

    addRequirements(drive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double deltaX = Robot.getRioAccell().getX();
    deltaXValues[count] = deltaX;

    // Computes Average deltaX - acceleration in X direction
    totalDeltaX = 0;
    for(int i = 0; i < deltaXValues.length; i++){
      totalDeltaX += deltaXValues[i];
    }
    count++;
    if(count > deltaXValues.length-1){
      count = 0;
    }
    avgDeltaX = totalDeltaX/deltaXValues.length;

    // Compute rate of change
    rateOfChange = avgDeltaX - oldDeltaX;

    // Update oldDeltaX
    oldDeltaX = avgDeltaX;

    frameCount++;
// add time delay for robot while it is tipping because the robot is nto getting contct with the ramp
    if(frameCount > 50){
      m_drive.drive(0.1, 0, 0, false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_drive.drive(0, 0, 0, false);
    m_drive.setX();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    System.out.println("DeltaX: " + avgDeltaX + ", " + rateOfChange + ", " + frameCount);

    if (avgDeltaX < 0.16) {
    //if (rateOfChange > .025) {
      if (frameCount > 100) {
        return true;
      } else {
        return false;
      }
    }

    return false;
  }
}
