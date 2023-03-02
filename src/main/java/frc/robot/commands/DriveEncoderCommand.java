// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Autos.DriveTimeTestCommand;
import frc.robot.subsystems.DriveSubsystem;

public class DriveEncoderCommand extends CommandBase {



  private DriveSubsystem m_robot;


  private double deltaX;
  private double deltaY;
  private double theta;
  private double time;

  private long startTime = 0;
  private long endTime = 0; 
  private double duration = 0;

  private double distance = 0;
  private double currentDistance = 0;
  private double targetDistance = 0;


  public DriveEncoderCommand(DriveSubsystem swerve, double deltaX, double deltaY, double theta, double distance){
    this.m_robot = swerve;


    this.deltaX = deltaX;
    this.deltaY = deltaY;
    this.theta = theta;
    this.distance = distance;

    // Inititalize the currentDistance
    currentDistance = m_robot.getLeftModuleDistance();
    targetDistance = currentDistance + distance;

    addRequirements(swerve);
  }



  /** Creates a new DriveEncoderCommand. */
  public DriveEncoderCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double dX = deltaX;
    double dY = deltaY;
    double Dtheta = theta;

    m_robot.drive(dX, dY, Dtheta, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_robot.drive(0, 0, 0, false);
    m_robot.setX();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    currentDistance = m_robot.getLeftModuleDistance();
    if(Math.abs(currentDistance-targetDistance) < .1){
      return true;
    }
    else{
      return false;
    }
  }
}
