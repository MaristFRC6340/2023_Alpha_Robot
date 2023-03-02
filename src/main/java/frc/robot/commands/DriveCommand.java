// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveCommand extends CommandBase {

  // Field for DriveSubsystem
  private final DriveSubsystem m_robotDrive;

  private double speedControl = 0.5;

  // Limelight Fields
  private NetworkTable limTable;
  private NetworkTableEntry tx;
  private NetworkTableEntry ledMode;

  // PID Control for Limelight Turn
  private double kP = 0.03;

  // For PID Control
  double turnPower = 0;


  /** Creates a new DriveCommand. */
  public DriveCommand(DriveSubsystem drive) {

    // Limelight Initialization
    limTable = NetworkTableInstance.getDefault().getTable("limelight");
    tx = limTable.getEntry("tx");
    ledMode = limTable.getEntry("ledMode");

    

    // Use addRequirements() here to declare subsystem dependencies.
    m_robotDrive = drive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ledMode.setDouble(1);
  }

  double leftX = 0;
  double leftY = 0;
  double rightX = 0;


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Updated Drive Command

    leftX = Robot.getDriveControlJoystick().getRawAxis(0);
    leftY = Robot.getDriveControlJoystick().getRawAxis(1);
    rightX = Robot.getDriveControlJoystick().getRawAxis(4) + turnPower; // Adjust for PID Limelight


    if (Robot.getDriveControlJoystick().getRawAxis(2) > 0.5) {
      ledMode.setDouble(3);
      double error = tx.getDouble(0);
      turnPower = kP * error;
      System.out.println(tx.getDouble(0) + ", " + turnPower);
      
    }
    else {
      ledMode.setDouble(1);
      turnPower = 0;
    }



    m_robotDrive.drive(
                MathUtil.applyDeadband(-leftY*speedControl, 0.06),
                MathUtil.applyDeadband(-leftX*speedControl, 0.06),
                MathUtil.applyDeadband(-rightX*speedControl, 0.06),
                true);

                
    if(Robot.getDriveControlJoystick().getPOV()!=-1){ // Left Stick Button
      m_robotDrive.zeroHeading(Robot.getDriveControlJoystick().getPOV());
    }

    // Speed Control
    // Left Bumper Full Speed
    if (Robot.getDriveControlJoystick().getRawButton(5)) {
      speedControl = 1.0;
    }
    // Right Bumper Half Speed
    if (Robot.getDriveControlJoystick().getRawButton(6)) {
      speedControl = 0.5;
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
