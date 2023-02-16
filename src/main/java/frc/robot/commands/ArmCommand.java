// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.Robot;

public class ArmCommand extends CommandBase {
  /** Creates a new ArmCommand. */

  private final ArmSubsystem arm;
  double leftY = 0, rightY = 0;
  double leftX = 0, rightX = 0;

  public ArmCommand( ArmSubsystem arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Arm Control
    if(Robot.getArmControlJoystick().getRawButton(5)){ // lb
      arm.armUp();
    }
    if(Robot.getArmControlJoystick().getRawButton(6)){ // rb
      arm.armDown();
    }

    // if(Robot.getArmControlJoystick().getRawButton(3)) {
    //   arm.setRotations(100);
    // }

    // Intake Control
    if(Robot.getArmControlJoystick().getRawButton(4)){ 
      //arm.setWristMotorPower(.5);
      arm.setIntakeLeftMotorPower(.8);
    }
    else if(Robot.getArmControlJoystick().getRawButton(1)){ 
      //arm.setWristMotorPower(-.5);
      arm.setIntakeLeftMotorPower(-.8);
    }
    else { 
      arm.setIntakeLeftMotorPower(0);
    }

    // Arm Length and Wrist Control
    leftY = Robot.getArmControlJoystick().getRawAxis(1); //arm motor
    
    rightY = Robot.getArmControlJoystick().getRawAxis(5)/4; //wrist motor

    leftX = Robot.getArmControlJoystick().getRawAxis(4);
    
    rightX = Robot.getArmControlJoystick().getRawAxis(0);

    arm.setArmLengthMotorPower(MathUtil.applyDeadband(leftY, 0.06));
    arm.setWristMotorPower(MathUtil.applyDeadband(rightY, 0.06));
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Set all motors to off
    arm.setIntakeLeftMotorPower(0);
    arm.setArmLengthMotorPower(0);
    arm.setWristMotorPower(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
