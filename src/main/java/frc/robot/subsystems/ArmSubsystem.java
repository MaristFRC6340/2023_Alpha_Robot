// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/**
 * Intake Subsystem to control Grasper
 * This is a stub class to be completed for Alpha Robot Testing
 * Use 2023Pneumatics2017Robot project for example
 * This system will use Pneumatics for Lifting of Arm and closing of Grasper
 * Thus, this will be for Arm and Intake
 * Angle of Intake is also controlled in this subsystem.
 * 
 * Might consider placing intake into a seperate subsystem.
 */

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {

  // Add Solenoids for Arm and Grasper
    private DoubleSolenoid armSol, intakeSolLeft, intakeSolRight;

    private CANSparkMax armLengthMotor; // Remove Wrist motor 18 Feb 23 michaudc
    private Spark intakeLeftMotor;

    private Compressor compressor;


  /** Creates a new Intake. */
  public ArmSubsystem() {
    // Initialize Solenoids
    armSol = new DoubleSolenoid(2, PneumaticsModuleType.CTREPCM,0,1);
    intakeSolLeft = new DoubleSolenoid(2, PneumaticsModuleType.CTREPCM, 3, 4);
    intakeSolRight = new DoubleSolenoid(2, PneumaticsModuleType.CTREPCM, 5, 6);

    // Set armSol to off to help keep pressurized: michaudc 09 mar 23
    armSol.set(Value.kOff);

    // Added 20 Feb 23 to control compressor state michaudc
    compressor = new Compressor(2, PneumaticsModuleType.CTREPCM);
    compressor.enableDigital();
    
    armLengthMotor = new CANSparkMax(6, MotorType.kBrushless);
    //armAngleMotor = new CANSparkMax(5, MotorType.kBrushless);
    //wristMotor = new CANSparkMax(7, MotorType.kBrushless); // removed 17 Feb 23 tonioloa
    intakeLeftMotor = new Spark(5);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  // Add Methods to Control Intake and Arm Solenoids
public void armUp(){
  armSol.set(Value.kReverse);
}

public void armDown(){
  armSol.set(Value.kForward);
}

public void openLeft(){
  intakeSolLeft.set(Value.kForward);
}

public void closeLeft(){
  intakeSolLeft.set(Value.kReverse);
}

public void openRight(){
  intakeSolRight.set(Value.kForward);
}

public void closeRight(){
  intakeSolRight.set(Value.kReverse);
}

public void setArmLengthMotorPower(double power){
  armLengthMotor.set(power);
}

public void setWristMotorPower(double power){
  // wristMotor.set(power); // removed 18 Feb 23 michaudc
}

public void setIntakeLeftMotorPower(double power){
  intakeLeftMotor.set(power);
}

public void turnCompressorOn() {
  compressor.enableDigital();
}

public void turnCompressorOff() {
  compressor.disable();
}

/* removed 18 Feb 23 michaudc
public void setRotations(double rotations) {
  arm_PIDController.setReference(rotations, CANSparkMax.ControlType.kPosition);
}

public double getWristPosition() {
  return arm_RelativeEncoder.getPosition();
}

public RelativeEncoder getWristEncoder() {
  return arm_RelativeEncoder;
}
*/

public CANSparkMax getWristMotor() {
  // return wristMotor; // removed 18 Feb 23 michaudc
  return null;
}

// Add Code to set Rotations for Wrist Motor


// public void setAngleMotorPower(double power){
//   armAngleMotor.set(power);
// }

}
