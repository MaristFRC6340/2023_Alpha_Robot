// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Autos.AutoBlueCubeRamp;
import frc.robot.Autos.AutoBlueCubeRampExit;
import frc.robot.Autos.AutoBlueHighCone;
import frc.robot.Autos.AutoBlueHighConePickup;
import frc.robot.Autos.AutoBlueHighConeRamp;
import frc.robot.Autos.AutoBlueLowRamp;
import frc.robot.Autos.AutoBlueOne;
import frc.robot.Autos.AutoBlueTwo;
import frc.robot.Autos.AutoRampClimb;
import frc.robot.Autos.DriveTimeTestCommand;
import frc.robot.Autos.AutoTurnTest;
import frc.robot.Autos.ExampleAuto;
import frc.robot.Autos.ShootCubeHighSequence;
import frc.robot.Autos.AutoRampClimbExit;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.NavXTurnCommand;
import frc.robot.commands.WristRotatePIDTestCommand;
import frc.robot.commands.WristRotateTestCommand;
import frc.robot.commands.WristTeleopCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.WristSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final ArmSubsystem m_armSubsystem = new ArmSubsystem();
  private final WristSubsystem m_wristSubsystem = new WristSubsystem();

  private final ArmCommand armCommand = new ArmCommand(m_armSubsystem);


  // The driver's controller
  // TODO: Move this to a Static field in Robot Class
  //XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    // Below commented out for refactoring
    /* 
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                MathUtil.applyDeadband(-Robot.getDriveControlJoystick().getLeftY()*DriveConstants.SpeedMultiplier, 0.06),
                MathUtil.applyDeadband(-Robot.getDriveControlJoystick().getLeftX()*DriveConstants.SpeedMultiplier, 0.06),
                MathUtil.applyDeadband(-Robot.getDriveControlJoystick().getRightX()*DriveConstants.SpeedMultiplier, 0.06),
                true),
            m_robotDrive));
    */
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 0.25),
                new Translation2d(2, 0.5)
               //new Translation2d(3, 0.5)
               ),

        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(5, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
  }
  
  public Command getArmCommand(){
    return armCommand;
  }

  public Command getAutoBlueOne() {
    return new AutoBlueOne(m_armSubsystem, m_wristSubsystem, m_robotDrive);
  }

  public Command getDriveCommand() {
    return new DriveCommand(m_robotDrive);
  }

  public Command getWristRotateTestCommand() {
    return new WristRotateTestCommand(m_wristSubsystem, -100);
  }

  public Command getWristPIDCommandTest() {
    return new WristRotatePIDTestCommand(m_wristSubsystem, 0);
  }

  public Command getWristTeleopCommand() {
    return new WristTeleopCommand(m_wristSubsystem, 10);
  }

  // Test of DriveTimeTestCommand
  public Command getDriveTimeTestCommand() {
    return new DriveTimeTestCommand(m_robotDrive, -0.15, 0, 0, 1);
  }

  public Command getAutoBlueTwo() {
    return new AutoBlueTwo(m_armSubsystem, m_wristSubsystem, m_robotDrive);
  }

  public Command getCubeShootSequence() {
    return new ShootCubeHighSequence(m_robotDrive, m_armSubsystem, m_wristSubsystem);
  }

  public Command getHighConeCommand() {
    return new AutoBlueHighCone(m_robotDrive, m_armSubsystem, m_wristSubsystem);
  }
  public Command getRampCommand(){
    return new AutoRampClimb(m_robotDrive, m_armSubsystem, m_wristSubsystem);
  }

  public Command getRampCubeCommand(){
    return new AutoBlueCubeRamp(m_robotDrive, m_armSubsystem, m_wristSubsystem);
  }

  public Command getRampCubeExitCommand(){
    return new AutoBlueCubeRampExit(m_robotDrive, m_armSubsystem, m_wristSubsystem);
  }

  public Command getHighConeRampCommand() {
    return new AutoBlueHighConeRamp(m_robotDrive, m_armSubsystem, m_wristSubsystem);
  }

  public Command getLowCubeRampCommand() {
    return new AutoBlueLowRamp(m_armSubsystem, m_wristSubsystem, m_robotDrive);
  }
  
  public Command getRampClimbExitCommand(){
    return new AutoRampClimbExit(m_robotDrive, m_armSubsystem, m_wristSubsystem);
  }

  public Command getTurnTestCommand(){
    return new AutoTurnTest(m_robotDrive);
  }

  public Command getHighConePickupCommand(){
    return new AutoBlueHighConePickup(m_robotDrive, m_armSubsystem, m_wristSubsystem);
  }

}
