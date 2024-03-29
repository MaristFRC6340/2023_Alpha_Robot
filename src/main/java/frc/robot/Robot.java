// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// This is the Alpha Robot Code for 2023 Marist FRC Team 6340

package frc.robot;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  // TODO: Define the controllers for driving / arm control in static form
  // We will need to decide the mappings
  private static Joystick m_armControlJoystick = new Joystick(Constants.OIConstants.kArmControllerPort); // Port zero for left joystick
  private static Joystick m_driverControlJoystick = new Joystick(Constants.OIConstants.kDriverControllerPort);


  private static Accelerometer accelerometer = new BuiltInAccelerometer();

  //Smart Dashboard and Auto Chooser
  private static final String kDefaultOption = "Default";
  private static final String kBlue1 = "Blue1 Cube";
  private static final String kBlue2 = "Blue2 Cone";
  private static final String kBlue3 = "Blue3";
  private static final String kRed1 = "Red1";
  private static final String kRed2 = "Red2";
  private static final String kRed3 = "Red3";
  private static final String kExampleAuto = "Example";
  private static final String kWristRotateTestCommand = "Wrist Rotate Test";
  private static final String kPIDWristTestCommand = "Wrist PID Test Command";
  private static final String kDriveTimeTestCommand = "Drive Time Test Command";
  private static final String kShootCubeHighCommand = "Shoot Cube High Sequence";
  private static final String kBlueHighCone = "Auto Blue High Cone";
  private static final String kRampClimb = "Ramp Climb";
  private static final String kBlueHighCubeRamp = "Auto Blue High Cube Ramp";
  private static final String kBlueHighCubeRampExit = "Auto Blue High Cube Ramp Exit";
  private static final String kBlueHighConeRamp = "Auto Blue High Cone Ramp";
  private static final String kBlueLowCubeRamp = "Auto Blue Low Cube Ramp";
  private static final String kExampleTest = "Example Auto Test";
  private static final String kBlueRampExit = "Auto Blue Ramp Exit";
  private static final String kTurnTest = "Auto Turn Test";
  private static final String kBlueHighConePickup = "Auto Blue High Cone Pickup";

  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser();



  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    //Initialize Chooser

    m_chooser.setDefaultOption("Default Auto", kDefaultOption);
    m_chooser.addOption("Blue1 Cube", kBlue1);
    m_chooser.addOption("Blue2 Cone", kBlue2);
    m_chooser.addOption("Blue3", kBlue3);
    m_chooser.addOption("Red1", kRed1);
    m_chooser.addOption("Red2", kRed2);
    m_chooser.addOption("Red3", kRed3);
    m_chooser.addOption("Example Auto", kExampleAuto);
    m_chooser.addOption("Wrist Rotate Test", kWristRotateTestCommand);
    m_chooser.addOption("Wrist PID Test", kPIDWristTestCommand);
    m_chooser.addOption("Drive Time Test", kDriveTimeTestCommand);
    m_chooser.addOption("Shoot Cube High Sequence", kShootCubeHighCommand);
    m_chooser.addOption("Auto Blue High Cone", kBlueHighCone);
    m_chooser.addOption("RampClimb", kRampClimb );
    m_chooser.addOption("Auto Blue High Cube Ramp", kBlueHighCubeRamp);
    m_chooser.addOption("Auto Blue High Cube Ramp Exit", kBlueHighCubeRampExit);
    m_chooser.addOption("Auto Blue High Cone Ramp", kBlueHighConeRamp);
    m_chooser.addOption(kBlueLowCubeRamp, kBlueLowCubeRamp);
    m_chooser.addOption("Example Auto Command", kExampleTest);
    m_chooser.addOption("Auto Blue Ramp Exit", kBlueRampExit);
    m_chooser.addOption("Auto Turn Test", kTurnTest);
    m_chooser.addOption("Auto Blue High Cone Pickup", kBlueHighConePickup);
  


    String[] choices = {kBlue1, kBlue2, 
                        kBlueHighCone, kRampClimb, kBlueHighCubeRamp,
                        kBlueHighConeRamp, kBlueLowCubeRamp, kExampleTest, kBlueHighCubeRampExit, kBlueRampExit,
                        kTurnTest, kBlueHighConePickup};

    SmartDashboard.putStringArray("Auto List", choices);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {

    m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultOption);
    System.out.println("Auto selected: " + m_autoSelected);

    switch (m_autoSelected) {
      case kBlue1:
        System.out.println("Blue1");
        m_autonomousCommand = m_robotContainer.getAutoBlueOne();
        break;
      case kBlue2:
        System.out.println("Blue2");
        m_autonomousCommand = m_robotContainer.getAutoBlueTwo();
        break;
      case kBlueHighCone:
        m_autonomousCommand = m_robotContainer.getHighConeCommand();
        break;
      case kRampClimb:
        m_autonomousCommand = m_robotContainer.getRampClimbExitCommand();
        break;
      case kBlueHighCubeRamp:
        m_autonomousCommand = m_robotContainer.getRampCubeCommand();
        break;
      case kBlueHighConeRamp:
        m_autonomousCommand = m_robotContainer.getHighConeRampCommand();
        break;
      case kBlueLowCubeRamp:
        m_autonomousCommand = m_robotContainer.getLowCubeRampCommand();
        break;
      case kBlueHighCubeRampExit:
        m_autonomousCommand = m_robotContainer.getRampCubeExitCommand();
        break;
      case kExampleTest:
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        break;
      case kBlueRampExit:
        m_autonomousCommand = m_robotContainer.getRampClimbExitCommand();
      case kTurnTest:
        m_autonomousCommand = m_robotContainer.getTurnTestCommand();
      case kBlueHighConePickup:
        m_autonomousCommand = m_robotContainer.getHighConePickupCommand();
      case kDefaultOption:
        System.out.println("Default");
        default:
        break;
    }

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.getArmCommand().schedule();
    m_robotContainer.getWristTeleopCommand().schedule();
    m_robotContainer.getDriveCommand().schedule();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  public static Joystick getArmControlJoystick(){
    return m_armControlJoystick;
  }

  public static Joystick getDriveControlJoystick() {
    return m_driverControlJoystick;
  }

  public static Accelerometer getRioAccell() {
    return accelerometer;
  }

}
