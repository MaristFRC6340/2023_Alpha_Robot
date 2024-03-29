package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class NavXTurnCommand extends CommandBase {
    private final DriveSubsystem m_robotDrive;


    private double angleDegrees;
    private double power;
    private double target;
    private double error;
    private double kP = 0.025;
    private double theta;

    public NavXTurnCommand(DriveSubsystem drive, double power, double target){
        m_robotDrive = drive;
        this.target = target;
        this.power = power;
        addRequirements(drive);
    }

    @Override
    public void initialize(){
        m_robotDrive.zeroHeading(0);
        error = 0;

    }

    @Override
    public void execute(){
        theta = m_robotDrive.getHeading();
        error = target - theta;
        double turnAdjust = error*kP;
        
        if(turnAdjust>power){
            turnAdjust = power;
        }
        if(turnAdjust<-power){
            turnAdjust = -power;
        }
        //System.out.println("error: " + error + "   turnAdjust: " + turnAdjust);
        m_robotDrive.drive(0, 0, -turnAdjust, false);

    }

    public boolean isFinished(){

        return Math.abs(error)<2;
    }
    public void end(boolean interrupted){
        m_robotDrive.drive(0,0,0,false);
    }

}
