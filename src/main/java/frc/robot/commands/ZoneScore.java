package frc.robot.commands;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SOTM;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Visualization;

public class ZoneScore extends Command {
  private final ShooterSubsystem m_shooter;
  private final IntakeSubsystem m_intake;
  private double distanceToTarget;
  private double hoodAngle;
  private double shooterVelocity;
  private Timer m_timer = new Timer();
  private int loopCount = 0;
  private boolean safe = false;
  private Translation2d target = new Translation2d();

  public ZoneScore(
      ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem) {
    m_shooter = shooterSubsystem;
    m_intake = intakeSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsystem, intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.restart();
    loopCount = 0;
    Robot.lights.setFire();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    safe = m_shooter.isShooterSafe();
    DogLog.log("SOTM: Safe", safe);

    // target = Constants.ShotCalc.targetpose.getTranslation();


    if (safe==true){

    Pose2d currentPose = RobotContainer.drivetrain.getState().Pose;
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    if ((currentPose.getX() > Constants.Pose.ZoneLine && alliance == Alliance.Blue)||(currentPose.getX() < Constants.Pose.ZoneLine && alliance == Alliance.Red)) {
        if (currentPose.getY() > Constants.Pose.Halfline){
           target = Constants.ShotCalc.upperPassPose.getTranslation();
          //  System.out.println("Upper Pass Pose");
           DogLog.log("SOTM: Target","Upper Pass Pose");
        } else {
            target = Constants.ShotCalc.lowerPassPose.getTranslation();
            // System.out.println("Lower Pass Pose");
            DogLog.log("SOTM: Target","Lower Pass Pose");
        }
    } else {
        target = Constants.ShotCalc.targetpose.getTranslation();
        // System.out.println("Target Pose");
        DogLog.log("SOTM: Target","Target Pose");
    }


        SOTM.calcSOTM(target, Constants.ShotCalc.ShotTime);
        distanceToTarget = SOTM.targetDistance();
        hoodAngle = Constants.ShotCalc.ShotAngle.get(distanceToTarget);
        shooterVelocity = Constants.ShotCalc.ShotVelocity.get(distanceToTarget);
            m_shooter.setShooterVelocity(shooterVelocity);
      if (loopCount % 10 == 0) {
      Visualization.LaunchFuelViz(shooterVelocity, Units.degreesToRadians(90)-hoodAngle);
    }
    } else {
        hoodAngle = Constants.Shooter.hoodStowAngle;
    }
    m_shooter.setHoodAngle(hoodAngle);
    m_shooter.setIndexerVelocity(Constants.Shooter.indexerVelocity);
    m_intake.setHopperRollerVelocity(Constants.Intake.intakeRollerVelocity);
    

    loopCount++;


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.setShooterVelocity(0.0);
    m_shooter.setHoodAngle(Constants.Shooter.hoodStowAngle);
    Robot.lights.setPreviousControl();
    m_shooter.setIndexerVelocity(0);
    m_intake.setHopperRollerVelocity(0);
  }

  // Returns true when the command should end.
  // @Override
  // public boolean isFinished() {
  //   return true;
  // }
}
