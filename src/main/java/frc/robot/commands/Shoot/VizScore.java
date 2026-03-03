package frc.robot.commands.Shoot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainExtra;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Visualization;

public class VizScore extends Command {
  private final ShooterSubsystem m_shooter;
  private final IntakeSubsystem m_intake;
  private double distanceToTarget;
  private double hoodAngle;
  private double shooterVelocity;
  private Timer m_timer = new Timer();
  private int loopCount = 0;

  public VizScore(
      ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem) {
    m_shooter = shooterSubsystem;
    m_intake = intakeSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsystem, intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    distanceToTarget = DrivetrainExtra.targetDistance(Constants.ShotCalc.targetpose);
    hoodAngle = Constants.ShotCalc.ShotAngle.get(distanceToTarget);
    shooterVelocity = Constants.ShotCalc.ShotVelocity.get(distanceToTarget);
    m_shooter.setHoodAngle(hoodAngle);
    m_shooter.setShooterVelocity(shooterVelocity);
    m_timer.restart();
    loopCount = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    distanceToTarget = DrivetrainExtra.targetDistance(Constants.ShotCalc.targetpose);
    hoodAngle = Constants.ShotCalc.ShotAngle.get(distanceToTarget);
    shooterVelocity = Constants.ShotCalc.ShotVelocity.get(distanceToTarget);
    m_shooter.setHoodAngle(hoodAngle);
    m_shooter.setShooterVelocity(shooterVelocity);
    if (loopCount % 10 == 0) {
    Visualization.LaunchFuelViz(shooterVelocity, Units.degreesToRadians(90)-hoodAngle);
    }
    loopCount++;
   m_intake.retractIntakeSlow();
    m_shooter.setIndexerVelocity(Constants.Shooter.indexerVelocity);
    m_intake.setHopperRollerVelocity(Constants.Intake.hopperVelocity);
    m_intake.setABRollerVelocity(Constants.Intake.SlowABRollerVelocity);
    m_intake.setCRollerVelocity(Constants.Intake.SlowCRollerVelocity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.setShooterVelocity(0.0);
    m_shooter.setHoodAngle(Constants.Shooter.hoodStowAngle);
    m_shooter.setIndexerVelocity(0);
    m_intake.setHopperRollerVelocity(0);
    m_intake.setABRollerVelocity(0);
    m_intake.setCRollerVelocity(0);
  }

  // Returns true when the command should end.
  // @Override
  // public boolean isFinished() {
  //   return true;
  // }
}
