package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainExtra;
import frc.robot.subsystems.SOTM;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Visualization;

public class STOMScore extends Command {
  private final ShooterSubsystem m_shooter;
  private double distanceToTarget;
  private double hoodAngle;
  private double shooterVelocity;
  private Timer m_timer = new Timer();
  private int loopCount = 0;

  public STOMScore(
      ShooterSubsystem shooterSubsystem) {
    m_shooter = shooterSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    distanceToTarget = SOTM.targetDistance();
    hoodAngle = Constants.ShotCalc.Angle.get(distanceToTarget);
    shooterVelocity = Constants.ShotCalc.Velocity.get(distanceToTarget);
    m_shooter.setHoodAngle(hoodAngle);
    m_shooter.setShooterVelocity(shooterVelocity);
    m_timer.restart();
    loopCount = 0;
    SOTM.calcSOTM();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    distanceToTarget = SOTM.targetDistance();
    hoodAngle = Constants.ShotCalc.Angle.get(distanceToTarget);
    shooterVelocity = Constants.ShotCalc.Velocity.get(distanceToTarget);
    m_shooter.setHoodAngle(hoodAngle);
    m_shooter.setShooterVelocity(shooterVelocity);
    if (loopCount % 10 == 0) {
    Visualization.LaunchFuelViz(shooterVelocity, Units.degreesToRadians(90)-hoodAngle);
    }
    loopCount++;
    SOTM.calcSOTM();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.setShooterVelocity(0.0);
    m_shooter.setHoodAngle(Constants.Shooter.hoodStowAngle);
  }

  // Returns true when the command should end.
  // @Override
  // public boolean isFinished() {
  //   return true;
  // }
}
