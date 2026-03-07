package frc.robot.commands.Auto;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Visualization;
import frc.robot.subsystems.HopperSubsystem;

public class AutoBumpScoreAngle extends Command {
  private final ShooterSubsystem m_shooter;
  private final HopperSubsystem m_hopper;
  private Timer m_timer = new Timer();
  private int loopCount = 0;

  public AutoBumpScoreAngle(
      ShooterSubsystem shooterSubsystem, HopperSubsystem hopperSubsystem) {
    m_shooter = shooterSubsystem;
    m_hopper = hopperSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsystem, hopperSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.setHoodAngle(Constants.Shooter.ShooterBumpVelocityAngle);
    m_shooter.setShooterVelocity(Constants.Shooter.ShooterBumpVelocityAngle);
    m_timer.restart();
    loopCount = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if(m_timer.get()>0.5){
          m_shooter.setIndexerVelocity(Constants.Shooter.indexerVelocity);
    m_hopper.setHopperRollerVelocity(Constants.Hopper.hopperVelocity);

    if (loopCount % 10 == 0) {
    Visualization.LaunchFuelViz(Constants.Shooter.ShooterBumpVelocity, Units.degreesToRadians(90)-Constants.Shooter.hoodBumpAngle);
    }
    }
    loopCount++;
    RobotContainer.intake.retractIntakeSlowShoot();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.setShooterVelocity(0.0);
    m_shooter.setHoodAngle(Constants.Shooter.hoodStowAngle);
    m_shooter.setIndexerVelocity(0);
    m_hopper.setHopperRollerVelocity(0);
    RobotContainer.intake.retractIntakeSlowShootSTOP();
  }

  // Returns true when the command should end.
  // @Override
  // public boolean isFinished() {
  //   return true;
  // }
}
