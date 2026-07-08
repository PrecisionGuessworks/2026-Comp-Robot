package frc.robot.commands.Shoot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Visualization;

public class BumpPass extends Command {
  private final ShooterSubsystem m_shooter;
  private final HopperSubsystem m_hopper;
  private Timer m_timer = new Timer();
  private int loopCount = 0;
  private boolean ShootReady = false;

  public BumpPass(
      ShooterSubsystem shooterSubsystem, HopperSubsystem hopperSubsystem) {
    m_shooter = shooterSubsystem;
    m_hopper = hopperSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsystem, hopperSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.setHoodAngle(Constants.Shooter.hoodBumpPassAngle);
    m_shooter.setShooterVelocity(Constants.Shooter.ShooterBumpPassVelocity);
    m_timer.restart();
    loopCount = 0;
    ShootReady = m_shooter.getShooterVelocity()>200;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if(m_timer.get()>0.5||ShootReady){
    m_shooter.setIndexerVelocity(Constants.Shooter.indexerVelocity);
    m_hopper.setHopperRollerVelocity(Constants.Hopper.hopperVelocity);

    if (loopCount % 10 == 0) {
    Visualization.LaunchFuelViz(Constants.Shooter.indexerVelocity, Units.degreesToRadians(90)-Constants.Shooter.hoodBumpPassAngle);
    }
    }
    loopCount++;
    // RobotContainer.intake.retractIntakeSlowShoot();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.setShooterVelocity(0.0);
    m_shooter.setHoodAngle(Constants.Shooter.hoodStowAngle);
    m_shooter.setIndexerVelocity(0);
    m_hopper.setHopperRollerVelocity(0);
    // RobotContainer.intake.retractIntakeSlowShootSTOP();
  }

  // Returns true when the command should end.
  // @Override
  // public boolean isFinished() {
  //   return true;
  // }
}
