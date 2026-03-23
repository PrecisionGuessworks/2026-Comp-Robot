package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class HooperOuttake extends Command {
  private final HopperSubsystem m_hopper;
  private final ShooterSubsystem m_shooter;

  private boolean m_lastAttackMode = false;
  // private Timer m_placeTimer = new Timer();

  public HooperOuttake(
      HopperSubsystem hopperSubsystem, ShooterSubsystem shooterSubsystem) {
    m_hopper = hopperSubsystem;
    m_shooter = shooterSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(hopperSubsystem, shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_shooter.setIndexerVelocity(-Constants.Shooter.indexerVelocity);
    m_hopper.setHopperRollerVelocity(-Constants.Hopper.hopperIntakeVelocity);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_hopper.setHopperRollerVelocity(0);
    m_shooter.setIndexerVelocity(0);

  }

  // Returns true when the command should end.
  // @Override
  // public boolean isFinished() {
  //   return m_placeTimer.hasElapsed(0.30);
  // }
}
