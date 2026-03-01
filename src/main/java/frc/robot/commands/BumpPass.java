package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Visualization;

public class BumpPass extends Command {
  private final ShooterSubsystem m_shooter;
  private final IntakeSubsystem m_intake;
  private Timer m_timer = new Timer();
  private int loopCount = 0;

  public BumpPass(
      ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem) {
    m_shooter = shooterSubsystem;
    m_intake = intakeSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsystem, intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.setHoodAngle(Constants.Shooter.hoodBumpPassAngle);
    m_shooter.setShooterVelocity(Constants.Shooter.ShooterBumpPassVelocity);
    m_timer.restart();
    loopCount = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if(m_timer.get()>0.5){
          m_shooter.setIndexerVelocity(Constants.Shooter.indexerVelocity);
    m_intake.setHopperRollerVelocity(Constants.Intake.hopperVelocity);

    if (loopCount % 10 == 0) {
    Visualization.LaunchFuelViz(Constants.Shooter.indexerVelocity, Units.degreesToRadians(90)-Constants.Shooter.hoodBumpPassAngle);
    }
    }
    loopCount++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.setShooterVelocity(0.0);
    m_shooter.setHoodAngle(Constants.Shooter.hoodStowAngle);
    m_shooter.setIndexerVelocity(0);
    m_intake.setHopperRollerVelocity(0);
  }

  // Returns true when the command should end.
  // @Override
  // public boolean isFinished() {
  //   return true;
  // }
}
