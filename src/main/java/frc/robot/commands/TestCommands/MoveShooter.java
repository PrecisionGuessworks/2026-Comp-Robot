package frc.robot.commands.TestCommands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;


public class MoveShooter extends Command {

  private final ShooterSubsystem m_shooter;
  // private Timer m_ejectTimer = new Timer();

  public MoveShooter(ShooterSubsystem shooter) {

    m_shooter = shooter;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.setHoodAngle(Units.degreesToRadians(10));
    
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.setHoodAngle(Units.degreesToRadians(0));
    
  }

  // Returns true when the command should end.
  // @Override
  // public boolean isFinished() {
  //   return m_ejectTimer.get() > 0.35;
  // }
}
