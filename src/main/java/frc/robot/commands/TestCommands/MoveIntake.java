package frc.robot.commands.TestCommands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;


public class MoveIntake extends Command {

  private final IntakeSubsystem m_intake;
  // private Timer m_ejectTimer = new Timer();

  public MoveIntake(IntakeSubsystem intake) {

    m_intake = intake;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.setPosition(Units.inchesToMeters(2));
    
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.setPosition(Units.inchesToMeters(0));

  }

  // Returns true when the command should end.
  // @Override
  // public boolean isFinished() {
  //   return m_ejectTimer.get() > 0.35;
  // }
}
