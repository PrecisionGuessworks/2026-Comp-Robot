package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class AutoIntakeRoller extends Command {
  private final IntakeSubsystem m_intake;
  // private Timer m_placeTimer = new Timer();

  public AutoIntakeRoller(
      IntakeSubsystem intakeSubsystem) {
    m_intake = intakeSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.setABRollerVelocity(Constants.Intake.intakeABRollerVelocity);
    m_intake.setCRollerVelocity(Constants.Intake.intakeCRollerVelocity);
    // m_intake.setIntakePosition();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.setABRollerVelocity(Constants.Intake.holdRollerVelocity);
    m_intake.setCRollerVelocity(Constants.Intake.holdRollerVelocity);

  }

  // Returns true when the command should end.
  // @Override
  // public boolean isFinished() {
  //   return m_placeTimer.hasElapsed(0.30);
  // }
}
