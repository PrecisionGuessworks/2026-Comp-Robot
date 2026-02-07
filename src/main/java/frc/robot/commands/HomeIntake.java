package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class HomeIntake extends Command {
  private final IntakeSubsystem m_intake;
  // private Timer m_placeTimer = new Timer();

  public HomeIntake(
      IntakeSubsystem intakeSubsystem) {
    m_intake = intakeSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.setABRollerVelocity(0);
    m_intake.setCRollerVelocity(0);
    m_intake.setSoftLimitsEnabled(false);
    m_intake.setDeployCurrent(Constants.Intake.retractHomeStatorCurrent, Constants.Intake.retractHomeSupplyCurrent);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.retractIntakeHome();


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.zeroDeploy();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_intake.isDeployStalled();
  }
}
