package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class Intake extends Command {
  private final IntakeSubsystem m_intake;
  // private Timer m_placeTimer = new Timer();

  public Intake(
      IntakeSubsystem intakeSubsystem) {
    m_intake = intakeSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.setPosition(Constants.Intake.intakeFullDeployExtension);
    m_intake.setRollerVelocity(Constants.Intake.intakeRollerVelocity);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //System.out.println(m_arm.getArmAngle());
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.setPosition(Constants.Intake.intakeSmallDeployExtension);
    m_intake.setRollerVelocity(Constants.Intake.holdRollerVelocity);


  }

  // Returns true when the command should end.
  // @Override
  // public boolean isFinished() {
  //   return m_placeTimer.hasElapsed(0.30);
  // }
}
