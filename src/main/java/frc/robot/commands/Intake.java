package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSubsystem;

public class Intake extends Command {
  private final IntakeSubsystem m_intake;
  private boolean m_lastAttackMode = false;
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

    m_intake.setIntakePosition();
    if (m_intake.getAttackMode()){
      m_intake.setABRollerVelocity(Constants.Intake.intakeABRollerVelocity);
      m_intake.setCRollerVelocity(Constants.Intake.intakeCRollerVelocity);
    } else {
      m_intake.setABRollerVelocity(Constants.Intake.intakeABRollerVelocity);
      m_intake.setCRollerVelocity(Constants.Intake.SlowCRollerVelocity);
    }
    if (!(RobotContainer.driver.leftBumper().getAsBoolean()||RobotContainer.operator.rightBumper().getAsBoolean()||RobotContainer.operator.leftTrigger().getAsBoolean())){
    RobotContainer.hopper.setHopperRollerVelocity(Constants.Hopper.hopperIntakeVelocity);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (m_intake.getAttackMode() != m_lastAttackMode){
      if (m_intake.getAttackMode()){
        m_intake.setABRollerVelocity(Constants.Intake.intakeABRollerAttackVelocity);
        m_intake.setCRollerVelocity(Constants.Intake.intakeCRollerVelocity);
      } else {
        m_intake.setABRollerVelocity(Constants.Intake.intakeABRollerVelocity);
        m_intake.setCRollerVelocity(Constants.Intake.SlowCRollerVelocity);
      }
      m_lastAttackMode = m_intake.getAttackMode();
    }


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.setABRollerVelocity(Constants.Intake.holdRollerVelocity);
    m_intake.setCRollerVelocity(Constants.Intake.holdRollerVelocity);
    if (!(RobotContainer.driver.leftBumper().getAsBoolean()||RobotContainer.operator.rightBumper().getAsBoolean()||RobotContainer.operator.leftTrigger().getAsBoolean())){
    RobotContainer.hopper.setHopperRollerVelocity(0);
    }

  }

  // Returns true when the command should end.
  // @Override
  // public boolean isFinished() {
  //   return m_placeTimer.hasElapsed(0.30);
  // }
}
