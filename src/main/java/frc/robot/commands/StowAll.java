package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class StowAll extends Command {
  private final ClimberSubsystem m_climber;
  private final ShooterSubsystem m_shooter;

  public StowAll(
      ClimberSubsystem climberSubsystem,
      ShooterSubsystem shooterSubsystem) {
    m_climber = climberSubsystem;
    m_shooter = shooterSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climberSubsystem, shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_climber.setHeight(Constants.Climber.stowHeight);
    m_shooter.setHoodAngle(Constants.Shooter.hoodStowAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  // @Override
  // public void execute() {
  //   System.out.println(m_arm.getArmAngle());
  // }

  // Called once the command ends or is interrupted.
  // @Override
  // public void end(boolean interrupted) {
  //   m_arm.setArmAngle(Constants.Arm.armStowAngle);
  //   m_arm.setWristAngle(Constants.Arm.wristStowAngle);
  //   m_elevator.setHeight(Constants.Elevator.stowHeight);
  // }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
