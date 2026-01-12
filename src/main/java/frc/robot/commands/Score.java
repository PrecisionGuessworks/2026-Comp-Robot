package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class Score extends Command {
  private final ClimberSubsystem m_elevator;
  private final ShooterSubsystem m_arm;

  public Score(
      ClimberSubsystem elevatorSubsystem,
      ShooterSubsystem armSubsystem) {
    m_elevator = elevatorSubsystem;
    m_arm = armSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevatorSubsystem, armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_elevator.setHeight(Constants.Elevator.stowHeight);
    m_arm.setArmAngle(Constants.Arm.armStowAngle);
    m_arm.setWristAngle(Constants.Arm.wristStowAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_arm.setAmpFeederVelocity(Constants.Arm.ampShootVelocity,Constants.Arm.feederShootVelocity);
      m_Timer.restart();
      if(!Robot.isReal()){
        ShotVelocity = Constants.Arm.quickShootVelocity * Constants.Arm.WheelRadius * Constants.Shotefficiency;  
            Robot.updateNoteViz(new Pose3d(RobotContainer.drivetrain.getState().Pose.getX(),RobotContainer.drivetrain.getState().Pose.getY(),0.4, new Rotation3d(0,-Constants.Arm.armShootAngle,RobotContainer.drivetrain.getState().Pose.getRotation().getRadians())), 
            new double[] {RobotContainer.drivetrain.getFieldSpeedsX() + ShotVelocity * Math.cos(Constants.Arm.armShootAngle)*Math.cos(RobotContainer.drivetrain.getState().Pose.getRotation().getRadians()),
              RobotContainer.drivetrain.getFieldSpeedsY() + ShotVelocity * Math.cos(Constants.Arm.armShootAngle)*Math.sin(RobotContainer.drivetrain.getState().Pose.getRotation().getRadians()), 
              ShotVelocity * Math.sin(Constants.Arm.armShootAngle) });
        }
  }

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
