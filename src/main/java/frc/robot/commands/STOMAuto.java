package frc.robot.commands;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainExtra;
import frc.robot.subsystems.SOTM;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Visualization;

public class STOMAuto extends Command {
  private final ShooterSubsystem m_shooter;
  private double distanceToTarget;
  private double hoodAngle;
  private double shooterVelocity;
  private Timer m_timer = new Timer();
  private int loopCount = 0;
  public double MaxAbsRotationalRate = 0.0;
  private ForwardPerspectiveValue ForwardPerspective = ForwardPerspectiveValue.OperatorPerspective;
  private PhoenixPIDController HeadingController = new PhoenixPIDController(Constants.Drive.PRotation, Constants.Drive.IRotation,  Constants.Drive.DRotation);
  // private SwerveControlParameters parameters = RobotContainer.drivetrain.getState()
  // private double starttime = 0.0;

  public STOMAuto(
      ShooterSubsystem shooterSubsystem) {
    m_shooter = shooterSubsystem;
     HeadingController.enableContinuousInput(-Math.PI, Math.PI);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.restart();
    // starttime = Utils.getCurrentTimeSeconds();

    loopCount = 0;
    System.out.println("STOM Auto Initialized");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SOTM.calcSOTM(Constants.ShotCalc.targetpose.getTranslation(), Constants.ShotCalc.ShotTime);
    distanceToTarget = SOTM.targetDistance();
    hoodAngle = Constants.ShotCalc.ShotAngle.get(distanceToTarget);
    shooterVelocity = Constants.ShotCalc.ShotVelocity.get(distanceToTarget);
    m_shooter.setHoodAngle(hoodAngle);
    m_shooter.setShooterVelocity(shooterVelocity);
    System.out.println("STOM Auto Distance: " + distanceToTarget + " Hood Angle: " + hoodAngle + " Shooter Velocity: " + shooterVelocity);
    if (loopCount % 10 == 0) {
    Visualization.LaunchFuelViz(shooterVelocity, Units.degreesToRadians(90)-hoodAngle);
    }

    PPHolonomicDriveController.overrideRotationFeedback(() -> {
        // Calculate feedback from your custom PID controller
    System.out.println("Auto Omega Override: " + AutoOmegaOverride(SOTM.targetAngleFeeds(), SOTM.targetangle()));
    return AutoOmegaOverride(SOTM.targetAngleFeeds(), SOTM.targetangle());
    });


    loopCount++;


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    PPHolonomicDriveController.clearRotationFeedbackOverride();
    m_shooter.setShooterVelocity(0.0);
    m_shooter.setHoodAngle(Constants.Shooter.hoodStowAngle);
  }

  // Returns true when the command should end.
  // @Override
  // public boolean isFinished() {
  //   return true;
  // }

    private double AutoOmegaOverride(AngularVelocity TargetRateFeedforward, Rotation2d TargetDirection) {
      Rotation2d angleToFace = TargetDirection;
            if (ForwardPerspective == ForwardPerspectiveValue.OperatorPerspective) {
                /* If we're operator perspective, rotate the direction we want to face by the angle */
                // angleToFace = angleToFace.rotateBy(parameters.operatorForwardDirection);
                angleToFace = angleToFace.rotateBy(RobotContainer.drivetrain.getOperatorForwardDirection());
            }

            // double toApplyOmega = TargetRateFeedforward.baseUnitMagnitude() +
            //     HeadingController.calculate(
            //         parameters.currentPose.getRotation().getRadians(),
            //         angleToFace.getRadians(),
            //         parameters.timestamp
            //     );
             double toApplyOmega = TargetRateFeedforward.baseUnitMagnitude() +
                HeadingController.calculate(
                    RobotContainer.drivetrain.getState().Pose.getRotation().getRadians(),
                    angleToFace.getRadians(),
                    Utils.getCurrentTimeSeconds()
                    // Utils.getCurrentTimeSeconds()-starttime
                );
            if (MaxAbsRotationalRate > 0.0) {
                if (toApplyOmega > MaxAbsRotationalRate) {
                    toApplyOmega = MaxAbsRotationalRate;
                } else if (toApplyOmega < -MaxAbsRotationalRate) {
                    toApplyOmega = -MaxAbsRotationalRate;
                }
            }
          return toApplyOmega;
          }


}
