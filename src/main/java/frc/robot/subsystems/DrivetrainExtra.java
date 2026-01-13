package frc.robot.subsystems;

import java.io.IOException;
import java.util.Optional;

import org.json.simple.parser.ParseException;
import org.photonvision.PhotonUtils;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import dev.doglog.internal.TimedCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.Pose;
import frc.robot.generated.TimedCommand2;

public class DrivetrainExtra {
    private static Optional<Alliance> m_ally = DriverStation.getAlliance();

    public static double getFieldSpeedsY(){
    return RobotContainer.drivetrain.getState().Speeds.vxMetersPerSecond * Math.sin(RobotContainer.drivetrain.getState().Pose.getRotation().getRadians()) + RobotContainer.drivetrain.getState().Speeds.vyMetersPerSecond * Math.cos(RobotContainer.drivetrain.getState().Pose.getRotation().getRadians());
    }

    public static double getFieldSpeedsX(){
        return RobotContainer.drivetrain.getState().Speeds.vxMetersPerSecond * Math.cos(RobotContainer.drivetrain.getState().Pose.getRotation().getRadians()) - RobotContainer.drivetrain.getState().Speeds.vyMetersPerSecond * Math.sin(RobotContainer.drivetrain.getState().Pose.getRotation().getRadians());
    }

    // Might need to be swapped idk.
    public static double getRobotAccelX(){
        return RobotContainer.drivetrain.getPigeon2().getAccelerationX().getValueAsDouble();
    }
    public static double getRobotAccelY(){
        return RobotContainer.drivetrain.getPigeon2().getAccelerationY().getValueAsDouble();
    }

    public static double getFieldAccelY(){
        return getRobotAccelX() * Math.sin(RobotContainer.drivetrain.getState().Pose.getRotation().getRadians()) + getRobotAccelY() * Math.cos(RobotContainer.drivetrain.getState().Pose.getRotation().getRadians());
    }

    public static double getFieldAccelX(){
        return getRobotAccelX() * Math.cos(RobotContainer.drivetrain.getState().Pose.getRotation().getRadians()) - getRobotAccelY() * Math.sin(RobotContainer.drivetrain.getState().Pose.getRotation().getRadians());
    }
    
     public static Rotation2d targetangle(Pose2d targetpose){
        /* First put the drivetrain into auto run mode, then run the auto */
        SwerveDriveState state = RobotContainer.drivetrain.getState();
        Pose2d pose = state.Pose;
        pose = new Pose2d(pose.getTranslation(), new Rotation2d(0));
        // Pose2d targetpose = new Pose2d(16.7,5.5,new Rotation2d(0));
        System.out.println(PhotonUtils.getYawToPose(pose,targetpose));
        return PhotonUtils.getYawToPose(pose,targetpose);
        
    }

    public static AngularVelocity targetAngleFeeds(Pose2d targetpose) {
        SwerveDriveState state = RobotContainer.drivetrain.getState();
        Pose2d pose = state.Pose;
        pose = new Pose2d(pose.getTranslation(), new Rotation2d(0));
        // Pose2d targetpose = new Pose2d(16.7,5.5,new Rotation2d(0));
        // Pose2d targetpose = Constants.ShotCalc.targetpose;
        double vx = getFieldSpeedsX();
        double vy = getFieldSpeedsY();
        double deltaX = targetpose.getX() - pose.getX();
        double deltaY = targetpose.getY() - pose.getY();
        double omega = -(vy * deltaX - vx * deltaY) / (deltaX * deltaX + deltaY * deltaY);
        return AngularVelocity.ofBaseUnits(omega, edu.wpi.first.units.Units.RadiansPerSecond);
    }

    public static double targetDistance(Pose2d targetpose) {
        /* Returns distance to target from target pose */
        SwerveDriveState state = RobotContainer.drivetrain.getState();
        Pose2d pose = state.Pose;
        // Pose2d targetpose = new Pose2d(16.7,5.5,new Rotation2d(0));
        // Pose2d targetpose = Constants.ShotCalc.targetpose;
        return pose.getTranslation().getDistance(targetpose.getTranslation());
    }

    public static Command LogTime(String key, Command command){
        return new TimedCommand2(command, key);
    }


    private static boolean zeroed = false;
    // Create the constraints to use while pathfinding
    
    public static Command pathfindingCommand(boolean left, boolean lineup) {

    double intercpet = Math.tan(Units.degreesToRadians(30))*4.5;
    double intercpetRed = Math.tan(Units.degreesToRadians(30))*13;
    double slope = Math.tan(Units.degreesToRadians(30));
    Pose2d targetPose = Constants.Pose.Error; // Example target pose
        
        PIDController xController = new PIDController(Constants.Pose.PTranslationSlow, Constants.Pose.ITranslationSlow, Constants.Pose.DTranslationSlow);
        xController.setIntegratorRange(-Constants.Pose.SpeedReductionFactor, Constants.Pose.SpeedReductionFactor);
        xController.setTolerance(Constants.Pose.Tolerance);
        PIDController yController = new PIDController(Constants.Pose.PTranslationSlow, Constants.Pose.ITranslationSlow, Constants.Pose.DTranslationSlow);
        yController.setIntegratorRange(-Constants.Pose.SpeedReductionFactor, Constants.Pose.SpeedReductionFactor);
        yController.setTolerance(Constants.Pose.Tolerance);
        PIDController thetaController = new PIDController(Constants.Pose.PRotationSlow, Constants.Pose.IRotationSlow, Constants.Pose.DRotationSlow);
        thetaController.enableContinuousInput(Units.degreesToRadians(-180),Units.degreesToRadians(180));
        thetaController.setTolerance(Constants.Pose.Tolerance);

        return new Command() {
            @Override
            public void initialize() {
                m_ally = DriverStation.getAlliance();
                // targetPose = getTargetPose(left);
                if (lineup){
                    xController.reset();
                    yController.reset();
                    thetaController.reset();
                }
            }
    
            @Override
            public void execute() {
                if (RobotContainer.driver.axisLessThan(0, Constants.Drive.DriveDeadband).getAsBoolean()&&RobotContainer.driver.axisLessThan(1, Constants.Drive.DriveDeadband).getAsBoolean()&&RobotContainer.driver.axisLessThan(4, Constants.Drive.RotationDeadband).getAsBoolean()&&RobotContainer.driver.axisLessThan(5, Constants.Drive.RotationDeadband).getAsBoolean()) {
                    zeroed = true;
                }
                if (lineup){
                    Pose2d currentPose = RobotContainer.drivetrain.getState().Pose;
                    // ChassisSpeeds currentSpeeds = RobotContainer.drivetrain.getState().Speeds;
                    double X = currentPose.getTranslation().getX();
                    double Y = currentPose.getTranslation().getY();
                    // double VX = currentSpeeds.vxMetersPerSecond;
                    // double VY = currentSpeeds.vyMetersPerSecond;
                    double xOutput =Constants.Pose.SpeedReductionFactor* RobotContainer.MaxSpeed * xController.calculate(X, targetPose.getX());
                    double yOutput =Constants.Pose.SpeedReductionFactor* RobotContainer.MaxSpeed * yController.calculate(Y, targetPose.getY());
                    double thetaOutput =Constants.Pose.SpeedReductionFactor* RobotContainer.MaxAngularRate * thetaController.calculate(currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());
                    
                    if (m_ally.get() == Alliance.Blue){
                        RobotContainer.drivetrain.applyRequest(() -> 
                            RobotContainer.drive.withVelocityX(xOutput)
                                 .withVelocityY(yOutput)
                                 .withRotationalRate(thetaOutput)
                        ).execute();
                    } else {
                        RobotContainer.drivetrain.applyRequest(() -> 
                        RobotContainer.drive.withVelocityX(-xOutput)
                                 .withVelocityY(-yOutput)
                                 .withRotationalRate(thetaOutput)   
                        ).execute();
                    }
                }
            }
    
            @Override
            public void end(boolean interrupted) {
                xController.close();
                yController.close();
                thetaController.close();
            }
    
            @Override
            public boolean isFinished() {
                return !lineup||
                (xController.atSetpoint() && yController.atSetpoint() && thetaController.atSetpoint()||
                (zeroed&&RobotContainer.driver.axisMagnitudeGreaterThan(0, Constants.Drive.DriveDeadband).getAsBoolean()||RobotContainer.driver.axisMagnitudeGreaterThan(1, Constants.Drive.DriveDeadband).getAsBoolean()||RobotContainer.driver.axisMagnitudeGreaterThan(4, Constants.Drive.RotationDeadband).getAsBoolean()||RobotContainer.driver.axisMagnitudeGreaterThan(5, Constants.Drive.RotationDeadband).getAsBoolean()));
            }
        };
    }

    
     public static Command pathfindingtofollowCommand() {
        // Since we are using a holonomic drivetrain, the rotation component of this pose
        // represents the goal holonomic rotation
        PathPlannerPath path = null;
        try {
            path = PathPlannerPath.fromPathFile("Testpath");
        } catch (FileVersionException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        } catch (ParseException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }

        // Create the constraints to use while pathfinding
        PathConstraints constraints = new PathConstraints(
                4.0, 4.0,
                Units.degreesToRadians(540), Units.degreesToRadians(720));

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        return AutoBuilder.pathfindThenFollowPath(
                path,
                constraints
                 
        );
    }
    
}
