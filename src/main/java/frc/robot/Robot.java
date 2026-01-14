package frc.robot;

import java.io.IOException;
import java.text.ParseException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;

import dev.doglog.DogLog;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.generated.Elastic;
import frc.robot.generated.LimelightHelpers;
import frc.robot.generated.Vision;
import frc.robot.subsystems.Visualization;


public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  private String autoName, newAutoName;

  private boolean elevatorOn = false;
  private boolean lineup = false;

  private final Field2d m_field = new Field2d();
  
  Optional<Alliance> ally = DriverStation.getAlliance();
  Optional<Alliance> newAlly;
  private Vision vision;

  public Robot() {
    m_robotContainer = new RobotContainer();
    vision = new Vision();
    SignalLogger.enableAutoLogging(false); // Disable CTRE Signal Logger auto logging
    LimelightHelpers.SetIMUMode(Constants.Vision.LimeLightCamerName, 1);
        
  }

  // @Override
  // public void robotInit(){
  // PathfindingCommand.warmupCommand().schedule();
  // }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    Visualization.Update3DVisualization();
    Visualization.updateFuelViz();

    // OLD LL vision code
    // var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
    // if (llMeasurement != null) {
    //  RobotContainer.drivetrain.addVisionMeasurement(llMeasurement.pose, Utils.fpgaToCurrentTime(llMeasurement.timestampSeconds));
    // }

    // First, tell Limelight your robot's current orientation
      double robotYaw = RobotContainer.drivetrain.getState().Pose.getRotation().getDegrees(); // CHECK !!!  
      LimelightHelpers.SetRobotOrientation(Constants.Vision.LimeLightCamerName, robotYaw, 0.0, 0.0, 0.0, 0.0, 0.0);
      
      // Get the pose estimate
      LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue(Constants.Vision.LimeLightCamerName);

      // Add it to your pose estimator
      RobotContainer.drivetrain.setVisionMeasurementStdDevs(Constants.Vision.LLTagStdDevs);
      if (limelightMeasurement != null){
      RobotContainer.drivetrain.addVisionMeasurement(
          limelightMeasurement.pose,
          limelightMeasurement.timestampSeconds
      );
      }


    try{
      var visionEst = vision.getEstimatedGlobalPose();
    visionEst.ifPresent(
            est -> {
                // Change our trust in the measurement based on the tags we can see
                var estStdDevs = vision.getEstimationStdDevs();

                RobotContainer.drivetrain.addVisionMeasurement(
                        est.estimatedPose.toPose2d(), Utils.fpgaToCurrentTime(est.timestampSeconds), estStdDevs);
            });
    } catch (Exception e) {
      e.printStackTrace();
    }

  if(RobotContainer.driver.back().getAsBoolean()) {
    lineup = true;
  }


  
  // double leftY = m_robotContainer.operator.getLeftY();
  // if (Math.abs(leftY) > 0.1) { // Deadband of 0.1
  //   if (m_robotContainer.operator.leftBumper().getAsBoolean() == true) {
  //     RobotContainer.climber.setTargetAdjust(leftY);
  //   }
  // }

    // if (RobotContainer.operator.rightStick().getAsBoolean() == true) {
    //   double rightyop = RobotContainer.operator.getRightY();
    //   RobotContainer.shooter.setShooterVelocity(rightyop*40);
    // } else {
    //   RobotContainer.shooter.setShooterVelocity(0);
    // }
  
  
  
}

  @Override
  public void disabledInit() {
    autoName = "";

    Command resetGryo = new Command()
    {
        public boolean runsWhenDisabled()
        {
            return true;
        }

        public void initialize()
        {
            RobotContainer.drivetrain.getPigeon2().reset();
        }
        public boolean isFinished()
        {
            return true;
        }
    };
 
    SmartDashboard.putData("Reset Gyro", resetGryo);
  }

  @Override
  public void disabledPeriodic() { 
    updateElasticField();
    
}
  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    LimelightHelpers.SetIMUMode(Constants.Vision.LimeLightCamerName, 2);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    autoName = "";
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    
  }

  @Override
  public void autonomousPeriodic() {
    updateElasticField();
  }

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    LimelightHelpers.SetIMUMode(Constants.Vision.LimeLightCamerName, 2);
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    SwerveDriveState state = RobotContainer.drivetrain.getState();
    Pose2d pose = state.Pose;
    m_field.getObject("path").setPoses();
    m_field.setRobotPose(pose);
    SmartDashboard.putData(m_field);
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {
// Update drivetrain simulation

SwerveDriveState state = RobotContainer.drivetrain.getState();
Pose2d pose = state.Pose;
// Update camera simulation
vision.simulationPeriodic(pose);

var debugField = vision.getSimDebugField();
debugField.getObject("EstimatedRobot").setPose(pose);
Visualization.Update2DVisualization();


  }

  public void updateElasticField() {
    ally = DriverStation.getAlliance();
    newAutoName = m_robotContainer.getAutonomousCommand().getName();
    if (autoName != newAutoName | ally != newAlly) {
        newAlly = ally;
        autoName = newAutoName;
        if (AutoBuilder.getAllAutoNames().contains(autoName)) {
            System.out.println("Displaying " + autoName);
            try {
                List<PathPlannerPath> pathPlannerPaths = PathPlannerAuto.getPathGroupFromAutoFile(autoName);
                List<Pose2d> poses = new ArrayList<>();
                for (PathPlannerPath path : pathPlannerPaths) {
                        if (ally.isPresent()) {
                          if (ally.get() == Alliance.Red) {
                            poses.addAll(path.getAllPathPoints().stream()
                            .map(point -> new Pose2d(Constants.Pose.feildFlip - point.position.getX(),Constants.Pose.feildFlipy - point.position.getY(), new Rotation2d()))
                          .collect(Collectors.toList()));
                          Elastic.selectTab("RED");
                          }
                          if (ally.get() == Alliance.Blue) {
                            poses.addAll(path.getAllPathPoints().stream()
                            .map(point -> new Pose2d(point.position.getX(), point.position.getY(), new Rotation2d()))
                          .collect(Collectors.toList()));
                          Elastic.selectTab("BLUE");
                          }
                        }
                        else {
                            System.out.println("No alliance found");
                            poses.addAll(path.getAllPathPoints().stream()
                            .map(point -> new Pose2d(point.position.getX(), point.position.getY(), new Rotation2d()))
                          .collect(Collectors.toList()));
                        }
                }
              
                m_field.getObject("path").setPoses(poses);
            } catch (IOException e) {
                e.printStackTrace();
            } catch (Exception e) {
                if (e instanceof ParseException) {
                    e.printStackTrace();
                } else {
                  e.printStackTrace();
                }
            }
        }
    }
    SwerveDriveState state = RobotContainer.drivetrain.getState();
    Pose2d pose = state.Pose;
    m_field.setRobotPose(pose);
    SmartDashboard.putData(m_field);
  }

}
