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
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.path.PathPlannerPath;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.generated.Elastic;
import frc.robot.generated.LimelightHelpers;
import frc.robot.generated.ShiftHelpers;
import frc.robot.generated.Vision;
import frc.robot.subsystems.Visualization;


public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  private String autoName, newAutoName;

  private final Field2d m_field = new Field2d();

  private Timer m_RewindTimer = new Timer();
  
  Optional<Alliance> ally = DriverStation.getAlliance();
  Optional<Alliance> newAlly;
  private Vision vision;
  private boolean UseLimeLightCamera2 = false; // Initialize as false or set to true as needed

  // public static final Lights lights = new Lights();

  public Robot() {
    m_robotContainer = new RobotContainer();
    
    vision = new Vision();
    SignalLogger.enableAutoLogging(false); // Disable CTRE Signal Logger auto logging
    SignalLogger.stop(); // Stop any existing logging sessions
    LimelightHelpers.SetIMUMode(Constants.Vision.LimeLightCamerName, 1);
    LimelightHelpers.SetIMUAssistAlpha(Constants.Vision.LimeLightCamerName, 0.001);
    if (Constants.Vision.UseLimeLightCamera2) {
      LimelightHelpers.SetIMUMode(Constants.Vision.LimeLightCamerName2, 1);
      LimelightHelpers.SetIMUAssistAlpha(Constants.Vision.LimeLightCamerName2, 0.001);
    }
    PathfindingCommand.warmupCommand().schedule();
        
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
    updateFeildTimers();
    LimelightHelpers.setRewindEnabled(Constants.Vision.LimeLightCamerName, Constants.UseRewind);
    if (Constants.Vision.UseLimeLightCamera2) {
      LimelightHelpers.setRewindEnabled(Constants.Vision.LimeLightCamerName2, Constants.UseRewind);
    }
    
    // if (!DriverStation.isDSAttached()) {
    //   lights.setNotConnected();
    // }

    // OLD LL vision code
    // var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
    // if (llMeasurement != null) {
    //  RobotContainer.drivetrain.addVisionMeasurement(llMeasurement.pose, Utils.fpgaToCurrentTime(llMeasurement.timestampSeconds));
    // }

    // First, tell Limelight your robot's current orientation
      double robotYaw = RobotContainer.drivetrain.getState().Pose.getRotation().getDegrees(); // CHECK !!!  
      LimelightHelpers.SetRobotOrientation(Constants.Vision.LimeLightCamerName, robotYaw, 0.0, 25.0, 0.0, 0.0, 0.0);
      
      // Get the pose estimate
      LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue(Constants.Vision.LimeLightCamerName);
      LimelightHelpers.PoseEstimate limelightMeasurement2 = null;
      if (Constants.Vision.UseLimeLightCamera2) {
        LimelightHelpers.SetRobotOrientation(Constants.Vision.LimeLightCamerName2, robotYaw, 0.0, 20.0, 0.0, 0.0, 0.0);
        limelightMeasurement2 = LimelightHelpers.getBotPoseEstimate_wpiBlue(Constants.Vision.LimeLightCamerName2);
      }
      // System.out.println(limelightMeasurement);
      // Add it to your pose estimator
      // RobotContainer.drivetrain.setVisionMeasurementStdDevs(Constants.Vision.LLTagStdDevs);
      // if (limelightMeasurement != null){
      // RobotContainer.drivetrain.addVisionMeasurement(
      //     limelightMeasurement.pose,
      //     Utils.fpgaToCurrentTime(limelightMeasurement.timestampSeconds)
      // );
      // }
      
      // DogLog.log("Vision: Robot Yaw", LimelightHelpers.);
      //  LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
      boolean doRejectUpdate = false;
      if (limelightMeasurement != null) {
      DogLog.log("Vision: Limelight Measurement", limelightMeasurement.pose);
      DogLog.log("Vision: Limelight Measurement Timestamp", LimelightHelpers.getBotPose_wpiBlue(Constants.Vision.LimeLightCamerName));

      if(limelightMeasurement.tagCount == 1 && limelightMeasurement.rawFiducials.length == 1)
      {
        if(limelightMeasurement.rawFiducials[0].ambiguity > .76)
        {
          doRejectUpdate = true;
        }
        if(limelightMeasurement.rawFiducials[0].distToCamera > 3)
        {
          doRejectUpdate = true;
        }
      }
      if(limelightMeasurement.tagCount == 0)
      {
        doRejectUpdate = true;
      }

      if(!doRejectUpdate)
      {
        RobotContainer.drivetrain.setVisionMeasurementStdDevs(Constants.Vision.LLTagStdDevs.times(limelightMeasurement.rawFiducials[0].distToCamera/2));
        RobotContainer.drivetrain.addVisionMeasurement(
          limelightMeasurement.pose,
          Utils.fpgaToCurrentTime(limelightMeasurement.timestampSeconds)
      );
      }
    }
    boolean doRejectUpdate2 = false;
            if (limelightMeasurement2 != null) {
        DogLog.log("Vision: Limelight Measurement 2", limelightMeasurement2.pose);
        DogLog.log("Vision: Limelight Measurement 2 Timestamp", LimelightHelpers.getBotPose_wpiBlue(Constants.Vision.LimeLightCamerName2));

      if(limelightMeasurement2.tagCount == 1 && limelightMeasurement2.rawFiducials.length == 1)
      {
        if(limelightMeasurement2.rawFiducials[0].ambiguity > .76)
        {
          doRejectUpdate2 = true;
        }
        if(limelightMeasurement2.rawFiducials[0].distToCamera > 3)
        {
          doRejectUpdate2 = true;
        }
      }
      if(limelightMeasurement2.tagCount == 0)
      {
        doRejectUpdate2 = true;
      }

      if(!doRejectUpdate2)
      {
        RobotContainer.drivetrain.setVisionMeasurementStdDevs(Constants.Vision.LLTagStdDevs.times(limelightMeasurement2.rawFiducials[0].distToCamera/2));
        RobotContainer.drivetrain.addVisionMeasurement(
          limelightMeasurement2.pose,
          Utils.fpgaToCurrentTime(limelightMeasurement2.timestampSeconds)
      );
      }
    }


    // try{
    //   var visionEst = vision.getEstimatedGlobalPose();
    // visionEst.ifPresent(
    //         est -> {
    //             // Change our trust in the measurement based on the tags we can see
    //             var estStdDevs = vision.getEstimationStdDevs();

    //             RobotContainer.drivetrain.addVisionMeasurement(
    //                     est.estimatedPose.toPose2d(), Utils.fpgaToCurrentTime(est.timestampSeconds), estStdDevs);
    //         });
    // } catch (Exception e) {
    //   e.printStackTrace();
    // }

  // if(RobotContainer.driver.back().getAsBoolean()) {
  //   lineup = true;
  // }

  


  
  // double leftY = m_robotContainer.operator.getLeftY();
  // if (Math.abs(leftY) > 0.1) { // Deadband of 0.1
  //   if (m_robotContainer.operator.leftBumper().getAsBoolean() == true) {
  //     RobotContainer.climber.setTargetAdjust(leftY);
  //   }
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
    if (DriverStation.isDSAttached()) {
    // lights.setConnectedAlliance();
    }

    if (Constants.UseRewind) {
      
       if (m_RewindTimer.hasElapsed(0.5)) {
        m_RewindTimer.reset();
        LimelightHelpers.triggerRewindCapture(Constants.Vision.LimeLightCamerName,165);
        if (Constants.Vision.UseLimeLightCamera2) {
         LimelightHelpers.triggerRewindCapture(Constants.Vision.LimeLightCamerName2,165);
        }
      }
    }
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
  
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    autoName = "";
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    LimelightHelpers.SetIMUMode(Constants.Vision.LimeLightCamerName, 4);
    if (Constants.Vision.UseLimeLightCamera2) {
      LimelightHelpers.SetIMUMode(Constants.Vision.LimeLightCamerName2, 4);
    }
  }

  @Override
  public void autonomousPeriodic() {
    updateElasticField();
  }

  @Override
  public void autonomousExit() {
        if (Constants.UseRewind) {
        LimelightHelpers.triggerRewindCapture(Constants.Vision.LimeLightCamerName,21);
        if (Constants.Vision.UseLimeLightCamera2) {
         LimelightHelpers.triggerRewindCapture(Constants.Vision.LimeLightCamerName2,21);
        }
    }
  }

  @Override
  public void teleopInit() {
    // LimelightHelpers.SetIMUMode(Constants.Vision.LimeLightCamerName, 4);
    // if (Constants.Vision.UseLimeLightCamera2) {
    //   LimelightHelpers.SetIMUMode(Constants.Vision.LimeLightCamerName2, 4);
    // }
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
  public void teleopExit() {

    // if (Constants.UseRewind) {
    //   m_RewindTimer.start();
    // }
    if (Constants.UseRewind) {
        LimelightHelpers.triggerRewindCapture(Constants.Vision.LimeLightCamerName,165);
        if (Constants.Vision.UseLimeLightCamera2) {
         LimelightHelpers.triggerRewindCapture(Constants.Vision.LimeLightCamerName2,165);
        }
    }

  }

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
  private final Color Red = new Color(255, 0, 0);
  private final Color Blue = new Color(0, 0, 255);
  private final Color Black = new Color(0, 0, 0);
  private final Color Green = new Color(0, 255, 0);
  private Color m_currentColor = Blue;
  private Color m_weWonAuto = Black;
  private Color m_OurShift = Black;

  public void updateFeildTimers() {
    double matchTime = DriverStation.getMatchTime();
    boolean WeBlue = ShiftHelpers.isCurrentShiftBlue(matchTime);
    boolean WeWon = false;

    SmartDashboard.putNumber("Match Time", matchTime);
    if (ShiftHelpers.blueWonAuto()) {
      m_currentColor = Blue;
    } else {
      m_currentColor = Red;
    }

    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
      if (ShiftHelpers.blueWonAuto()&&ShiftHelpers.isDataReady()) {
        m_weWonAuto = Green;
      }
      else {
        m_weWonAuto = Red;
      }
    }
    else {
      if (!ShiftHelpers.blueWonAuto()&&ShiftHelpers.isDataReady()) {
        m_weWonAuto = Green;
      }
      else {
        m_weWonAuto = Red;
      }
    }
    if (!ShiftHelpers.isDataReady()) {
      m_weWonAuto = Black;
    }

    SmartDashboard.putString("Won Auto", m_currentColor.toHexString());
    SmartDashboard.putString("We Won Auto", m_weWonAuto.toHexString());
    DogLog.log("Match Info: Match Time", matchTime);
    DogLog.log("Match Info: Current Shift Color", m_currentColor.toHexString());
    SmartDashboard.putNumber("Shift Time", ShiftHelpers.timeLeftInShiftSeconds(matchTime));    
    SmartDashboard.putNumber("Time",matchTime);
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
      // SmartDashboard.putBoolean("Our Shift", ShiftHelpers.isCurrentShiftBlue(matchTime));
      if (ShiftHelpers.isCurrentShiftBlue(matchTime)){
        m_OurShift = Green;
      } else {
        m_OurShift = Black;
      }
      SmartDashboard.putString("Our Shift", m_OurShift.toHexString());
    } else {
      // SmartDashboard.putBoolean("Our Shift", ShiftHelpers.isCurrentShiftRed(matchTime));
      if (ShiftHelpers.isCurrentShiftRed(matchTime)){
        m_OurShift = Green;
      } else {
        m_OurShift = Black;
      }
      SmartDashboard.putString("Our Shift", m_OurShift.toHexString());
    }
  }

  public void updateElasticField() {
    ally = DriverStation.getAlliance();
    newAutoName = m_robotContainer.getAutonomousCommand().getName();
    if (autoName != newAutoName | ally != newAlly) {
        newAlly = ally;
        autoName = newAutoName;
        DogLog.log("Commands: Auto: Name", autoName);
        DogLog.log("Commands: Auto: Alliance", ally.isPresent() ? ally.get().toString() : "None");
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
