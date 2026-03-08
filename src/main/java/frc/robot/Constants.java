package frc.robot;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;

import dev.doglog.DogLog;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.quixlib.devices.CANDeviceID;
import frc.quixlib.motorcontrol.MechanismRatio;
import frc.quixlib.motorcontrol.PIDConfig;
import frc.robot.generated.Elastic;



public class Constants {
    // CANID's:
    //
    // Drivetrain 1-19
    // - 4 drive, 4 steer
    // Shooter / Indexer 20-29 
    // - 2 hood, 2 shooter, 1 indexer
    // Intake / Hopper 30-39
    // - 1 intake deploy, 2 intake roller, 1 hopper
    // Climber 40-49
    // - 2 climber motors
    


    // "rio" for rio bus
    public static final String kDriveTrainCanivoreName = "driveTrain"; // need to update in tuner generated file when new file is used
    public static final String kSuperStructureCanivoreName = "superStructure";


    public static final double g = 9.81; 
    public static final double defaultPeriodSecs = 0.02; 
    public static final boolean isSim =  edu.wpi.first.wpilibj.RobotBase.isSimulation(); // Uses diffrent constants if sim or real
    public static final boolean SimFuel = isSim; // Set to true to enable fuel simulation
    public static final boolean DogLogEnabled = true; // Set to true to enable DogLog telemetry
    public static final boolean DogLogNetworkTables = true; // Set to true to enable DogLog over NetworkTables
    public static final boolean LogHardware = false; // Set to true to enable hardware logging in DogLog (Should be on unless low on ram/cpu)

    public static final class ShotCalc {

    public static final double kAccelCompFactor = 0.01; // Factor for Compensating for Robot Acceleration when Shooting on the Move

    // CHECK!
    public static final Pose2d targetpose =  DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ?
    new Pose2d(4.64,4.05,new Rotation2d(0)) : // Blue
    new Pose2d(12,4.05,new Rotation2d(0)); // Red

    public static final Pose2d upperPassPose = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ?
    new Pose2d(2.5,6,new Rotation2d(0)) : // Blue
    new Pose2d(14,6,new Rotation2d(0)); // Red

    public static final Pose2d lowerPassPose = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? 
    new Pose2d(2.5,2,new Rotation2d(0)) : // Blue
    new Pose2d(14,2,new Rotation2d(0)); // Red


    public static InterpolatingDoubleTreeMap ShotVelocity;
    // Distance in meters , velocity in rads per second
    static {
        ShotVelocity = new InterpolatingDoubleTreeMap();
        ShotVelocity.put(0.5, 250.0);
        ShotVelocity.put(1.0, 275.0);
        ShotVelocity.put(1.5, 300.0);
        ShotVelocity.put(2.0, 325.0);
        ShotVelocity.put(2.5, 350.0);
        ShotVelocity.put(3.0, 370.0);
        ShotVelocity.put(3.5, 390.0);
        ShotVelocity.put(4.0, 410.0);
        ShotVelocity.put(4.5, 435.0);
        ShotVelocity.put(5.0, 460.0);
        ShotVelocity.put(5.5, 475.0);
    }
    public static InterpolatingDoubleTreeMap ShotAngle;
    // Distance in meters , Angle of Hood
    static {
        ShotAngle = new InterpolatingDoubleTreeMap();
        ShotAngle.put(0.5, Units.degreesToRadians(8));
        ShotAngle.put(1.0, Units.degreesToRadians(10));
        ShotAngle.put(1.5, Units.degreesToRadians(12));
        ShotAngle.put(2.0, Units.degreesToRadians(15));
        ShotAngle.put(2.5, Units.degreesToRadians(20));
        ShotAngle.put(3.0, Units.degreesToRadians(27.2));
        ShotAngle.put(3.5, Units.degreesToRadians(30.5));
        ShotAngle.put(4.0, Units.degreesToRadians(33));
        ShotAngle.put(4.5, Units.degreesToRadians(36));
        ShotAngle.put(5.0, Units.degreesToRadians(39));
        ShotAngle.put(5.5, Units.degreesToRadians(40));
    }
    // Used for sotm (Shoot on the move)
    public static InterpolatingDoubleTreeMap ShotTime;
    // Distance in meters , time for shot to score in seconds
    static {
        ShotTime = new InterpolatingDoubleTreeMap();
        ShotTime.put(0.5, 1.2);
        ShotTime.put(1.0, 1.2);
        ShotTime.put(1.5, 1.3);
        ShotTime.put(2.0, 1.43);
        ShotTime.put(2.5, 1.44);
        ShotTime.put(3.0, 1.44);
        ShotTime.put(3.5, 1.45);
        ShotTime.put(4.0, 1.5);
        ShotTime.put(4.5, 1.5);
        ShotTime.put(5.0, 1.5);
        ShotTime.put(5.5, 1.5);
    }

    public static InterpolatingDoubleTreeMap PassVelocity;
    static {
        PassVelocity = new InterpolatingDoubleTreeMap();
        PassVelocity.put(4.0, 450.0);
        PassVelocity.put(6.0, 475.0);
        PassVelocity.put(8.0, 475.0);
    }

    public static InterpolatingDoubleTreeMap PassAngle;
    static {
        PassAngle = new InterpolatingDoubleTreeMap();
        PassAngle.put(4.0, Units.degreesToRadians(40));
        PassAngle.put(6.0, Units.degreesToRadians(40));
        PassAngle.put(8.0, Units.degreesToRadians(40));
    }

    public static InterpolatingDoubleTreeMap PassTime;
    static {
        PassTime = new InterpolatingDoubleTreeMap();
        PassTime.put(4.0, 1.0);
        PassTime.put(6.0, 1.2);
        PassTime.put(8.0, 1.4);
    }

    // For simulation only
    public static final double SimShotefficiency = 0.4;
    public static final double Drag = 0.03; // Drag Coefficient for Simulations
    public static final double Friction = 0.6; // Friction Coefficient for Simulations 
    public static final double MagnusLift = 1; // Magnus Lift Coefficient for Simulations (NOT USED RN)

    }
    
    
    
    public static class Drive { 
        //Drive Constants that are not in TunerConstants / Gnenerated

        // PID for Rotation and Translation for Auto and Teleop Snap
        // public static final double PTranslation = 5;
        // public static final double ITranslation = 0.001;
        // public static final double DTranslation = 0.1;

        // public static final double PRotation = 6;
        // public static final double IRotation = 0.01;
        // public static final double DRotation = 0.1;


        public static final double PTranslation = 10;
        public static final double ITranslation = 0;
        public static final double DTranslation = 0;

        public static final double PRotation = 4;
        public static final double IRotation = 0;
        public static final double DRotation = 0;
        
        // 0.0-1.0 of the max speed
        public static final double MaxSpeedPercentage = 0.95; // Default 1.0
        public static final double SlowSpeedPercentage = 0.15; // Default 0.15
        
        // Rotation per second max angular velocity
        public static final double MaxAngularRatePercentage = 0.72; // Default 0.75 
        public static final double SnapMaxAngularRatePercentage = 0.75; // Default 0.5
        public static final double SlowRotPercentage = 0.15; // Default 0.15

        // Deadbands for the drive and rotation
        public static final double DriveDeadband = isSim ? 0.15 : 0.01; // Drive Deadband   Uses higher deadband in sim because my controller sucks
        public static final double RotationDeadband = isSim ? 0.15 : 0.01; // Rotation Deadband
        public static final double SnapDriveDeadband = 0.001; // Snap Rotation Deadband
        public static final double SnapRotationDeadband = 0.00001; // Snap Rotation Deadband

    }

  public static final class Shooter {
    public static final int beamBreakPort = 0;

    public static final CANDeviceID hoodMotorID = new CANDeviceID(32, kSuperStructureCanivoreName);
    // public static final CANDeviceID hoodCoderID = new CANDeviceID(26, kSuperStructureCanivoreName);
    // public static final MechanismRatio hoodMotorRatio =
    //     isSim ? 
    //     new MechanismRatio(
    //         1, (45.0 / 1.0) * (42.0 / 20.0)) : // Sim
    //     new MechanismRatio(
    //         1, (45.0 / 1.0) * (42.0 / 20.0)); // Real
    public static final MechanismRatio hoodMotorRatio =
        isSim ? 
        new MechanismRatio(
            1, (53.1 / 1.0)) : // Sim
        new MechanismRatio(
            1, (53.1 / 1.0)); // Real
    // public static final MechanismRatio hoodSensorRatio =
    //     new MechanismRatio(1, (1.0));
    public static final boolean hoodMotorInvert = false;

    public static final CANDeviceID shooterMotorID = new CANDeviceID(50, kSuperStructureCanivoreName);
    public static final MechanismRatio shooterMotorRatio = new MechanismRatio(12, 18);
    public static final boolean shooterMotorInvert = false;

    public static final CANDeviceID shooter2MotorID = new CANDeviceID(51, kSuperStructureCanivoreName);
    public static final MotorAlignmentValue shooter2MotorInvert = MotorAlignmentValue.Opposed;

    public static final CANDeviceID indexerMotorID = new CANDeviceID(37, kSuperStructureCanivoreName);
    public static final MechanismRatio indexerMotorRatio = new MechanismRatio(1, 1);
    public static final boolean indexerMotorInvert = false;


    //public static final ArmFeedforward armFeedForward = new ArmFeedforward(3.0, 0.3, 0.6);
    public static final Constraints HoodConstraints =
        new Constraints(4, 8); // rad/s and rad/s^2  8, 20.0
    public static final double HoodMaxJerk = 0; // rad/s^3
    public static final int hoodPositionPIDSlot = 0;
    // public static final PIDConfig hoodPositionPIDConfig = new PIDConfig(7, 0.0001, 0.01, 0.1, 4.24, 0.01, 0.13, GravityTypeValue.Arm_Cosine);
    // public static final double hoodExpo_kV = 6;    OLD!!!
    public static final PIDConfig hoodPositionPIDConfig = new PIDConfig(8, 0.00, 0.15, 0.2, 0.2, 0.01, 0.12, GravityTypeValue.Arm_Cosine);
    public static final double hoodExpo_kV = 0;  // 6  
    public static final double hoodExpo_kA = 0;
  //  public static final int armCoralPositionPIDSlot = 1;
  //  public static final PIDConfig armCoralPositionPIDConfig = new PIDConfig(2.0, 0, 0.1, 0, 0.12, 0.007, 0);

    public static final SimpleMotorFeedforward shooterFeedforward =
        new SimpleMotorFeedforward(0.001, 0.02);
    public static final int shooterVelocityPIDSlot = 0;
    public static final PIDConfig shooterVelocityPIDConfig = new PIDConfig(0.2, 0, 0);

    public static final SimpleMotorFeedforward indexerFeedforward =
        new SimpleMotorFeedforward(0.1, 0.02);
    public static final int indexerVelocityPIDSlot = 0;
    public static final PIDConfig indexerVelocityPIDConfig = new PIDConfig(0.1, 0, 0);
    // public static final int shooterPositionPIDSlot = 0;
    // public static final PIDConfig shooterPositionPIDConfig = new PIDConfig(30.0, 0.0, 0.0);

    public static final double hoodMinAngle = Units.degreesToRadians(0.0);
    public static final double hoodMaxAngle = Units.degreesToRadians(81.0);
    public static final double hoodStartingAngle = Units.degreesToRadians(0);
    public static final double hoodCgOffset = Units.degreesToRadians(0);
    public static final double hoodStowAngle = Units.degreesToRadians(1);
    
    public static final double hoodBumpPassAngle = Units.degreesToRadians(40);
    // public static final double hoodBumpPassAngle = Units.degreesToRadians(35); // TEST

    public static final double hoodBumpAngle = Units.degreesToRadians(9); // REAL
    public static final double ShooterBumpVelocity = 250.0; 
    // public static final double hoodBumpAngle = Units.degreesToRadians(27.4); // 3.05
    // public static final double ShooterBumpVelocity = 370.0; 


    public static final double ShooterBumpPassVelocity = 475.0; 
    public static final double WarmupVelocity = 350.0;
    public static final double WarmupVelocityBump = 250;

    // AUTOS
    public static final double ShooterBumpVelocityAngle = 270.0;
        public static final double hoodBumpAngleAngle = Units.degreesToRadians(11);
            public static final double ShooterBumpVelocityAuto = 270.0;
        public static final double hoodBumpAngleAuto = Units.degreesToRadians(7.5);

    

    public static final double AngleTolerance = Units.degreesToRadians(0.1);

    public static final double outtakeVelocity = 1300.0; // rads/s

     public static final double indexerVelocity = 500.0; // rads/s
    public static final double ArmHeight = Units.inchesToMeters(12);

    // For simulation only
    public static final double WheelRadius = Units.inchesToMeters(2);
    public static final double simHoodMOI = 0.171; // kgMetersSquared
    public static final double simHoodCGLength = Units.inchesToMeters(5); // m
    public static final double simShooterMOI = 0.002; // kgMetersSquared
    
  }

  public static final class Hopper {
       public static final CANDeviceID hopperMotorID = new CANDeviceID(39, kSuperStructureCanivoreName);
    public static final MechanismRatio hopperMotorRatio =
        new MechanismRatio(1, 1);
    public static final boolean hopperMotorInvert = false;

    public static final SimpleMotorFeedforward rollerFeedforward =
        new SimpleMotorFeedforward(0.3, 0.12, 0);
    public static final PIDConfig rollerPIDConfig = new PIDConfig(0.1, 0, 0);
    public static final int rollerVelocitySlot = 0;
    
    public static final double hopperVelocity = 400;
    public static final double hopperIntakeVelocity = 60;


  }


  public static final class Intake {


    public static final CANDeviceID ABrollerID = new CANDeviceID(33, kSuperStructureCanivoreName);
    public static final MechanismRatio ABrollerRatio =
        new MechanismRatio(1, (1.0 / 1.0));

    public static final CANDeviceID CrollerID = new CANDeviceID(34, kSuperStructureCanivoreName);
    public static final MechanismRatio CrollerRatio =
        new MechanismRatio(1, (1.0 / 1.0));
    public static final boolean CrollerInvert = false;
    public static final MotorArrangementValue CrollerArrangement = MotorArrangementValue.NEO_JST;


    public static final MechanismRatio rollerMotorRatio =
        new MechanismRatio(1, (1.0 / 1.0));
    public static final boolean rollerMotorInvert = false;
    public static final SimpleMotorFeedforward rollerFeedforward =
        new SimpleMotorFeedforward(0.3, 0.12, 0);
    public static final PIDConfig rollerPIDConfig = new PIDConfig(0.1, 0, 0);
    public static final int rollerVelocitySlot = 0;



    public static final CANDeviceID deployMotorID = new CANDeviceID(40, kSuperStructureCanivoreName);
    public static final MotorAlignmentValue followerInvert = MotorAlignmentValue.Opposed;
    public static final double sprocketPitchDiameter = Units.inchesToMeters(1.714286);
    public static final MechanismRatio deployMotorRatio =
        isSim ? 
        new MechanismRatio(
            1, (25.0 / 1.0), Math.PI * sprocketPitchDiameter) : // Sim
        new MechanismRatio(
           1, (25.0 / 1.0), Math.PI * sprocketPitchDiameter); // Real
    public static final boolean deployMotorInvert = true;
    public static final PIDConfig deployPIDConfig = new PIDConfig(3.0, 0.00, 0.1, 0.00, 0.00, 0.00, 0.00, GravityTypeValue.Elevator_Static);
    public static final int deployPositionSlot = 0;
    public static final double Expo_kV = 0.0;
    public static final double Expo_kA = 0.0; 

    public static final double deployMaxVelocity = 1; // m/s
    public static final double deployMaxAcceleration = 2.0; // m/s^2
    public static final double deployMaxJerk = 5.0; // m/s^3

    public static final double minExtension = Units.inchesToMeters(0.0);
    public static final double maxExtension = Units.inchesToMeters(13);
    public static final double startingPosition = minExtension;
    public static final double intakeStow = Units.inchesToMeters(0.5);

    public static final double attackPosition = Units.inchesToMeters(12.5);
    public static final double defPosition = Units.inchesToMeters(4.0);

    public static final double retractSlowSpeed = Units.inchesToMeters(0.09); 
    public static final double retractHomeSpeed = Units.inchesToMeters(0.09);
    public static final double retractHomeStatorCurrent = 10.0; // Amps
    public static final double retractHomeSupplyCurrent = 10.0; // Amps
    public static final double retractHomeCutoffVelocity = 0.01; // m/s, when to stop retracting in home command

    public static final double intakeCRollerVelocity = 100;
    public static final double antiJamCRollerVelocity = 25;
    public static final double antiJamCRollerTime = 0.1;
    public static final double antiJamCRollerTimeCool = 0.2;
    public static final double antiJamCRollerTimeSpinup = 1;


    public static final double intakeABRollerVelocity = 60;
    public static final double SlowCRollerVelocity = 10;
    public static final double SlowABRollerVelocity = 25;
    public static final double outtakeRollerVelocity = -100;

    public static final double holdRollerVelocity = 0;

    public static final double ManualSpeed = 0.01; // increase to make manual control faster, decrease to make it slower

    // For simulation only
    public static final double simCarriageMass = 3.0; // kg
    public static final double simRollerMOI = 0.01; // kgMetersSquared
  }

  

  public static final class Climber {
    public static final CANDeviceID motorID = new CANDeviceID(40, kSuperStructureCanivoreName);
    // public static final CANDeviceID followerID = new CANDeviceID(21, kSuperStructureCanivoreName);
    public static final double StatorLimit = 120.0;
    public static final double SupplyLimit = 60.0;
    public static final double sprocketPitchDiameter = Units.inchesToMeters(0.75); 
    public static final MechanismRatio motorRatio = isSim ? 
        new MechanismRatio(
            1, (25.0 / 1.0), Math.PI * sprocketPitchDiameter) : // Sim
        new MechanismRatio(
            1, (25.0 / 1.0), Math.PI * sprocketPitchDiameter); // Real
    public static final boolean motorInvert = true;
    // public static final MotorAlignmentValue followerInvert = MotorAlignmentValue.Opposed;
    public static final int motorPositionSlot = 0;
    public static final PIDConfig motorPIDConfig = isSim ? 
        new PIDConfig(5, 0.001, 0.1, 0.04, 41, 0.04, 0.4, GravityTypeValue.Elevator_Static) : 
        new PIDConfig(20, 0.01, 0.04, 0.04, 41, 0.04, 0.4, GravityTypeValue.Elevator_Static);
    public static final double maxVelocity = 0.3; // m/s 
    public static final double maxAcceleration = 8.0; // m/s^2
    public static final double maxJerk = 2.0; // m/s^3 (0 disables jerk limit)
    public static final double Expo_kV = 0.1;    
    public static final double Expo_kA = 0.1; // Use a slower kA of 0.1 V/(rps/s)

    public static final double minHeight = 0.0;
    public static final double maxHeight = Units.inchesToMeters(10.0); 
    public static final double startingHeight = minHeight;
    public static final double stowHeight = Units.inchesToMeters(2.0);
    public static final double PostClimbHeight = Units.inchesToMeters(5); 
    public static final double PreClimbHeight = Units.inchesToMeters(9.0);
    public static final double Tolerance = Units.inchesToMeters(0.1);

    public static final double ManualSpeed = 0.01; // increase to make manual control faster, decrease to make it slower

    // For simulation only
    public static final double simCarriageMass = 60.0; // kg

  }



    public static final class Pose {

    // public static final PathConstraints constraints = new PathConstraints(
    //         3, 2.5,
    //         Units.degreesToRadians(400), Units.degreesToRadians(600));

    public static final double SpeedReductionFactor = 0.15;

    public static final double PTranslationSlow = 3;
    public static final double ITranslationSlow = 10;
    public static final double DTranslationSlow = 0.03;

    public static final double PRotationSlow = 3;
    public static final double IRotationSlow = 10;
    public static final double DRotationSlow = 0.03;

    public static final double Tolerance = 0.01;

    public static final double feildFlip = 16.5;
    public static final double feildFlipy = 8;

    public static final Pose2d Error = new Pose2d(6, 6, Rotation2d.fromDegrees(0));

    public static double ZoneLine = ShotCalc.targetpose.getX();
    // public static final double RedZoneLine = 11.18;
    public static double Halfline = ShotCalc.targetpose.getY();

    public static final double[][] HoodSafetyZones = {
      //minX, maxX, minY, maxY Poses of Safety Zones where Hood should not be up. ie trench.
      {3.5, 5.75, -1.0, 1.75}, 
      {3.5, 5.75, 6, 8.5},
      {10.8, 13, -1.0, 1.75},
      {10.8, 13, 6, 8.5}
    };
    public static final double HoodSafetyVelocityOffset = 0.1; // Multiples by current velocity to see if Robot is going into a Safety Zone with Hood up

  }

  public static class Vision {

    public static final String LimeLightCamerName = "limelight-shooter"; // Front
    // public static final Matrix<N3, N1> LLTagStdDevs = VecBuilder.fill(.8, .8, 9999999);
    public static final Matrix<N3, N1> LLTagStdDevs = VecBuilder.fill(1, 1, 2);
    
    public static final String kCameraName = "FrontCamera"; // Front
    
    // Cam mounted facing forward, 13in forward of center, 7in up from center, up 20 degs.
    public static final Transform3d kRobotToCam =
            new Transform3d(new Translation3d(Units.inchesToMeters(13.311564), 0.0, Units.inchesToMeters(7.332072)), new Rotation3d(0, Math.toRadians(-20), 0));

    public static final AprilTagFieldLayout kTagLayout =
            AprilTagFields.k2026RebuiltAndymark.loadAprilTagLayoutField();

    // Increase these numbers to trust your state estimate less.

    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8); // m, m, rad
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    public static final Matrix<N3, N1> ODOM_STD_DEV = VecBuilder.fill(0.3, 0.3, Units.degreesToRadians(0.01));
    
}
  


  public static final class Viz {
    public static final double xOffset = Units.inchesToMeters(30.0);

    public static final double intakeX = xOffset + Units.inchesToMeters(20);
    public static final double intakeY = Units.inchesToMeters(6);
    public static final double intakeLength = Units.inchesToMeters(10.0);
    public static final double intakeBaseX = xOffset + Units.inchesToMeters(19.0);
    public static final double intakeBaseY = Units.inchesToMeters(5.0);
    public static final double intakeBaseLength = Units.inchesToMeters(12.0);

    public static final double climberBaseX = xOffset + Units.inchesToMeters(2.0);
    public static final double climberBaseY = Units.inchesToMeters(3.0);
    public static final Rotation2d climberAngle = Rotation2d.fromDegrees(90.0);
    public static final double climberBaseLength = Units.inchesToMeters(18.0);
    public static final double climberCarriageLength = Units.inchesToMeters(6.0);

    public static final double HoodPivotX = xOffset + Units.inchesToMeters(12.0);
    public static final double HoodPivotY = Units.inchesToMeters(18.0); 
    public static final double HoodLength = Units.inchesToMeters(4.0);
    public static final double ShooterRadius = Units.inchesToMeters(2.0);

    public static final double angularVelocityScalar = 0.001;
  }

  public static final class Viz3d {
    public static double stage1Height = Units.inchesToMeters(26.0);
    public static final Pose3d intakePivotBase =
        new Pose3d(Units.inchesToMeters(-12.5), 0.0, Units.inchesToMeters(11.0), new Rotation3d());
    public static final Pose3d elevatorBase =
        new Pose3d(
            Units.inchesToMeters(3.5),
            0,
            Units.inchesToMeters(4.0),
            new Rotation3d(0, 0, 0));
    public static final Pose3d shooterBase =
        new Pose3d(
            Units.inchesToMeters(3.5),
            0,
            Units.inchesToMeters(4.0),
            new Rotation3d(0, 0, 0));
    public static final Pose3d intakeBase =
        new Pose3d(
            Units.inchesToMeters(0),
            0,
            Units.inchesToMeters(0),
            new Rotation3d(0, 0, 0));
    public static final Transform3d elevatorCarriageToLauncherArmPivot =
        new Transform3d(0, 0, Units.inchesToMeters(16.0), new Rotation3d());
  }
//   private final DoubleSubscriber ShotVelocitySubscriber1;
//   private final DoubleSubscriber ShotVelocitySubscriber2;
//   private final DoubleSubscriber ShotVelocitySubscriber3;

//   public void updateTunables() {
//     if (DriverStation.isFMSAttached()){
//     // new Alert("FMS is attatched, Tunables updates will be ignored", AlertType.kWarning).set(true);
//     Elastic.Notification notification = new Elastic.Notification(Elastic.NotificationLevel.ERROR, "Tunables are still LIVE!!!", "FMS is attatched, Tunables updates will be ignored");
//     Elastic.sendNotification(notification);
//     Elastic.Notification notification2 = new Elastic.Notification(Elastic.NotificationLevel.WARNING, "I hope this is a practice match", "");
//     Elastic.sendNotification(notification2);
//     }

//     ShotVelocitySubscriber1 = DogLog.tunable("ShotCalc /ShotVelocity",Constants.ShotCalc.ShotVelocity);
//     for (Double key : Constants.ShotCalc.ShotVelocity.) {
//         double value = Constants.ShotCalc.ShotVelocity.getInterpolated(key);
//         ShotVelocitySubscriber.set(value, key);
//     }

//   }

}