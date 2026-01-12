package frc.robot;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.quixlib.devices.CANDeviceID;
import frc.quixlib.motorcontrol.MechanismRatio;
import frc.quixlib.motorcontrol.PIDConfig;

public class Constants {
    // CANID's:
    //
    // Drivetrain 1-19
    // Elevator / Arm 20-29 
    // Intake 30-39
    


    // "rio" for rio bus
    public static final String kDriveTrainCanivoreName = "driveTrain"; // need to update in tuner generated file when new file is used
    public static final String kSuperStructureCanivoreName = "superStructure";


    public static final double g = 9.81; 
    public static final double defaultPeriodSecs = 0.02; 
    public static final boolean isSim =  false;// edu.wpi.first.wpilibj.RobotBase.isSimulation(); // Uses diffrent constants if sim or real
    public static final boolean SimFuel = true; // Set to true to enable fuel simulation
    public static final boolean DogLogEnabled = true; // Set to true to enable DogLog telemetry
    public static final boolean DogLogNetworkTables = true; // Set to true to enable DogLog over NetworkTables

    public static final class ShotCalc {

    // CHECK!
    public static final Pose2d targetpose =  DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ?
    new Pose2d(4.64,4.05,new Rotation2d(0)) : // Blue
    new Pose2d(12,4.05,new Rotation2d(0)); // Red

    public static final InterpolatingDoubleTreeMap Velocity;
    static {
        Velocity = new InterpolatingDoubleTreeMap();
        Velocity.put(0.0, 200.0);
        Velocity.put(2.0, 250.0);
        Velocity.put(3.0, 300.0);
        Velocity.put(4.0, 350.0);
    }
    public static final InterpolatingDoubleTreeMap Angle;
    static {
        Angle = new InterpolatingDoubleTreeMap();
        Angle.put(0.0, Units.degreesToRadians(80));
        Angle.put(1.0, Units.degreesToRadians(75));
        Angle.put(2.0, Units.degreesToRadians(71));
        Angle.put(3.0, Units.degreesToRadians(68));
        Angle.put(4.0, Units.degreesToRadians(65));
        Angle.put(5.0, Units.degreesToRadians(63));
    }

    public static final double SimShotefficiency = 0.85; 
  }
    
    
    
    
    
    public static class Drive { 
        //Drive Constants that are not in TunerConstants / Gnenerated

        // PID for Rotation and Translation for Auto and Teleop Snap
        public static final double PTranslation = 5;
        public static final double ITranslation = 0.001;
        public static final double DTranslation = 0.1;

        public static final double PRotation = 3;
        public static final double IRotation = 0.001;
        public static final double DRotation = 0.03;
        
        // 0.0-1.0 of the max speed
        public static final double MaxSpeedPercentage = 0.95; // Default 1.0
        public static final double SlowSpeedPercentage = 0.10; // Default 0.15
        
        // Rotation per second max angular velocity
        public static final double MaxAngularRatePercentage = 0.82; // Default 0.75 
        public static final double SlowRotPercentage = 0.15; // Default 0.15

        // Deadbands for the drive and rotation
        public static final double DriveDeadband = isSim ? 0.15 : 0.02; // Drive Deadband
        public static final double RotationDeadband = isSim ? 0.15 : 0.02; // Rotation Deadband
        public static final double SnapDriveDeadband = 0.001; // Snap Rotation Deadband
        public static final double SnapRotationDeadband = 0.001; // Snap Rotation Deadband

    }


  public static final class Climber {
    public static final CANDeviceID motorID = new CANDeviceID(20, kSuperStructureCanivoreName);
    public static final CANDeviceID followerID = new CANDeviceID(21, kSuperStructureCanivoreName);
    public static final double StatorLimit = 120.0;
    public static final double SupplyLimit = 60.0;
    public static final double sprocketPitchDiameter = Units.inchesToMeters(2.273); // 16T #25
    public static final MechanismRatio motorRatio =
        isSim ? 
        new MechanismRatio(
            1, (28.0 / 10.0) * (1.0 / 4.0) * (42.0 / 18.0), Math.PI * sprocketPitchDiameter) : // Sim
        new MechanismRatio(
            1, (9.0 / 1.0), Math.PI * sprocketPitchDiameter); // Real
    public static final boolean motorInvert = true;
    public static final MotorAlignmentValue followerInvert = MotorAlignmentValue.Opposed;
    public static final int motorPositionSlot = 0;
    public static final PIDConfig motorPIDConfig = isSim ? 
        new PIDConfig(5, 0.001, 0.1, 0.04, 0.02, 0.008, 0.13, GravityTypeValue.Elevator_Static) : 
        new PIDConfig(20, 0.01, 0.04, 0.04, 0.13, 0.008, 0.13, GravityTypeValue.Elevator_Static);
    public static final double maxVelocity = 1.8; // m/s // 1.2
    public static final double maxAcceleration = 27.0; // m/s^2
    public static final double maxJerk = 2.0; // m/s^3 (0 disables jerk limit)
    public static final double Expo_kV = 0.1;    
    public static final double Expo_kA = 0.1; // Use a slower kA of 0.1 V/(rps/s)

    public static final double minHeight = 0.0; // m
    public static final double maxHeight = Units.inchesToMeters(60.0); // m
    public static final double stowHeight = Units.inchesToMeters(0.5); // m
    public static final double wristStowHeight = Units.inchesToMeters(4); // m
    public static final double armStowHeight = Units.inchesToMeters(8); // m
    public static final double stowTolerance = Units.inchesToMeters(0.1); // m
    public static final double intakeHeight = Units.inchesToMeters(11.75); // m // 

    public static final double L1 = Units.inchesToMeters(1); // m
    public static final double L2 = Units.inchesToMeters(13.75); // m
    public static final double L3 = Units.inchesToMeters(29.5); // m
    public static final double L4 = Units.inchesToMeters(56); // m 56.3 max, 55.8 was a lil too low

    public static final double L2Algae = Units.inchesToMeters(15); // m
    public static final double L3Algae = Units.inchesToMeters(31.5); // m

    public static final double PreStow = Units.inchesToMeters(10); // m
    public static final double SlowmodeHeight = Units.inchesToMeters(25); // m

    // For simulation only
    public static final double simCarriageMass = 7.0; // kg

  }

  

  public static final class Shooter {
    public static final int beamBreakPort = 0;

    public static final CANDeviceID hoodMotorID = new CANDeviceID(25, kSuperStructureCanivoreName);
    // public static final CANDeviceID armCoderID = new CANDeviceID(26, kSuperStructureCanivoreName);
    public static final MechanismRatio hoodMotorRatio =
        isSim ? 
        new MechanismRatio(
            1, (90.0 / 1.0) * (80.0 / 38.0)) : // Sim
        new MechanismRatio(
            1, (60.0 / 1.0) * (80.0 / 38.0)); // Real
    public static final MechanismRatio hoodSensorRatio =
        new MechanismRatio(1, (1.0));
    public static final boolean hoodMotorInvert = true;

    public static final CANDeviceID rollerMotorID = new CANDeviceID(28, kSuperStructureCanivoreName);
    public static final MechanismRatio rollerMotorRatio = new MechanismRatio(12, 18);
    public static final boolean rollerMotorInvert = false;


    //public static final ArmFeedforward armFeedForward = new ArmFeedforward(3.0, 0.3, 0.6);
    public static final Constraints ArmConstraints =
        new Constraints(3.5, 10.0); // rad/s and rad/s^2  8, 20.0
    public static final double ArmMaxJerk = 1.0; // rad/s^3
    public static final int armPositionPIDSlot = 0;
    public static final PIDConfig armPositionPIDConfig = new PIDConfig(8, 0.0001, 0.03, 0, 0.25, 0.0008, 0.09, GravityTypeValue.Arm_Cosine);
    public static final double armExpo_kV = 0.25;    
    public static final double armExpo_kA = 0.01; // Use a slower kA of 0.1 V/(rps/s)
  //  public static final int armCoralPositionPIDSlot = 1;
  //  public static final PIDConfig armCoralPositionPIDConfig = new PIDConfig(2.0, 0, 0.1, 0, 0.12, 0.007, 0);

    public static final SimpleMotorFeedforward rollerFeedforward =
        new SimpleMotorFeedforward(0.1, 0.028);
    public static final int rollerVelocityPIDSlot = 1;
    public static final PIDConfig rollerVelocityPIDConfig = new PIDConfig(0.1, 0.0, 0.0);
    public static final int rollerPositionPIDSlot = 0;
    public static final PIDConfig rollerPositionPIDConfig = new PIDConfig(30.0, 0.0, 0.0);

    public static final double armMinAngle = Units.degreesToRadians(-1.0); // rads 
    public static final double armMaxAngle = Units.degreesToRadians(150.0); // rads 
    public static final double armStartingAngle = Units.degreesToRadians(90);
    public static final double armCgOffset = Units.degreesToRadians(0);

    public static final double AngleTolerance = Units.degreesToRadians(1);

    public static final double intakeVelocity = -150.0; // rads/s
    public static final double outtakeVelocity = 1300.0; // rads/s

    public static final double rollerStallVelocity = 40; // rads/s
    public static final double rollerStallCurrent = 30; // Amps


    public static final double armIntakeAngle = Units.degreesToRadians(130);

    
    public static final Transform2d robotToArm =
        new Transform2d(Units.inchesToMeters(12.0), 0.0, new Rotation2d());
    public static final double ArmHeight = Units.inchesToMeters(12);

    // For simulation only
    public static final double WheelRadius = Units.inchesToMeters(1.5);
    public static final double simArmMOI = 0.379; // kgMetersSquared
    public static final double simArmCGLength = Units.inchesToMeters(8.5); // m
    public static final double simRollerMOI = 0.003; // kgMetersSquared
    
  }


  public static final class Pose {

    // public static final PathConstraints constraints = new PathConstraints(
    //         3, 2.5,
    //         Units.degreesToRadians(400), Units.degreesToRadians(600));

     public static final double XvelocityFactor = 0.1;
     public static final double YvelocityFactor = 0.1;

    public static final double SpeedReductionFactor = 0.15;

    public static final double PTranslationSlow = 3;
    public static final double ITranslationSlow = 10;
    public static final double DTranslationSlow = 0.03;

    public static final double PRotationSlow = 3;
    public static final double IRotationSlow = 10;
    public static final double DRotationSlow = 0.03;

    public static final double Tolerance = 0.01;

    public static final double feildFlip = 17.5;
    public static final double feildFlipy = 8;

    public static final Pose2d Error = new Pose2d(6, 6, Rotation2d.fromDegrees(0));

  }

  public static class Vision {

    public static final String LimeLightCamerName = "FrontLimelight"; // Front
    public static final Matrix<N3, N1> LLTagStdDevs = VecBuilder.fill(.5, .5, 9999999);
    
    public static final String kCameraName = "FrontCamera"; // Front
    
    // Cam mounted facing forward, 13in forward of center, 7in up from center, up 20 degs.
    public static final Transform3d kRobotToCam =
            new Transform3d(new Translation3d(Units.inchesToMeters(13.311564), 0.0, Units.inchesToMeters(7.332072)), new Rotation3d(0, Math.toRadians(-20), 0));

    public static final AprilTagFieldLayout kTagLayout =
            AprilTagFields.kDefaultField.loadAprilTagLayoutField();

    // Increase these numbers to trust your state estimate less.
    // 461 values
    // public static final Matrix<N3, N1> ODOM_STD_DEV = VecBuilder.fill(0.03, 0.03, Units.degreesToRadians(0.01));
    // public static final Function<Double, Matrix<N3, N1>> VISION_STD_DEV_MULTITAG_FUNCTION =
    //         dist -> dist < 3.0
    //                 ? VecBuilder.fill(Math.min(0.03, 0.03 * dist), Math.min(0.03, 0.03 * dist), DriverStation.isEnabled() ? Units.degreesToRadians(5.0) : Units.degreesToRadians(0.05))
    //                 : VecBuilder.fill(0.05 * dist, 0.05 * dist, Units.degreesToRadians(180.0) * dist);
    // public static final Function<Double, Matrix<N3, N1>> VISION_STD_DEV_FUNCTION =
    //         dist -> dist < 3.0
    //                 ? VecBuilder.fill(0.075 * dist, 0.075 * dist, Units.degreesToRadians(180.0) * dist)
    //                 : VecBuilder.fill(0.1 * dist, 0.1 * dist, Units.degreesToRadians(180.0) * dist);

    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8); // m, m, rad
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    public static final Matrix<N3, N1> ODOM_STD_DEV = VecBuilder.fill(0.03, 0.03, Units.degreesToRadians(0.01));
    
}
  


  public static final class Viz {
    public static final double xOffset = Units.inchesToMeters(30.0);

    public static final double intakePivotX = xOffset + Units.inchesToMeters(27.25);
    public static final double intakePivotY = Units.inchesToMeters(11.25);
    public static final double intakeArmLength = Units.inchesToMeters(14.0);

    public static final double elevatorBaseX = xOffset + Units.inchesToMeters(18.0);
    public static final double elevatorBaseY = Units.inchesToMeters(3.0);
    public static final Rotation2d elevatorAngle = Rotation2d.fromDegrees(90.0);
    public static final double elevatorBaseLength = Units.inchesToMeters(35.0);
    public static final double elevatorCarriageLength = Units.inchesToMeters(6.0);

    public static final double ArmArmPivotX = Units.inchesToMeters(4.0);
    public static final double ArmArmLength = Units.inchesToMeters(12.0);
    public static final double ArmWristLength = Units.inchesToMeters(6.0);
    public static final double ArmRollerX = Units.inchesToMeters(8.0);
    public static final double ArmRollerY = Units.inchesToMeters(0);

    public static final double angularVelocityScalar = 0.01;
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
    public static final Transform3d elevatorCarriageToLauncherArmPivot =
        new Transform3d(0, 0, Units.inchesToMeters(16.0), new Rotation3d());
  }


  public static final class Intake {
    public static final int beamBreakPort = 1;

    public static final CANDeviceID rollerMotorID = new CANDeviceID(33, kSuperStructureCanivoreName);

    public static final MechanismRatio rollerMotorRatio =
        new MechanismRatio(1, (1.0 / 3.0));
    public static final boolean rollerMotorInvert = false;
    public static final SimpleMotorFeedforward rollerFeedforward =
        new SimpleMotorFeedforward(0.3, 0.12, 0);
    public static final PIDConfig rollerPIDConfig = new PIDConfig(0.1, 0, 0);
    public static final int rollerVelocitySlot = 0;

    public static final CANDeviceID deployMotorID = new CANDeviceID(31, kSuperStructureCanivoreName);
    public static final CANDeviceID deployFollowerID = new CANDeviceID(32, kSuperStructureCanivoreName);

    public static final MotorAlignmentValue followerInvert = MotorAlignmentValue.Opposed;
    public static final MechanismRatio deployMotorRatio =
        isSim ? 
        new MechanismRatio(
            1, (42.0 / 10.0) * (22.0 / 22.0) * (42.0 / 16.0) * (36.0 / 16.0)) : // Sim
        new MechanismRatio(
            1, (27.0 / 1.0) * (36.0 / 16.0)); // Real
    public static final boolean deployMotorInvert = false;
    public static final PIDConfig deployPIDConfig = new PIDConfig(2.0, 0, 0.3, 0, 1.5, 0.000, 0.08, GravityTypeValue.Arm_Cosine);
    public static final int deployPositionSlot = 0;
    public static final double deployMaxVelocity = 0.2; // rad/s
    public static final double deployMaxAcceleration = 140.0; // rad/s^2
    public static final double deployMaxJerk = 800.0; // rad/s^3

    public static final double minAngle = Units.degreesToRadians(-20.0); // rads
    public static final double maxAngle = Units.degreesToRadians(110.0); // rads
    public static final double startingAngle = maxAngle;
    public static final double intakeDeployAngle = Math.toRadians(40); // rad
    public static final double intakeScoreAngle = Math.toRadians(85); // rad
    public static final double intakeStowAngle = Math.toRadians(105); // rad
    public static final double intakeClimbAngle = Math.toRadians(100); // rad
    public static final double intakeRollerVelocity = 100; // rad/s
    public static final double outtakeRollerVelocity = -100; // rad/s
    public static final double holdRollerVelocity = 10; // rad/s

    // For simulation only
    public static final double simArmMOI = 0.2; // kgMetersSquared
    public static final double simArmCGLength = Units.inchesToMeters(7.0); // m
    public static final double simRollerMOI = 0.01; // kgMetersSquared
  }

  

}