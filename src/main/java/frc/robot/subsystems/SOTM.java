package frc.robot.subsystems;

import org.photonvision.PhotonUtils;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.Constants.GoalConstants;
// import frc.robot.Constants.ShooterConstants;
// import frc.robot.Utilities.FieldRelativeAccel;
// import frc.robot.Utilities.FieldRelativeSpeed;
// import frc.robot.Utilities.LinearInterpolationTable;
// import frc.robot.subsystems.ColorSensor;
// import frc.robot.subsystems.Drivetrain;
// import frc.robot.subsystems.Limelight;
// import frc.robot.subsystems.Shooter;
// import frc.robot.subsystems.ShooterHood;
// import frc.robot.subsystems.Turret;
import frc.robot.RobotContainer;
import frc.robot.Constants;
import frc.robot.Constants.Pose;
import frc.robot.Constants.Shooter;
import frc.robot.Constants.ShotCalc;

public class SOTM {
    // private final Shooter m_shooter;
    // private final Turret m_turret;
    // private final Drivetrain m_drive;
    // private final ShooterHood m_hood;
    // private final boolean m_updatePose;
    // private final ColorSensor m_color;
    // private final XboxController m_driver;
    private double m_wrongBallTime;
    private final Timer m_timer = new Timer();
    public static Translation2d movingGoalLocation = new Translation2d();

    // private static LinearInterpolationTable m_timeTable = ShooterConstants.kTimeTable;
    // private static LinearInterpolationTable m_hoodTable = ShooterConstants.kHoodTable;
    // private static LinearInterpolationTable m_rpmTable = ShooterConstants.kRPMTable;

    // public SOTM(Shooter shooter, Turret turret, Drivetrain drive, ShooterHood hood, boolean updatePose,
    //         ColorSensor color, XboxController driver) {
    //     m_shooter = shooter;
    //     m_turret = turret;
    //     m_drive = drive;
    //     m_hood = hood;
    //     m_updatePose = updatePose;
    //     m_color = color;
    //     m_driver = driver;
    //     addRequirements(shooter, turret, hood);
    // }


    // @Override
    // public void initialize() {
    //     m_turret.trackTarget(true);
    //     m_timer.reset();
    //     m_timer.start();
    //     SmartDashboard.putNumber("SetHoodAdjust", 0.0);
    //     SmartDashboard.putNumber("SetShotAdjust", 0);
    //     SmartDashboard.putBoolean("Adjust Shot?", false);
    //     m_wrongBallTime = Double.NEGATIVE_INFINITY;
    // }

    // @Override
    // Retrun maybe [Hood,shotvelocity,angle,angle feeds]
    public static void calcSOTM() {

        // double currentTime = m_timer.get();
        Pose2d robotPose = RobotContainer.drivetrain.getState().Pose;


        Translation2d target = ShotCalc.targetpose.getTranslation();

        // if (currentTime <= m_wrongBallTime + 0.100) {
        //     target = GoalConstants.kWrongBallGoal;
        // }

        Translation2d robotToGoal = target.minus(robotPose.getTranslation());
        double dist = robotToGoal.getDistance(new Translation2d()) ; // * 39.37    in meteres

        DogLog.log("SOTM: Distance to Goal", dist);

        //double fixedShotTime = m_timeTable.getOutput(dist);
        double shotTime = Constants.ShotCalc.Time.get(dist);
        

        DogLog.log("SOTM: Fixed Time", shotTime);

        // Translation2d movingGoalLocation = new Translation2d();

        for(int i=0;i<5;i++){

            double virtualGoalX = target.getX()
                    - shotTime * (DrivetrainExtra.getFieldSpeedsX() + DrivetrainExtra.getFieldAccelX() * ShotCalc.kAccelCompFactor);
            double virtualGoalY = target.getY()
                    - shotTime * (DrivetrainExtra.getFieldSpeedsY() + DrivetrainExtra.getFieldAccelY() * ShotCalc.kAccelCompFactor);

            // SmartDashboard.putNumber("Goal X", virtualGoalX);
            // SmartDashboard.putNumber("Goal Y", virtualGoalY);
            Translation2d VirtualGoal = new Translation2d(virtualGoalX, virtualGoalY);
            DogLog.log("SOTM: Virtual Goal", new Pose2d(VirtualGoal, new Rotation2d()));

            Translation2d testGoalLocation = new Translation2d(virtualGoalX, virtualGoalY);

            Translation2d toTestGoal = testGoalLocation.minus(robotPose.getTranslation());

            double newShotTime = Constants.ShotCalc.Time.get(toTestGoal.getDistance(new Translation2d()) );

            if(Math.abs(newShotTime-shotTime) <= 0.010){
                i=4;
            }
            
            if(i == 4){
                movingGoalLocation = testGoalLocation;
                DogLog.log("SOTM: New Shot Time", newShotTime);
            }
            else{
                shotTime = newShotTime;
            }

        }
        DogLog.log("SOTM: Moving Goal", new Pose2d(movingGoalLocation, new Rotation2d()));

        double newDist = movingGoalLocation.minus(robotPose.getTranslation()).getDistance(new Translation2d()) ;

        DogLog.log("SOTM: New Distance", newDist);

        // m_turret.aimAtGoal(RobotContainer.drivetrain.getState().Pose, movingGoalLocation, false);
        // if (SmartDashboard.getBoolean("Adjust Shot?", false)) {
        //     m_shooter.run(m_rpmTable.getOutput(newDist) + SmartDashboard.getNumber("SetShotAdjust", 0));
        //     m_hood.run(m_hoodTable.getOutput(newDist) + SmartDashboard.getNumber("SetHoodAdjust", 0));
        // } else {
        //     m_shooter.run(m_rpmTable.getOutput(newDist));
        //     m_hood.run(m_hoodTable.getOutput(newDist));

        // }

        // if (currentTime > 0.250 && Limelight.valid() && Limelight.getDistance() >= 85.0) {
        //     double dL = Limelight.getDistance() * 0.0254;
        //     double tR = m_drive.getGyro().getRadians();
        //     double tT = m_turret.getMeasurement() - Math.PI;
        //     double tL = -1.0 * Limelight.tx();

        //     Pose2d pose = calcPoseFromVision(dL, tR, tT, tL, GoalConstants.kGoalLocation);

        //     if (m_updatePose) {
        //         m_drive.setPose(pose);
        //     }

        // }

        // if (m_turret.closeToDeadzone()) {
        //     m_driver.setRumble(RumbleType.kLeftRumble, 1.0);
        //     m_driver.setRumble(RumbleType.kRightRumble, 1.0);
        // } else {
        //     m_driver.setRumble(RumbleType.kLeftRumble, 0.0);
        //     m_driver.setRumble(RumbleType.kRightRumble, 0.0);
        // }

    }

    // @Override
    // public void end(boolean interrupted) {
    //     SmartDashboard.putBoolean("Shooter Running", false);
    //     m_turret.trackTarget(false);
    //     m_turret.stop();
    //     m_shooter.stop();
    //     m_hood.stop();
    //     m_timer.stop();
    //     m_driver.setRumble(RumbleType.kLeftRumble, 0.0);
    //     m_driver.setRumble(RumbleType.kRightRumble, 0.0);
    // }

    // private Pose2d calcPoseFromVision(double dL, double tR, double tT, double tL, Translation2d goal) {
    //     double tG = tR + tT + tL;
    //     double rX = goal.getX() - dL * Math.cos(tG);
    //     double rY = goal.getY() - dL * Math.sin(tG);

    //     return new Pose2d(rX, rY, new Rotation2d(-tR));
    // }

    public static Rotation2d targetangle(){
        /* First put the drivetrain into auto run mode, then run the auto */
        Pose2d pose = RobotContainer.drivetrain.getState().Pose;
        Rotation2d temp = PhotonUtils.getYawToPose(pose, new Pose2d(movingGoalLocation,new Rotation2d()));
        System.out.println(temp);
        return temp;
        
    }

    public static double targetDistance() {
        Pose2d pose = RobotContainer.drivetrain.getState().Pose;
        double distance = pose.getTranslation().getDistance(movingGoalLocation);
        System.out.println("Distance: " + distance);
        return distance;
    }

    public static AngularVelocity targetAngleFeeds() {
        Pose2d pose = RobotContainer.drivetrain.getState().Pose;
        double vx = DrivetrainExtra.getFieldSpeedsX();
        double vy = DrivetrainExtra.getFieldSpeedsY();
        double deltaX = movingGoalLocation.getX() - pose.getX();
        double deltaY = movingGoalLocation.getY() - pose.getY();
        double omega = -(vy * deltaX - vx * deltaY) / (deltaX * deltaX + deltaY * deltaY);
        System.out.println("omega: " + omega);
        return AngularVelocity.ofBaseUnits(omega, edu.wpi.first.units.Units.RadiansPerSecond);
    }

}