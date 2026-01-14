package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.CoralEleUp;
import frc.robot.commands.Intake;
import frc.robot.commands.Moveup;
import frc.robot.commands.Score;
import frc.robot.commands.StowAll;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.DrivetrainExtra;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ClimberSubsystem;

public class RobotContainer {
    public static double MaxSpeed = Constants.Drive.MaxSpeedPercentage*(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond)); // kSpeedAt12Volts desired top speed
    public static double MaxAngularRate = RotationsPerSecond.of(Constants.Drive.MaxAngularRatePercentage).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    public static final CommandXboxController driver = new CommandXboxController(0);
    // public static final CommandXboxController operator = new CommandXboxController(1);

    // public static final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public static final CommandSwerveDrivetrain drivetrain = new CommandSwerveDrivetrain(TunerConstants.DrivetrainConstants,250, Constants.Vision.ODOM_STD_DEV, Constants.Vision.kSingleTagStdDevs, TunerConstants.FrontLeft, TunerConstants.FrontRight, TunerConstants.BackLeft, TunerConstants.BackRight);

    /* Setting up bindings for necessary control of the swerve drive platform */
    public static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * Constants.Drive.DriveDeadband).withRotationalDeadband(MaxAngularRate * Constants.Drive.RotationDeadband)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    public static final SwerveRequest.FieldCentric driveAuto = new SwerveRequest.FieldCentric()
            .withDeadband(Constants.Drive.SnapDriveDeadband).withRotationalDeadband(Constants.Drive.SnapRotationDeadband)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors


    public final SwerveRequest.FieldCentricFacingAngle angle = new SwerveRequest.FieldCentricFacingAngle()
        .withDeadband(MaxSpeed * Constants.Drive.DriveDeadband).withRotationalDeadband(Constants.Drive.SnapRotationDeadband) // Add a deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors 
         //  .withSteerRequestType(SteerRequestType.MotionMagicExpo); // Use motion magic control for steer motors

    private PowerDistribution powerDistribution = new PowerDistribution();

    /* Path follower */
    private final SendableChooser<Command> autoChooser;


    Map<String, Command> robotCommands  = new HashMap<String, Command>();





    public static final ClimberSubsystem climber = new ClimberSubsystem();
    public static final IntakeSubsystem intake = new IntakeSubsystem();
    public static final ShooterSubsystem shooter = new ShooterSubsystem();






    public RobotContainer() {
        // // Starts recording to data log
        // DataLogManager.start();
        // // Record both DS control and joystick data
        // DriverStation.startDataLog(DataLogManager.getLog());

        DogLog.setOptions(new DogLogOptions().withNtPublish(Constants.DogLogNetworkTables));
        DogLog.setOptions(new DogLogOptions().withNtTunables(Constants.DogLogNetworkTables));
        DogLog.setOptions(new DogLogOptions().withCaptureDs(Constants.DogLogNetworkTables));
        DogLog.setOptions(new DogLogOptions().withCaptureConsole(true));
        DogLog.setOptions(new DogLogOptions().withLogExtras(true));
        // DogLog.setEnabled(Constants.DogLogEnabled);

        //robotCommands.put("IntakePiece", new IntakeAlgae(intake,1).withTimeout(2.5));
        robotCommands.put("StowArm", DrivetrainExtra.LogTime("StowAll", new StowAll(climber, shooter)));
        robotCommands.put("L4", Commands.runOnce(() -> RobotContainer.climber.setHeightLocation(4)));
    
        NamedCommands.registerCommands(robotCommands);



        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto", autoChooser);
        angle.HeadingController.setPID( Constants.Drive.PRotation,  Constants.Drive.IRotation , Constants.Drive.DRotation);
        configureBindings();

    


        SmartDashboard.putData(
        "Gyro",
        builder -> {
          builder.setSmartDashboardType("Gyro");
          builder.addDoubleProperty("Value", () -> drivetrain.getPigeon2().getYaw().getValueAsDouble(), null);
        });
         // SmartDashboard.putNumber("Time",Timer.getMatchTime());
        SmartDashboard.putNumber("Time",DriverStation.getMatchTime());
        SmartDashboard.putNumber("Voltage",RobotController.getBatteryVoltage());
        SmartDashboard.putNumber("CAN",RobotController.getCANStatus().percentBusUtilization * 100.0);
        SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
        DogLog.setPdh(powerDistribution);
          

        //PathfindingCommand.warmupCommand().ignoringDisable(true).schedule();;
        
    }

    

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driver.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driver.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driver.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        //driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // driver.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))
        // ));

        // driver.leftBumper().whileTrue(new ParallelCommandGroup(new CoralMoveScore(climber, shooter), DrivetrainExtra.pathfindingCommand(true,true)));
        // driver.rightBumper().whileTrue(new ParallelCommandGroup(new CoralMoveScore(climber, shooter), DrivetrainExtra.pathfindingCommand(false,true)));
        // driver.leftBumper().onFalse(new CoralMoveStow(climber, shooter));
        // driver.rightBumper().onFalse(new CoralMoveStow(climber, shooter));

        //driver.y().whileTrue(new ClimbSet(climber));
        //driver.x().whileTrue(pathfindingtofollowCommand());
        driver.leftTrigger().or(() -> (RobotContainer.climber.getHeight() >= Constants.Climber.SlowmodeHeight) && !DriverStation.isAutonomous()).whileTrue(drivetrain.applyRequest(() ->
        drive.withVelocityX(-driver.getLeftY() * MaxSpeed * Constants.Drive.SlowSpeedPercentage) // Drive forward with negative Y (forward)
            .withVelocityY(-driver.getLeftX() * MaxSpeed * Constants.Drive.SlowSpeedPercentage) // Drive left with negative X (left)
            .withRotationalRate(-driver.getRightX() * MaxAngularRate * Constants.Drive.SlowRotPercentage) // Drive counterclockwise with negative X (left)
        ));
        // driver.rightTrigger().whileTrue(new IntakeCoral(climber, shooter));
       // driver.leftTrigger().whileTrue(new IntakeAlgae(intake, 0));
        driver.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        driver.back().whileTrue(new StowAll(climber, shooter));

        driver.rightBumper().whileTrue(new ParallelCommandGroup(new Score(shooter),drivetrain.applyRequest(() ->
        angle.withVelocityX(-driver.getLeftY() * MaxSpeed)
            .withVelocityY(-driver.getLeftX() * MaxSpeed)
            .withTargetDirection(DrivetrainExtra.targetangle(Constants.ShotCalc.targetpose ))
            .withTargetRateFeedforward(DrivetrainExtra.targetAngleFeeds(Constants.ShotCalc.targetpose))
            )));

        driver.rightTrigger().whileTrue(new Intake(intake));
        
        
        

        // driver.y().whileTrue(drivetrain.applyRequest(() ->
        //     angle.withVelocityX(-driver.getLeftY() * MaxSpeed)
        //     .withVelocityY(-driver.getLeftX() * MaxSpeed)
        //     .withTargetDirection(new Rotation2d(Math.toRadians(0))))
        // );
        


       // driver.a().whileTrue(new AlgeaWack(elevator, shooter));
       
    //    driver.a().whileTrue(new MoveupArm(1, climber, shooter)); 
    //    driver.b().whileTrue(DrivetrainExtra.LogTime("test", new MoveupArm(2, climber, shooter))); 
       driver.y().whileTrue(new Moveup(climber));

    }


// --------------------------------------------------------- Commands --------------------------------------------------------- 


    public Command getAutonomousCommand() {
       
        return autoChooser.getSelected();
    }

   
}
//Abhi was here
