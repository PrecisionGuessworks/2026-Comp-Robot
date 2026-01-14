package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.quixlib.viz.Link2d;
import frc.quixlib.viz.Viz2d;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import dev.doglog.DogLog;

public class Visualization {



private static  final Viz2d robotViz =
      new Viz2d("Robot Viz", Units.inchesToMeters(80.0), Units.inchesToMeters(120.0), 1.0);

private static  final Link2d chassisViz =
      robotViz.addLink(
          new Link2d(
              robotViz,
              "Chassis",
              Units.inchesToMeters(29.0),
              30.0,
              new Color("#FAB604"),
              new Transform2d(Constants.Viz.xOffset, Units.inchesToMeters(3.0), new Rotation2d())));

  // Climber viz
  private  static final Link2d climberFrameViz =
      robotViz.addLink(
          new Link2d(
              robotViz,
              "Climber Base",
              Constants.Viz.climberBaseLength,
              4.0,
              Color.kGreen,
              new Transform2d(
                  Constants.Viz.climberBaseX,
                  Constants.Viz.climberBaseY,
                  Constants.Viz.climberAngle)));
  private static  final Link2d climberCarriageViz =
      climberFrameViz.addLink(
          new Link2d(
              robotViz,
              "Climber Carriage",
              Constants.Viz.climberCarriageLength,
              6.0,
              Color.kLightGreen));

// Intake viz
private static final Link2d intakeViz =
robotViz.addLink(
    new Link2d(robotViz, "Intake", Constants.Viz.intakeLength, 10.0, Color.kBlue));
private static final Link2d intakeBaseViz =
robotViz.addLink(
    new Link2d(robotViz,"Intake Base",Constants.Viz.intakeBaseLength,6.0,Color.kNavy,new Transform2d(Constants.Viz.intakeBaseX,Constants.Viz.intakeBaseY,new Rotation2d())));
private static final Link2d intakeRollerViz =
intakeViz.addLink(
    new Link2d(robotViz, "Intake Roller", Units.inchesToMeters(1.0), 10.0, Color.kLightBlue));


private static final Link2d HoodViz =
robotViz.addLink(
        new Link2d(robotViz, "Hood", Constants.Viz.HoodLength, 10, Color.kRed));
private static final Link2d ShooterViz =
HoodViz.addLink(
        new Link2d(robotViz, "Shooter", Units.inchesToMeters(2.0), 10, Color.kCoral));





public static void Update2DVisualization() {

        climberCarriageViz.setRelativeTransform(
            new Transform2d(RobotContainer.climber.getHeight(), 0.0, new Rotation2d()));

        HoodViz.setRelativeTransform(
        new Transform2d(
            Constants.Viz.HoodPivotX,
            Constants.Viz.HoodPivotY,
            Rotation2d.fromRadians(-Units.degreesToRadians(RobotContainer.shooter.getHoodAngle()) + Units.degreesToRadians( 180))));

    ShooterViz.setRelativeTransform(
        new Transform2d(
            Constants.Viz.HoodLength,
            0.0,
            Rotation2d.fromRadians(
                ShooterViz.getRelativeTransform().getRotation().getRadians()
                    + RobotContainer.shooter.getShooterVelocity() * Constants.Viz.angularVelocityScalar)));


        
    intakeViz.setRelativeTransform(
        new Transform2d(
            Constants.Viz.intakeX+RobotContainer.intake.getPosition(),
            Constants.Viz.intakeY,
            new Rotation2d()));
    intakeRollerViz.setRelativeTransform(
        new Transform2d(
            Constants.Viz.intakeLength,
            0.0,
            Rotation2d.fromRadians(
                intakeRollerViz.getRelativeTransform().getRotation().getRadians()
                    + RobotContainer.intake.getRollerVelocity()
                        * Constants.Viz.angularVelocityScalar)));

    }







    public static void Update3DVisualization() {
    // 3d viz
    final double stage1Height = Constants.Viz3d.stage1Height;
    final double CarrageHeight = RobotContainer.climber.getHeight();
    final Pose3d stageOne =
      Constants.Viz3d.elevatorBase.transformBy(
          new Transform3d(0, 0, CarrageHeight >= stage1Height ? CarrageHeight-stage1Height : 0, new Rotation3d()));
    final Pose3d elevatorCarriage =
        Constants.Viz3d.elevatorBase.transformBy(
            new Transform3d(0, 0, CarrageHeight+ Units.inchesToMeters(0.5), new Rotation3d()));
    final Pose3d armViz = elevatorCarriage.transformBy(
        new Transform3d(0, 0, Units.inchesToMeters(7.7), new Rotation3d(0,Units.degreesToRadians( -RobotContainer.shooter.getHoodAngle()+90),0)));

    final Pose3d hoodViz = Constants.Viz3d.shooterBase.transformBy(
        new Transform3d(0, 0, Units.inchesToMeters(7.7), new Rotation3d(0,Units.degreesToRadians( -RobotContainer.shooter.getHoodAngle()+90),0)));
    final Pose3d intakeViz = Constants.Viz3d.intakeBase.transformBy(
        new Transform3d(0, 0, Units.inchesToMeters(2.0), new Rotation3d(0,0,0))); 
    
    DogLog.log("3DViz: Zeropublisher", new Pose3d());
    DogLog.log("3DViz: 1DriveBase", RobotContainer.drivetrain.getState().Pose);     
    DogLog.log("3DViz: 3ElevatorCarriage", elevatorCarriage);
    DogLog.log("3DViz: 2Stage1", stageOne);
    DogLog.log("3DViz: 4ArmViz", armViz);
    DogLog.log("3DViz: 5HoodViz", hoodViz);
    DogLog.log("3DViz: 6IntakeViz", intakeViz);
    DogLog.log("3DViz: FuelViz:1", fuelViz[1]);
    DogLog.log("3DViz: FuelViz:5", fuelViz[5]);
    DogLog.log("3DViz: FuelViz:10", fuelViz[10]);
    DogLog.log("3DViz: FuelViz:15", fuelViz[15]);
    DogLog.log("3DViz: FuelViz:19", fuelViz[19]);
    }




// ------------------------------------------------------------------------------ Fuel Visualization ------------------------------------------------------------------------------




  public static  StructArrayPublisher<Pose3d> fuelpublisher = NetworkTableInstance.getDefault()
        .getStructArrayTopic("fuelViz", Pose3d.struct).publish();
  public static Pose3d[] fuelViz = {new Pose3d(),new Pose3d(),new Pose3d(),new Pose3d(),new Pose3d(),new Pose3d(),new Pose3d(),new Pose3d(),new Pose3d(),new Pose3d(),new Pose3d(),new Pose3d(),new Pose3d(),new Pose3d(),new Pose3d(),new Pose3d(),new Pose3d(),new Pose3d(),new Pose3d(),new Pose3d()};
  public static double fuelVelocity[][] = {{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0}}; 
  public static int fuelIndex = 0;

  public static void updateSingleFuelViz(Pose3d fuelStartPose3d, double[] fuelStartVelocity){
    fuelViz[fuelIndex] = fuelStartPose3d;
    fuelVelocity[fuelIndex] = fuelStartVelocity;
    fuelIndex++;
    if (fuelIndex > fuelViz.length - 1) fuelIndex = 0;
  }

  public static void LaunchFuelViz(double velocity,double angle){
    if(Constants.SimFuel){
        double ShotVelocity = velocity * Constants.Shooter.WheelRadius * Constants.ShotCalc.SimShotefficiency;  
            updateSingleFuelViz(new Pose3d(RobotContainer.drivetrain.getState().Pose.getX(),RobotContainer.drivetrain.getState().Pose.getY(),0.4, new Rotation3d(0,-angle,RobotContainer.drivetrain.getState().Pose.getRotation().getRadians())), 
            new double[] {DrivetrainExtra.getFieldSpeedsX() + ShotVelocity * Math.cos(angle)*Math.cos(RobotContainer.drivetrain.getState().Pose.getRotation().getRadians()),
              DrivetrainExtra.getFieldSpeedsY() + ShotVelocity * Math.cos(angle)*Math.sin(RobotContainer.drivetrain.getState().Pose.getRotation().getRadians()), 
              ShotVelocity * Math.sin(angle) });
    }
  }

  public static void updateFuelViz(){
    if(Constants.SimFuel){
    for (int i = 0; i < fuelViz.length; i++) {
      if (fuelViz[i].getZ() > 0.1 ){
        fuelVelocity[i][2] -= (9.81 * Constants.defaultPeriodSecs + (fuelVelocity[i][2] > 0 ? 1 : -1) * Constants.ShotCalc.Drag * (fuelVelocity[i][2]) * (fuelVelocity[i][2]) * Constants.defaultPeriodSecs);
        fuelVelocity[i][0] -= (fuelVelocity[i][0] > 0 ? 1 : -1) * Constants.ShotCalc.Drag*(fuelVelocity[i][0]) * (fuelVelocity[i][0])* Constants.defaultPeriodSecs; 
        fuelVelocity[i][1] -= (fuelVelocity[i][1] > 0 ? 1 : -1) * Constants.ShotCalc.Drag*(fuelVelocity[i][1]) * (fuelVelocity[i][1])* Constants.defaultPeriodSecs;  
      } else {
        fuelViz[i] = new Pose3d(fuelViz[i].getX(),fuelViz[i].getY(),0.1, new Rotation3d(0,0,0));
        fuelVelocity[i][0] -= (fuelVelocity[i][0] > 0 ? 1 : -1) * Constants.ShotCalc.Friction*(fuelVelocity[i][0]) * (fuelVelocity[i][0])* Constants.defaultPeriodSecs;
        fuelVelocity[i][1] -= (fuelVelocity[i][1] > 0 ? 1 : -1) * Constants.ShotCalc.Friction*(fuelVelocity[i][1]) * (fuelVelocity[i][1])* Constants.defaultPeriodSecs;
        if (Math.abs(fuelVelocity[i][0])+Math.abs(fuelVelocity[i][1])  < 0.5){
        fuelVelocity[i][0] = 0;
        fuelVelocity[i][1] = 0;
        }
        fuelVelocity[i][2] = 0;
      }
      fuelViz[i] = new Pose3d(
        fuelViz[i].getX() + fuelVelocity[i][0] * Constants.defaultPeriodSecs,
        fuelViz[i].getY() + fuelVelocity[i][1] * Constants.defaultPeriodSecs,
        fuelViz[i].getZ() + fuelVelocity[i][2] * Constants.defaultPeriodSecs,
        new Rotation3d(0,0,0)
      );
     }
     fuelpublisher.set(new Pose3d[] {fuelViz[0],fuelViz[1],fuelViz[2],fuelViz[3],fuelViz[4],fuelViz[5],fuelViz[6],fuelViz[7],fuelViz[8],fuelViz[9],fuelViz[10],fuelViz[11],fuelViz[12],fuelViz[13],fuelViz[14],fuelViz[15],fuelViz[16],fuelViz[17],fuelViz[18],fuelViz[19]} );
    }
    }
    

    
}
