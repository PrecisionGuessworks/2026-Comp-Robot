package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import frc.quixlib.viz.Link2d;
import frc.quixlib.viz.Viz2d;
import frc.robot.Constants;
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

  // Elevator viz
  private  static final Link2d elevatorFrameViz =
      robotViz.addLink(
          new Link2d(
              robotViz,
              "Elevator Base",
              Constants.Viz.elevatorBaseLength,
              4.0,
              Color.kGreen,
              new Transform2d(
                  Constants.Viz.elevatorBaseX,
                  Constants.Viz.elevatorBaseY,
                  Constants.Viz.elevatorAngle)));
  private static  final Link2d elevatorCarriageViz =
      elevatorFrameViz.addLink(
          new Link2d(
              robotViz,
              "Elevator Carriage",
              Constants.Viz.elevatorCarriageLength,
              6.0,
              Color.kLightGreen));

// Intake viz
// private static final Link2d intakeArmViz =
// robotViz.addLink(
//     new Link2d(robotViz, "Intake Arm", Constants.Viz.intakeArmLength, 10.0, Color.kBlue));
// private static final Link2d intakeRollerViz =
// intakeArmViz.addLink(
//     new Link2d(robotViz, "Intake Roller", Units.inchesToMeters(1.0), 10.0, Color.kLightBlue));

//     m_intakeArmViz.setRelativeTransform(
//         new Transform2d(
//             Constants.Viz.intakePivotX,
//             Constants.Viz.intakePivotY,
//             Rotation2d.fromRadians(m_armSim.getAngleRads())));
//     m_intakeRollerViz.setRelativeTransform(
//         new Transform2d(
//             Constants.Viz.intakeArmLength,
//             0.0,
//             Rotation2d.fromRadians(
//                 m_intakeRollerViz.getRelativeTransform().getRotation().getRadians()
//                     + m_rollerSim.getAngularVelocityRadPerSec()
//                         * Constants.Viz.angularVelocityScalar)));


private static final Link2d ArmArmViz =
elevatorCarriageViz.addLink(
        new Link2d(robotViz, "Arm Arm", Constants.Viz.ArmArmLength, 10, Color.kRed));
private static final Link2d ArmWristViz =
ArmArmViz.addLink(
        new Link2d(robotViz, "Arm Wrist", Constants.Viz.ArmWristLength, 10, Color.kOrange));
private static final Link2d ArmWheelViz =
ArmWristViz.addLink(
        new Link2d(robotViz, "Arm Wheel", Units.inchesToMeters(2.0), 10, Color.kCoral));





public static void Update2DVisualization() {

        elevatorCarriageViz.setRelativeTransform(
            new Transform2d(RobotContainer.elevator.getHeight(), 0.0, new Rotation2d()));

         ArmArmViz.setRelativeTransform(
        new Transform2d(
            Constants.Viz.ArmArmPivotX,
            0,
            Rotation2d.fromRadians(Units.degreesToRadians(RobotContainer.arm.getArmAngle()) + Units.degreesToRadians(- Constants.Viz.elevatorAngle.getDegrees()))));

    ArmWristViz.setRelativeTransform(
      new Transform2d(
          Constants.Viz.ArmArmLength,
          0.0,
          Rotation2d.fromRadians(
            //m_ArmWristViz.getRelativeTransform().getRotation().getRadians()
                  + Units.degreesToRadians(RobotContainer.arm.getWristAngle()+90)+ Units.degreesToRadians(- Constants.Viz.elevatorAngle.getDegrees()) - Units.degreesToRadians(90))));

    ArmWheelViz.setRelativeTransform(
        new Transform2d(
            Constants.Viz.ArmWristLength,
            0.0,
            Rotation2d.fromRadians(
                ArmWheelViz.getRelativeTransform().getRotation().getRadians()
                    + RobotContainer.arm.RollerTargetVelocity)));

    }









    public static void Update3DVisualization() {
    // 3d viz
    final double stage1Height = Constants.Viz3d.stage1Height;
    final double CarrageHeight = RobotContainer.elevator.getHeight();
    final Pose3d stageOne =
      Constants.Viz3d.elevatorBase.transformBy(
          new Transform3d(0, 0, CarrageHeight >= stage1Height ? CarrageHeight-stage1Height : 0, new Rotation3d()));
    final Pose3d elevatorCarriage =
        Constants.Viz3d.elevatorBase.transformBy(
            new Transform3d(0, 0, CarrageHeight+ Units.inchesToMeters(0.5), new Rotation3d()));
    final Pose3d armViz = elevatorCarriage.transformBy(
        new Transform3d(0, 0, Units.inchesToMeters(7.7), new Rotation3d(0,Units.degreesToRadians( -RobotContainer.arm.getArmAngle()+90),0)));
    final Pose3d wristViz = armViz.transformBy(
        new Transform3d(0, 0, Units.inchesToMeters(11), new Rotation3d(0,Units.degreesToRadians( -RobotContainer.arm.getWristAngle()+90),0)));
    Pose3d coralViz = new Pose3d(0,0,-1, new Rotation3d());
    if (RobotContainer.arm.getHasPiece()) {
        Pose3d drive3d = new Pose3d(RobotContainer.drivetrain.getState().Pose);
        Pose3d temPose3d = wristViz.transformBy(
            new Transform3d(Units.inchesToMeters(-3), Units.inchesToMeters(2.4), Units.inchesToMeters(8), new Rotation3d(0,Units.degreesToRadians( 90),0)));
        coralViz = drive3d.transformBy(
            new Transform3d(temPose3d.getTranslation(), temPose3d.getRotation()));
    
    }

    DogLog.log("3DViz: Zeropublisher", new Pose3d());
    DogLog.log("3DViz: 1DriveBase", RobotContainer.drivetrain.getState().Pose);     
    DogLog.log("3DViz: 3ElevatorCarriage", elevatorCarriage);
    DogLog.log("3DViz: 2Stage1", stageOne);
    DogLog.log("3DViz: 4ArmViz", armViz);
    DogLog.log("3DViz: 5WristViz", wristViz);
    DogLog.log("3DViz: CoralViz", coralViz);
    }

    
}
