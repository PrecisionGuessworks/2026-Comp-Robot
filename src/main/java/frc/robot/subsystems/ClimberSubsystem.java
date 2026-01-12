package frc.robot.subsystems;

import dev.doglog.DogLog;
import static edu.wpi.first.units.Units.Inches;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.quixlib.motorcontrol.QuixTalonFX;
import frc.quixlib.viz.Link2d;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class ClimberSubsystem extends SubsystemBase {
  private final QuixTalonFX m_motor =
      new QuixTalonFX(
          Constants.Climber.motorID,
          Constants.Climber.motorRatio,
          QuixTalonFX.makeDefaultConfig()
              .setBrakeMode()
              .setSupplyCurrentLimit(40.0)
              .setStatorCurrentLimit(90.0)
              .setInverted(Constants.Climber.motorInvert)
              .setPIDConfig(Constants.Climber.motorPositionSlot, Constants.Climber.motorPIDConfig)
              .setMotionMagicConfig(
                  Constants.Climber.maxVelocity,
                  Constants.Climber.maxAcceleration,
                  Constants.Climber.maxJerk,
                  Constants.Climber.Expo_kV,
                  Constants.Climber.Expo_kA)
              .setReverseSoftLimit(Constants.Climber.minHeight)
              .setForwardSoftLimit(Constants.Climber.maxHeight));
  private final QuixTalonFX m_follower = new QuixTalonFX(
      Constants.Climber.followerID,
      m_motor,
      Constants.Climber.followerInvert,
      QuixTalonFX.makeDefaultConfig().setBrakeMode()
      .setSupplyCurrentLimit(40.0)
      .setStatorCurrentLimit(90.0)
      .setInverted(Constants.Climber.motorInvert)
      .setPIDConfig(Constants.Climber.motorPositionSlot, Constants.Climber.motorPIDConfig)
      .setMotionMagicConfig(
          Constants.Climber.maxVelocity,
          Constants.Climber.maxAcceleration,
          Constants.Climber.maxJerk)
      .setReverseSoftLimit(Constants.Climber.minHeight)
      .setForwardSoftLimit(Constants.Climber.maxHeight));

  private double m_setTargetHeight = Constants.Climber.minHeight;
  private double m_targetHeight = Constants.Climber.minHeight;
  public int m_HeightLocation = 4;
  private boolean m_ElevatorOff = false;
  private boolean m_ElevatorOffLast = m_ElevatorOff;

  public ClimberSubsystem() {
    // Show scheduler status in SmartDashboard.
    SmartDashboard.putData(this);

  }

  // public boolean isAtScore(){
  //   if (m_HeightLocation == 4){
  //     return isAtHeight(Constants.Elevator.L4, Units.inchesToMeters(3));
  //   } else if (m_HeightLocation == 3){
  //     return isAtHeight(Constants.Elevator.L3, Units.inchesToMeters(3));
  //   } else if (m_HeightLocation == 2){
  //     return isAtHeight(Constants.Elevator.L2, Units.inchesToMeters(3));
  //   } else if (m_HeightLocation == 1){
  //     return isAtHeight(Constants.Elevator.L1, Units.inchesToMeters(3));
  //   } else {
  //     return false;
  //   }
  // }

  public double getHeight() {
    return Constants.Climber.motorRatio.sensorRadiansToMechanismPosition(m_motor.getSensorPosition());
  }

  public void setHeight(double targetHeight) {
    m_setTargetHeight = targetHeight;
  }
  public void setHeightLocation(int targetHeight) {
    m_HeightLocation = targetHeight;
  }
  public int getHeightLocation() {
    return m_HeightLocation;
  }

  public boolean isAtHeight(double height, double tolerance) {
    return Math.abs(height - getHeight()) <= tolerance;
  }
  private double armAngle = 0;
  private double wristAngle = 0;

  public void setElevatorOn(boolean lineup){
    m_ElevatorOff = lineup;
}
public boolean getElevatorOn(){
    return m_ElevatorOff;
}

  @Override
  public void periodic() {
    // armAngle = RobotContainer.shooter.gethoodAngle();
    // if (armAngle <= 50 && m_setTargetHeight <= Constants.Climber.armStowHeight){ 
    //   m_targetHeight = Constants.Climber.armStowHeight;
    // }

    //m_targetHeight = m_setTargetHeight;

    // This method will be called once per scheduler run

    if (m_ElevatorOff != m_ElevatorOffLast){
      m_ElevatorOffLast = m_ElevatorOff;
      if (m_ElevatorOff){
        m_motor.setStatorCurrentLimit(1,1);
        m_follower.setStatorCurrentLimit(1,1);
      } else {
        m_motor.setStatorCurrentLimit(70,30);
        m_follower.setStatorCurrentLimit(70,30);
      }
    }
    
    m_motor.setMotionMagicPositionSetpointExpo(
        Constants.Climber.motorPositionSlot,
        m_targetHeight
        );

    
    SmartDashboard.putBoolean(
          "Climber", !m_ElevatorOff);
    DogLog.log("Climber: On", !m_ElevatorOff);
          
    DogLog.log("Climber: Height", Units.metersToInches(getHeight()),"Inch");
    DogLog.log("Climber: Target Height", Units.metersToInches(Constants.Climber.motorRatio.sensorRadiansToMechanismPosition(m_motor.getClosedLoopReference())),"Inchs");
    DogLog.log("Climber: Target set Height", Units.metersToInches(m_targetHeight),"In");
    

    m_motor.logMotorState();
    // m_motor.updateTunerConstants0();
    m_follower.logMotorState();

  }

  // --- BEGIN STUFF FOR SIMULATION ---
  private static final ElevatorSim m_elevatorSim =
      new ElevatorSim(
          DCMotor.getKrakenX60Foc(2),
          Constants.Climber.motorRatio.reduction(),
          Constants.Climber.simCarriageMass,
          Constants.Climber.sprocketPitchDiameter * 0.5,
          Constants.Climber.minHeight,
          Constants.Climber.maxHeight,
          true,
          0);


  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    m_elevatorSim.setInput(m_motor.getPercentOutput() * RobotController.getBatteryVoltage());
    m_elevatorSim.update(TimedRobot.kDefaultPeriod);
    m_motor.setSimSensorPositionAndVelocity(
        m_elevatorSim.getPositionMeters(),
        // m_elevatorSim.getVelocityMetersPerSecond(), // TODO: Figure out why this causes jitter
        0.0,
        TimedRobot.kDefaultPeriod,
        Constants.Climber.motorRatio);

  }
  // --- END STUFF FOR SIMULATION ---
}