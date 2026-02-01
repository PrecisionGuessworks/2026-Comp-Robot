package frc.robot.subsystems;

import dev.doglog.DogLog;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.quixlib.motorcontrol.QuixTalonFX;
import frc.quixlib.motorcontrol.QuixTalonFXS;
import frc.robot.Constants;
import frc.robot.Robot;

public class IntakeSubsystem extends SubsystemBase {
  // public final DigitalInput m_beamBreak = new DigitalInput(Constants.Intake.beamBreakPort);
  

  private final QuixTalonFX m_ABrollerMotor =
      new QuixTalonFX(
          Constants.Intake.ABrollerID,
          Constants.Intake.ABrollerRatio,
          QuixTalonFX.makeDefaultConfig()
              .setInverted(Constants.Intake.rollerMotorInvert)
              .setSupplyCurrentLimit(40.0)
              .setStatorCurrentLimit(80.0)
              .setBrakeMode()
              .setPIDConfig(Constants.Intake.rollerVelocitySlot, Constants.Intake.rollerPIDConfig));

  private final QuixTalonFXS m_CrollerMotor =
      new QuixTalonFXS(
          Constants.Intake.CrollerID,
          Constants.Intake.CrollerRatio,
          Constants.Intake.CrollerArrangement,
          QuixTalonFXS.makeDefaultConfig()
              .setInverted(Constants.Intake.rollerMotorInvert)
              .setSupplyCurrentLimit(40.0)
              .setStatorCurrentLimit(80.0)
              .setBrakeMode()
              .setPIDConfig(Constants.Intake.rollerVelocitySlot, Constants.Intake.rollerPIDConfig));

  private final QuixTalonFX m_hopperMotor =
      new QuixTalonFX(
          Constants.Intake.hopperMotorID,
          Constants.Intake.hopperMotorRatio,
          QuixTalonFX.makeDefaultConfig()
              .setInverted(Constants.Intake.hopperMotorInvert)
              .setSupplyCurrentLimit(40.0)
              .setStatorCurrentLimit(80.0)
              .setBrakeMode()
              .setPIDConfig(Constants.Intake.rollerVelocitySlot, Constants.Intake.rollerPIDConfig));

  private final QuixTalonFX m_deployMotor =
      new QuixTalonFX(
          Constants.Intake.deployMotorID,
          Constants.Intake.deployMotorRatio,
          QuixTalonFX.makeDefaultConfig()
              .setBrakeMode()
              .setSupplyCurrentLimit(40.0)
              .setStatorCurrentLimit(120.0)
              .setInverted(Constants.Intake.deployMotorInvert)
              .setPIDConfig(Constants.Intake.deployPositionSlot, Constants.Intake.deployPIDConfig)
              .setMotionMagicConfig(
                  Constants.Intake.deployMaxVelocity,
                  Constants.Intake.deployMaxAcceleration,
                  Constants.Intake.deployMaxJerk,
                  Constants.Intake.Expo_kV,
                  Constants.Intake.Expo_kA)
              .setReverseSoftLimit(Constants.Intake.minExtension)
              .setForwardSoftLimit(Constants.Intake.maxExtension));


  private double m_targetPosition = Constants.Intake.startingPosition; 
  private double m_setPosition = m_targetPosition;
  private Timer m_lastPieceTimer = new Timer();
  public boolean m_hasPiece = false;
  public boolean m_attackMode = false;
  public boolean m_pastAttackMode = false;

  private final Color Yellow = new Color(255, 255, 0);
  private final Color Blue = new Color(0, 0, 255);
  private Color m_currentColor = Blue;

  public IntakeSubsystem() {
    m_lastPieceTimer.start();
    m_lastPieceTimer.reset();

    // Show scheduler status in SmartDashboard.
    SmartDashboard.putData(this);
  }

  public boolean hasPiece() {
    //m_rollerMotor.getSupplyCurrent();
    return m_hasPiece;
  }

  public void setHasPiece(boolean hasPiece) {
    m_hasPiece = hasPiece;
  }

  public boolean recentlyHadPiece() {
    return m_lastPieceTimer.get() < 1.0;
  }

    public double getPosition() {
    return Constants.Intake.deployMotorRatio.sensorRadiansToMechanismPosition(m_deployMotor.getSensorPosition());
  }

  public void setPosition(double targetPosition) {
    m_targetPosition = targetPosition;
  }

  public boolean isAtPosition(double position, double tolerance) {
    return Math.abs(position - getPosition()) <= tolerance;
  }

  // public boolean isAtTarget(double tolerance) {
  //    isAtPosition(m_targetPosition, tolerance);
  //    return
  // }

  public void setABRollerVelocity(double velocity) {
    if (velocity == 0.0) {
      m_ABrollerMotor.setPercentOutput(0.0);
    } else {
      m_ABrollerMotor.setVelocitySetpoint(
          Constants.Intake.rollerVelocitySlot,
          velocity,
          Constants.Intake.rollerFeedforward.calculate(velocity));
    }
  }

  public void setABRollerCurrent (double stator, double supply){
    m_ABrollerMotor.setStatorCurrentLimit(stator,supply);
  }

  public double getABRollerCurrent (){
    return m_ABrollerMotor.getSupplyCurrent();
  }

  public double getABRollerVelocity() {
    return m_ABrollerMotor.getSensorVelocity();
  }

   public void setCRollerVelocity(double velocity) {
    if (velocity == 0.0) {
      m_CrollerMotor.setPercentOutput(0.0);
    } else {
      m_CrollerMotor.setVelocitySetpoint(
          Constants.Intake.rollerVelocitySlot,
          velocity,
          Constants.Intake.rollerFeedforward.calculate(velocity));
    }
  }

  public void setCRollerCurrent (double stator, double supply){
    m_CrollerMotor.setStatorCurrentLimit(stator,supply);
  }

  public double getCRollerCurrent (){
    return m_CrollerMotor.getSupplyCurrent();
  }

  public double getCRollerVelocity() {
    return m_CrollerMotor.getSensorVelocity();
  }


  public void setAttackMode(boolean attackMode) {
    m_attackMode = attackMode;
    setIntakePosition();
  }

  public boolean getAttackMode() {
    return m_attackMode;
  }

  public void flipAttackMode() {
    m_attackMode = !m_attackMode;
    setIntakePosition();
  }

  public void setIntakePosition() {
    if (m_attackMode) {
        m_targetPosition = Constants.Intake.attackPosition;
        Robot.lights.setAttack();
      } else {
        m_targetPosition = Constants.Intake.defPosition;
      }
  }

  public void retractIntakeSlow() {
    m_targetPosition -= Constants.Intake.retractSlowSpeed;
  }


  @Override
  public void periodic() {


    // if (m_attackMode != m_pastAttackMode) {
    //   if (m_attackMode) {
    //     m_targetPosition = Constants.Intake.attackPosition;
    //   } else {
    //     m_targetPosition = Constants.Intake.defPosition;
    //   }
    //   m_pastAttackMode = m_attackMode;
    // }


    if (m_attackMode) {
      m_currentColor = Yellow;
    } else {
      m_currentColor = Blue;
    }
    SmartDashboard.putString("Intake Mode", m_currentColor.toHexString());

    if (m_targetPosition >= Constants.Intake.minExtension && m_targetPosition <= Constants.Intake.maxExtension) {
      m_setPosition = m_targetPosition;
    }

    m_deployMotor.setMotionMagicPositionSetpoint(
        Constants.Intake.deployPositionSlot, m_setPosition);


    DogLog.log("Intake: Attack Mode", m_attackMode);
    DogLog.log("Intake: Position", Units.metersToInches(getPosition()),"In");
    DogLog.log("Intake: Target Position", Units.metersToInches(Constants.Intake.deployMotorRatio.sensorRadiansToMechanismPosition(m_deployMotor.getClosedLoopReference())),"In");
    DogLog.log("Intake: Target set Position", Units.metersToInches(m_targetPosition),"In");

    DogLog.log("Intake: AB Roller Velocity", getABRollerVelocity(),"Rad/s");
    DogLog.log("Intake: AB Roller Target Velocity", m_ABrollerMotor.getClosedLoopReference(),"Rad/s");

    DogLog.log("Intake: C Roller Velocity", getCRollerVelocity(),"Rad/s");
    DogLog.log("Intake: C Roller Target Velocity", m_CrollerMotor.getClosedLoopReference(),"Rad/s");
    

    m_ABrollerMotor.logMotorState();
    m_CrollerMotor.logMotorState();
    m_deployMotor.logMotorState();
    m_hopperMotor.logMotorState();
    
    // m_deployFollower.logMotorState();
  }

  // --- BEGIN STUFF FOR SIMULATION ---
  private static final ElevatorSim m_elevatorSim =
      new ElevatorSim(
          DCMotor.getKrakenX60Foc(1),
          Constants.Intake.deployMotorRatio.reduction(),
          Constants.Intake.simCarriageMass,
          Constants.Intake.sprocketPitchDiameter * 0.5,
          Constants.Intake.minExtension,
          Constants.Intake.maxExtension,
          false,
          0);

  static final DCMotor m_simMotor = DCMotor.getKrakenX60Foc(1);
  private static final FlywheelSim m_rollerSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              m_simMotor,
              Constants.Intake.simRollerMOI,
              Constants.Intake.rollerMotorRatio.reduction()),
          m_simMotor);

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    m_elevatorSim.setInput(m_deployMotor.getPercentOutput() * RobotController.getBatteryVoltage());
    m_elevatorSim.update(TimedRobot.kDefaultPeriod);
    m_deployMotor.setSimSensorPositionAndVelocity(
        m_elevatorSim.getPositionMeters(),
        // m_elevatorSim.getVelocityMetersPerSecond(), // TODO: Figure out why this causes jitter
        0.0,
        TimedRobot.kDefaultPeriod,
        Constants.Climber.motorRatio);


    m_rollerSim.setInput(m_CrollerMotor.getPercentOutput() * RobotController.getBatteryVoltage());
    m_rollerSim.update(TimedRobot.kDefaultPeriod);
    m_CrollerMotor.setSimSensorVelocity(
        m_rollerSim.getAngularVelocityRadPerSec(),
        TimedRobot.kDefaultPeriod,
        Constants.Intake.deployMotorRatio);


  }
  // --- END STUFF FOR SIMULATION ---
}
