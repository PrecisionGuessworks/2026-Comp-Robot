package frc.robot.subsystems;

import dev.doglog.DogLog;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
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
import frc.robot.RobotContainer;

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

  

  private final QuixTalonFX m_deployMotor =
      new QuixTalonFX(
          Constants.Intake.deployMotorID,
          Constants.Intake.deployMotorRatio,
          QuixTalonFX.makeDefaultConfig()
              .setBrakeMode()
              .setSupplyCurrentLimit(30.0)
              .setStatorCurrentLimit(40.0)
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
  private double m_intakeABsetpoint = 0.0;
  private double m_intakeCsetpoint = 0.0;
  private boolean m_intakeCjam = false;
  private boolean m_intakeCjamCool = false;
  private Timer m_antiJamTimer = new Timer();
  private Timer m_antiJamTimerCool = new Timer();
  private Timer m_antiJamTimerSpinup = new Timer();
  public boolean m_hasPiece = false;
  public boolean m_attackMode = false;
  public boolean m_pastAttackMode = false;
  private boolean m_softLimitsEnabled = true;

  private final Color Yellow = new Color(255, 255, 0);
  private final Color Blue = new Color(0, 0, 255);
  private Color m_currentColor = Blue;

  public IntakeSubsystem() {
    // m_lastPieceTimer.start();
    // m_lastPieceTimer.reset();

    // Show scheduler status in SmartDashboard.
    // SmartDashboard.putData(this);
  }

  // public boolean hasPiece() {
  //   //m_rollerMotor.getSupplyCurrent();
  //   return m_hasPiece;
  // }

  // public void setHasPiece(boolean hasPiece) {
  //   m_hasPiece = hasPiece;
  // }

  // public boolean recentlyHadPiece() {
  //   return m_lastPieceTimer.get() < 1.0;
  // }

    public double getPosition() {
    return Constants.Intake.deployMotorRatio.sensorRadiansToMechanismPosition(m_deployMotor.getSensorPosition());
  }

  public void setPosition(double targetPosition) {
    m_targetPosition = targetPosition;
  }

  public void setManualHeight(double speed) { // Speed should be between -1 and 1, where positive is up and negative is down
    m_targetPosition += speed * Constants.Intake.ManualSpeed;
  }

  public boolean isAtPosition(double position, double tolerance) {
    return Math.abs(position - getPosition()) <= tolerance;
  }

  // public boolean isAtTarget(double tolerance) {
  //    isAtPosition(m_targetPosition, tolerance);
  //    return
  // }

  public void setABRollerVelocity(double velocity) {
    m_intakeABsetpoint = velocity;
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
    m_antiJamTimerSpinup.restart();
    m_intakeCsetpoint = velocity;
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
        // Robot.lights.setAttack();
      } else {
        m_targetPosition = Constants.Intake.defPosition;
      }
  }

  public void retractIntakeSlowShoot() {
    if (!RobotContainer.driver.rightTrigger().getAsBoolean()) {
    m_targetPosition -= Constants.Intake.retractSlowSpeed;
  setABRollerVelocity(Constants.Intake.SlowABRollerVelocity);
  setCRollerVelocity(Constants.Intake.SlowCRollerVelocity);
  }
  }

  public void retractIntakeSlowShootSTOP() {
    if (!RobotContainer.driver.rightTrigger().getAsBoolean()) {
    setABRollerVelocity(0);
    setCRollerVelocity(0);
  }
  }

  public void setSoftLimitsEnabled(boolean enabled) {
    m_deployMotor.setForwardSoftLimit(enabled);
    m_deployMotor.setReverseSoftLimit(enabled);
    m_softLimitsEnabled = enabled;
  }
  
  public void toggleSoftLimitsEnabled() {
    boolean enabled = m_softLimitsEnabled;
    setSoftLimitsEnabled(!enabled);
    m_softLimitsEnabled = !enabled;
    if (m_softLimitsEnabled) {
    zeroDeploy();
    }  
  }

  public void setDeployCurrent (double stator, double supply){
    m_deployMotor.setStatorCurrentLimit(stator,supply);
  }

  public void zeroDeploy() {
    m_deployMotor.setSensorPosition(0.0);
  }

  public void retractIntakeHome() {
    m_targetPosition -= Constants.Intake.retractHomeSpeed;
  }

  public double getDepolyMotorCurrent() {
    return m_deployMotor.getSupplyCurrent();
  }

  public double getDeployVelocity() {
    return m_deployMotor.getSensorVelocity();
  }

  public boolean isDeployStalled() { // Maybe use velocity and current to determine if stalled?
    return getDepolyMotorCurrent() >= Constants.Intake.retractHomeStatorCurrent;
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

    if (getCRollerVelocity() < 100 && m_intakeCsetpoint > 30 && !m_intakeCjam && m_antiJamTimerSpinup.hasElapsed(Constants.Intake.antiJamCRollerTimeSpinup)) {
      m_intakeCjam = true;
      m_antiJamTimerCool.start();
      setCRollerVelocity(0);
      m_intakeCjamCool = true;
    }
    if (m_intakeCsetpoint == 0&&m_intakeCjam){
      m_intakeCjam = false;
      m_intakeCjamCool = false;
      m_antiJamTimer.reset();
      m_antiJamTimerCool.reset();
    }
    if (m_intakeCjam) {

      if (m_intakeCjamCool && m_antiJamTimerCool.hasElapsed(Constants.Intake.antiJamCRollerTimeCool)) {
      m_antiJamTimerCool.reset();
      m_antiJamTimer.start();
      setCRollerVelocity(m_intakeCsetpoint);
      m_intakeCjamCool = false;

      if (!m_intakeCjamCool && m_antiJamTimer.hasElapsed(Constants.Intake.antiJamCRollerTime)) {
        m_antiJamTimer.reset();
        m_antiJamTimerCool.start();
        setCRollerVelocity(0);
      m_intakeCjamCool = true;
  
      }



    }


    }
    

    if (m_attackMode) {
      m_currentColor = Yellow;
    } else {
      m_currentColor = Blue;
    }
    SmartDashboard.putString("Intake Mode", m_currentColor.toHexString());

    if (!m_softLimitsEnabled){
      m_setPosition = m_targetPosition;
    } else if (m_targetPosition >= Constants.Intake.minExtension && m_targetPosition <= Constants.Intake.maxExtension) {
      m_setPosition = m_targetPosition;
    }

    m_deployMotor.setMotionMagicPositionSetpoint(
        Constants.Intake.deployPositionSlot, m_setPosition);


    DogLog.log("Intake/ Attack Mode", m_attackMode);
    DogLog.log("Intake/ Position", Units.metersToInches(getPosition()),"In");
    DogLog.log("Intake/ Target Position", Units.metersToInches(Constants.Intake.deployMotorRatio.sensorRadiansToMechanismPosition(m_deployMotor.getClosedLoopReference())),"In");
    DogLog.log("Intake/ Target set Position", Units.metersToInches(m_targetPosition),"In");

    DogLog.log("Intake/ AB Roller Velocity", getABRollerVelocity(),"Rad/s");
    DogLog.log("Intake/ AB Roller Setpoint", m_intakeABsetpoint,"Rad/s");
    DogLog.log("Intake/ AB Roller Target Velocity", m_ABrollerMotor.getClosedLoopReference(),"Rad/s");

    DogLog.log("Intake/ C Roller Velocity", getCRollerVelocity(),"Rad/s");
    DogLog.log("Intake/ C Roller Setpoint", m_intakeCsetpoint,"Rad/s");
    DogLog.log("Intake/ C Roller Target Velocity", m_CrollerMotor.getClosedLoopReference(),"Rad/s");
    

    m_ABrollerMotor.logMotorState();
    m_CrollerMotor.logMotorState();
    m_deployMotor.logMotorState();
    
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
        // m_elevatorSim.getVelocityMetersPerSecond(), // Fix: Figure out why this causes jitter
        0.0,
        TimedRobot.kDefaultPeriod,
        Constants.Intake.deployMotorRatio);


    m_rollerSim.setInput(m_CrollerMotor.getPercentOutput() * RobotController.getBatteryVoltage());
    m_rollerSim.update(TimedRobot.kDefaultPeriod);
    m_CrollerMotor.setSimSensorVelocity(
        m_rollerSim.getAngularVelocityRadPerSec(),
        TimedRobot.kDefaultPeriod,
        Constants.Intake.rollerMotorRatio);


  }
  // --- END STUFF FOR SIMULATION ---
}
