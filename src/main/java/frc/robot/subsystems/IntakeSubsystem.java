package frc.robot.subsystems;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.quixlib.motorcontrol.QuixTalonFX;
import frc.quixlib.viz.Link2d;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  public final DigitalInput m_beamBreak = new DigitalInput(Constants.Intake.beamBreakPort);
  

  private final QuixTalonFX m_rollerMotor =
      new QuixTalonFX(
          Constants.Intake.rollerMotorID,
          Constants.Intake.rollerMotorRatio,
          QuixTalonFX.makeDefaultConfig()
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
              .setSupplyCurrentLimit(40.0)
              .setStatorCurrentLimit(90.0)
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
  private Timer m_lastPieceTimer = new Timer();
  public boolean m_hasPiece = false;

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
    return Constants.Climber.motorRatio.sensorRadiansToMechanismPosition(m_deployMotor.getSensorPosition());
  }

  public void setPosition(double targetPosition) {
    m_targetPosition = targetPosition;
  }

  public boolean isAtPosition(double position, double tolerance) {
    return Math.abs(position - getPosition()) <= tolerance;
  }

  public void setRollerVelocity(double velocity) {
    if (velocity == 0.0) {
      m_rollerMotor.setPercentOutput(0.0);
    } else {
      m_rollerMotor.setVelocitySetpoint(
          Constants.Intake.rollerVelocitySlot,
          velocity,
          Constants.Intake.rollerFeedforward.calculate(velocity));
    }
  }

  public void setRollerCurrnt (double current){
    m_rollerMotor.setStatorCurrentLimit(current);
  }

  public double getRollerCurrent (){
    return m_rollerMotor.getSupplyCurrent();
  }

    public double getRollerVelocity() {
    return m_rollerMotor.getSensorVelocity();
  }


  // public void disabledInit() {
  //   m_deployMotor.setBrakeMode(true);
  // }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (hasPiece()) {
      m_lastPieceTimer.reset();
    }

    // SmartDashboard.putBoolean("Intake: Beam Break", m_beamBreak.get());
    // SmartDashboard.putBoolean("Intake: Has Piece", hasPiece());

    m_deployMotor.setMotionMagicPositionSetpoint(
        Constants.Intake.deployPositionSlot, m_targetPosition);

    DogLog.log("Intake: Position", Units.metersToInches(getPosition()),"In");
    DogLog.log("Intake: Target Position", Units.metersToInches(Constants.Intake.deployMotorRatio.sensorRadiansToMechanismPosition(m_deployMotor.getClosedLoopReference())),"In");
    DogLog.log("Intake: Target set Position", Units.metersToInches(m_targetPosition),"In");

    m_rollerMotor.logMotorState();
    m_deployMotor.logMotorState();
    // m_deployFollower.logMotorState();
  }

  // --- BEGIN STUFF FOR SIMULATION ---
  private static final ElevatorSim m_elevatorSim =
      new ElevatorSim(
          DCMotor.getKrakenX60Foc(2),
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

    m_rollerSim.setInput(m_rollerMotor.getPercentOutput() * RobotController.getBatteryVoltage());
    m_rollerSim.update(TimedRobot.kDefaultPeriod);
    m_rollerMotor.setSimSensorVelocity(
        m_rollerSim.getAngularVelocityRadPerSec(),
        TimedRobot.kDefaultPeriod,
        Constants.Intake.deployMotorRatio);


  }
  // --- END STUFF FOR SIMULATION ---
}
