package frc.robot.subsystems;

import com.ctre.phoenix6.signals.SensorDirectionValue;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.quixlib.devices.QuixCANCoder;
import frc.quixlib.motorcontrol.QuixTalonFX;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ShooterSubsystem extends SubsystemBase {
  //public final DigitalInput m_beamBreak = new DigitalInput(Constants.Shooter.beamBreakPort);

  static private final QuixCANCoder m_armCoder = 
      new QuixCANCoder(Constants.Shooter.armCoderID, Constants.Shooter.armMotorRatio, SensorDirectionValue.Clockwise_Positive);
  
  // static private final QuixAbsoluteEncoder m_armCoder = 
  //     new QuixAbsoluteEncoder(Constants.Shooter.armCoderID, Constants.Shooter.armMotorRatio, SensorDirectionValue.Clockwise_Positive);

      static double ArmStartingAngle = Constants.Shooter.armStartingAngle;
      public double shooterTargetVelocity = 0.0;
     // : Units.rotationsToRadians(m_armCoder.getAbsPosition()); Constants.isSim ? 
  private final QuixTalonFX m_shooterMotor =
      new QuixTalonFX(
          Constants.Shooter.shooterMotorID,
          Constants.Shooter.shooterMotorRatio,
          QuixTalonFX.makeDefaultConfig()
              .setInverted(Constants.Shooter.shooterMotorInvert)
              .setSupplyCurrentLimit(50.0)
              .setStatorCurrentLimit(80.0)
              .setPIDConfig(Constants.Shooter.shooterVelocityPIDSlot, Constants.Shooter.shooterPositionPIDConfig));

  private final QuixTalonFX m_hoodMotor =
      new QuixTalonFX(
          Constants.Shooter.hoodMotorID,
          Constants.Shooter.hoodMotorRatio,
          QuixTalonFX.makeDefaultConfig()
              .setInverted(Constants.Shooter.hoodMotorInvert)
              .setBrakeMode()
              .setSupplyCurrentLimit(40.0)
              .setStatorCurrentLimit(80.0)
              .setMotionMagicConfig(
                  Constants.Shooter.ArmConstraints.maxVelocity,
                  Constants.Shooter.ArmConstraints.maxAcceleration,
                  Constants.Shooter.ArmMaxJerk,
                  Constants.Shooter.armExpo_kV,
                  Constants.Shooter.armExpo_kA)

              .setPIDConfig(Constants.Shooter.armPositionPIDSlot, Constants.Shooter.armPositionPIDConfig)
              .setBootPositionOffset(ArmStartingAngle)
              // .setReverseSoftLimit(Constants.Shooter.armMinAngle)
              // .setForwardSoftLimit(Constants.Shooter.armMaxAngle)
              // .setFeedbackConfig(FeedbackSensorSourceValue.FusedCANcoder, 15, 0.0,Constants.Shooter.armMotorRatio,Constants.Shooter.armSensorRatio)
              );


  private double m_armTargetAngle = ArmStartingAngle;
  private double setm_armTargetAngle = ArmStartingAngle;
  private boolean hasPiece = true;
  private Timer m_lastPieceTimer = new Timer();

  public ShooterSubsystem() {
    m_lastPieceTimer.start();
    m_lastPieceTimer.reset();

    // Show scheduler status in SmartDashboard.
    SmartDashboard.putData(this);
  }

  // public boolean hasPiece() {
  //   return m_beamBreak.get();
  // }

  public boolean recentlyHadPiece() {
    return m_lastPieceTimer.get() < 1.0;
    }

    public double getHoodAngle() { 
    return Units.radiansToDegrees(m_hoodMotor.getSensorPosition()) * Constants.Shooter.hoodMotorRatio.inverseReduction() + Units.radiansToDegrees(ArmStartingAngle) ;
    //: Units.rotationsToRadians(m_armCoder.getAbsPosition());   Constants.isSim ? 
    }
    
  public double getshooterCurrent() {
    return m_shooterMotor.getSupplyCurrent();
  }

  public double getHoodCoder(){
    return Units.rotationsToDegrees(m_armCoder.getAbsPosition());
  }

  public void setHoodAngle(double targetAngle) {
    setm_armTargetAngle = targetAngle;
  }

  public void setHasPiece(boolean thasPiece) {
    hasPiece = thasPiece;
  }
  public boolean getHasPiece() {
    return hasPiece;
  }


  public void seShooterCurrent(double StatorCurrentLimit, double SupplyCurrentLimit) {
    m_shooterMotor.setStatorCurrentLimit(StatorCurrentLimit,SupplyCurrentLimit);
  }

  public boolean isAtAngle(double angle, double tolerance) {
    return Math.abs(angle - m_hoodMotor.getSensorPosition()) <= tolerance;
  }

  public boolean isShooterStalled() {
    //return Math.abs(m_shooterMotor.getSensorVelocity()) < Constants.Shooter.shooterStallVelocity;
    //return m_shooterMotor.getSupplyCurrent() > Constants.Shooter.shooterStallCurrent;
  return false;
  }

  public void setShooterVelocity(double velocity) {
    shooterTargetVelocity = velocity;
    if (velocity == 0.0) {
      m_shooterMotor.setPercentOutput(0.0);
    } else {
      m_shooterMotor.setVelocitySetpoint(
          Constants.Shooter.shooterVelocityPIDSlot,
          velocity,
          Constants.Shooter.shooterFeedforward.calculate(velocity));
    }
  }

  // public void disabledInit() {
  //   m_armMotor.setBrakeMode(true);
  // }

  // public void disabledExit() {
  //   m_armMotor.setBrakeMode(false);
  // }

  //private double elevatorLocation = 0; 

  @Override
  public void periodic() {


    if (getHoodAngle() <= 115 && getArmAngle() >= 92 && Units.radiansToDegrees(setm_armTargetAngle) < 92 && !RobotContainer.elevator.isAtHeight(Constants.Elevator.stowHeight, Units.inchesToMeters(.75))) { // might need check 
       m_armTargetAngle = Constants.Shooter.armStowIntakeAngle;
      //System.out.println("1");
    }
    

    // if(hasPiece){
    //   m_armMotor.setMotionMagicPositionSetpoint(
    //     Constants.Shooter.armCoralPositionPIDSlot, m_armTargetAngle);
    //   m_wristMotor.setMotionMagicPositionSetpoint(
    //     Constants.Shooter.wristCoralPositionPIDSlot, m_wristTargetAngle);
    // } else {
      // m_armMotor.setMotionMagicPositionSetpoint(
      //   Constants.Shooter.armPositionPIDSlot, m_armTargetAngle);
      m_armMotor.setMotionMagicPositionSetpointExpo(
          Constants.Shooter.armPositionPIDSlot, m_armTargetAngle);


   // }
    
    //  Logging
    DogLog.log("Hood: Current Angle (deg)", Units.radiansToDegrees(m_armMotor.getSensorPosition()),"deg");
    DogLog.log("Hood: Current CANcoder Angle (deg)", getHoodCoder(),"deg");
    DogLog.log("Hood: Real Current Angle (deg)", getHoodAngle(),"deg");
    DogLog.log("Hood: Target Angle (deg)", Units.radiansToDegrees(m_armMotor.getClosedLoopReference()),"deg");
    DogLog.log("Hood: Target set Angle (deg)", Units.radiansToDegrees(m_armTargetAngle),"deg");
    DogLog.log("Hood: Current Velocity (deg per sec)", Units.radiansToDegrees(m_armMotor.getSensorVelocity()),"deg per sec");

    m_shooterMotor.logMotorState();
    m_hoodMotor.logMotorState();
    // m_hoodCoder.logSensorState();
  

  }

  // --- BEGIN STUFF FOR SIMULATION ---
  
  private static final SingleJointedArmSim m_hoodSim =
      new SingleJointedArmSim(
          DCMotor.getKrakenX44Foc(1),
          Constants.Shooter.hoodMotorRatio.reduction(),
          Constants.Shooter.simHoodMOI,
          Constants.Shooter.simHoodCGLength,
          Constants.Shooter.hoodMinAngle,
          Constants.Shooter.hoodMaxAngle,
          true, // Simulate gravity
          ArmStartingAngle);

  static final DCMotor m_simMotor = DCMotor.getKrakenX60Foc(1);
  private static final FlywheelSim m_shooterSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              m_simMotor,
              Constants.Shooter.simshooterMOI,
              Constants.Shooter.shooterMotorRatio.reduction()),
          m_simMotor);


  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    m_armSim.setInput(m_armMotor.getPercentOutput() * RobotContshooter.getBatteryVoltage());
    m_armSim.update(TimedRobot.kDefaultPeriod);
    m_armMotor.setSimSensorPositionAndVelocity(
        m_armSim.getAngleRads() - ArmStartingAngle,
        // m_armSim.getVelocityRadPerSec(), // TODO: Figure out why this causes jitter
        0.0,
        TimedRobot.kDefaultPeriod,
        Constants.Shooter.armMotorRatio);
    

    m_shooterSim.setInput(m_shooterMotor.getPercentOutput() * RobotContshooter.getBatteryVoltage());
    m_shooterSim.update(TimedRobot.kDefaultPeriod);
    m_shooterMotor.setSimSensorVelocity(
        m_shooterSim.getAngularVelocityRadPerSec(),
        TimedRobot.kDefaultPeriod,
        Constants.Shooter.armMotorRatio);

    // Update arm viz.
   
  }
  // --- END STUFF FOR SIMULATION ---
}