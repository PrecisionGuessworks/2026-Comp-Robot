package frc.robot.subsystems;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.quixlib.motorcontrol.QuixTalonFX;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class HopperSubsystem extends SubsystemBase {
private final QuixTalonFX m_hopperMotor =
      new QuixTalonFX(
          Constants.Hopper.hopperMotorID,
          Constants.Hopper.hopperMotorRatio,
          QuixTalonFX.makeDefaultConfig()
              .setInverted(Constants.Hopper.hopperMotorInvert)
              .setSupplyCurrentLimit(40.0)
              .setStatorCurrentLimit(80.0)
              .setBrakeMode()
              .setPIDConfig(Constants.Hopper.rollerVelocitySlot, Constants.Hopper.rollerPIDConfig));
private double hopperTargetVelocity = 0.0;
private double hoppersetTargetVelocity = 0.0;

public HopperSubsystem() {

  }

public void setHopperRollerVelocity(double velocity) {
    hopperTargetVelocity = velocity;

  }

  public void setHopperRollerCurrent (double stator, double supply){
    m_hopperMotor.setStatorCurrentLimit(stator,supply);
  }

  public double getHopperRollerCurrent (){
    return m_hopperMotor.getSupplyCurrent();
  }

  public double getHopperRollerVelocity() {
    return m_hopperMotor.getSensorVelocity();
  }


    @Override
  public void periodic() {
    if (RobotContainer.driver.rightTrigger().getAsBoolean() && DriverStation.isTeleop()) {
      hoppersetTargetVelocity = Constants.Hopper.hopperIntakeVelocity;
    } else {
      hoppersetTargetVelocity = hopperTargetVelocity;
    }

    
        if (hoppersetTargetVelocity == 0.0) {
      m_hopperMotor.setPercentOutput(0.0);
    } else {
      m_hopperMotor.setVelocitySetpoint(
          Constants.Hopper.rollerVelocitySlot,
          hoppersetTargetVelocity,
          Constants.Hopper.rollerFeedforward.calculate(hoppersetTargetVelocity));
    }

    DogLog.log("Hopper/ Current Velocity (rotations per sec)", m_hopperMotor.getSensorVelocity(),"rotations per sec");
    DogLog.log("Hopper/ Target set Velocity (rotations per sec)", hoppersetTargetVelocity,"rotations per sec");
    m_hopperMotor.logMotorState();
  }

    
}
