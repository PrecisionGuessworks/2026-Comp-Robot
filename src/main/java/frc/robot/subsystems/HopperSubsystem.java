package frc.robot.subsystems;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.quixlib.motorcontrol.QuixTalonFX;
import frc.robot.Constants;

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

public HopperSubsystem() {

  }

public void setHopperRollerVelocity(double velocity) {
    hopperTargetVelocity = velocity;
    if (velocity == 0.0) {
      m_hopperMotor.setPercentOutput(0.0);
    } else {
      m_hopperMotor.setVelocitySetpoint(
          Constants.Hopper.rollerVelocitySlot,
          velocity,
          Constants.Hopper.rollerFeedforward.calculate(velocity));
    }
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

    DogLog.log("Hopper/ Current Velocity (rad per sec)", m_hopperMotor.getSensorVelocity(),"rad per sec");
    DogLog.log("Hopper/ Target set Velocity (rad per sec)", hopperTargetVelocity,"rad per sec");
    m_hopperMotor.logMotorState();
  }

    
}
