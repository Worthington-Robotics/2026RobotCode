package frc.WorBots.subsystems.turret;

import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class TurretIOSim implements TurretIO {
    private TurretIOInputs inputs;
    private final DCMotorSim motor = 
      new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1),0, 0), null, null);

  public TurretIOSim(){}

  @Override
  public void setVoltage(double volts) {
    motor.setInputVoltage(volts);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    inputs.turretAbsAngle = motor.getAngularPositionRad();
  
  }

  @Override
  public void setPosition(double positionRads) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setPosition'");
  }

  @Override
  public boolean atSetPoint() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'atSetPoint'");
  }

  
    

}


