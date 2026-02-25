package frc.WorBots.subsystems.turret;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class TurretIOSim implements TurretIO {

  private final DCMotorSim turretMotor = 
    new DCMotorSim(
      LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 1.0, 1.0),
       DCMotor.getKrakenX60(1));

  @Override
  public void setVoltage(double volts) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setVoltage'");
  }


  @Override
  public void updateInputs(TurretIOInputs inputs) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'updateInputs'");
  }

  public void resetOffset(){
    //TODO make this
  }

    //TODO add turretIOSim
}
