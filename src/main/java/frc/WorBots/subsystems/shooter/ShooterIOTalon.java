package frc.WorBots.subsystems.shooter;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import frc.WorBots.CanIDs;
import frc.WorBots.Constants;
import frc.WorBots.util.HardwareUtils;
import frc.WorBots.util.HardwareUtils.TalonSignalsPositional;

public class ShooterIOTalon implements ShooterIO {

  private final TalonFX leader = new TalonFX(CanIDs.SuperStructure.HOOD_ID, CanIDs.SuperStructure.CAN_BUS);
  private final TalonFX follower = new TalonFX(CanIDs.SuperStructure.FOLLOWER_ID, CanIDs.SuperStructure.CAN_BUS);
  private final TalonFX hood = new TalonFX(CanIDs.SuperStructure.HOOD_ID, CanIDs.SuperStructure.CAN_BUS);
  
  //Linear filter is excluded for now, pending implementation.
  
  private final TalonSignalsPositional leaderSignals;
  private final TalonSignalsPositional hoodSignals;

  public ShooterIOTalon(){
    //TODO set motor PID values
    leader.setNeutralMode(NeutralModeValue.Coast);
    follower.setNeutralMode(NeutralModeValue.Coast);
    hood.setNeutralMode(NeutralModeValue.Brake);

    //TODO: Make sure that they actually are aligned 
    follower.setControl(new Follower(CanIDs.SuperStructure.LEADER_ID, MotorAlignmentValue.Aligned));

    leaderSignals = new TalonSignalsPositional(leader);
    hoodSignals = new TalonSignalsPositional(hood);

    //TODO: Actually set these values
    HardwareUtils.setCurrentLimit(leader, 0);
    HardwareUtils.setCurrentLimit(hood, 0);

    hood.setPosition(0.0);
  }
  
  @Override
  public void setLeaderVolts(double volts){
    hood.setVoltage(volts);
  }

  @Override
  public void setHoodVolts(double volts){
    hood.setVoltage(volts);
  }

  @Override  
  public void updateInputs(ShooterIOInputs inputs){
    leaderSignals.update(inputs.leader, leader);
    hoodSignals.update(inputs.hood, hood);
    inputs.actualHoodPosition = Units.rotationsToRadians(hood.getPosition().getValueAsDouble()) * Constants.TurretShooterConstants.Hood_GEAR_RATIO;
    inputs.actualLeaderVelocityRadPerSec = leader.getVelocity().getValueAsDouble() * 2 * Math.PI;
  }


}
