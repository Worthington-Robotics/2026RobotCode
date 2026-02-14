package frc.WorBots.subsystems.shooter;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.WorBots.CanIDs;
import frc.WorBots.Constants;
import frc.WorBots.util.HardwareUtils;
import frc.WorBots.util.HardwareUtils.TalonSignalsPositional;

public class ShooterIOTalon implements ShooterIO {
  private final TalonFX leader = new TalonFX(CanIDs.SuperStructure.HOOD_ID, Constants.SUPERSTRUCTURE_CANBUS);
  private final TalonFX follower = new TalonFX(CanIDs.SuperStructure.FOLLOWER_ID, Constants.SUPERSTRUCTURE_CANBUS);
  private final TalonFX hood = new TalonFX(CanIDs.SuperStructure.HOOD_ID, Constants.SUPERSTRUCTURE_CANBUS);
  
  //exclude linear filter for now
  private final TalonSignalsPositional leaderSignals;
  private final TalonSignalsPositional hoodSignals;

  public ShooterIOTalon(){
    leader.setNeutralMode(NeutralModeValue.Brake);
    hood.setNeutralMode(NeutralModeValue.Brake);
    
    //Make sure that they actually are aligned 
    follower.setControl(new Follower(CanIDs.SuperStructure.LEADER_ID, MotorAlignmentValue.Aligned));

    leaderSignals = new TalonSignalsPositional(leader);
    hoodSignals = new TalonSignalsPositional(hood);
    //Actually set these values
    HardwareUtils.setCurrentLimit(leader, 0);
    HardwareUtils.setCurrentLimit(hood, 0);

    //Actually set this one too
    hood.setPosition(0.0);
  }
  
  public void setLeaderVolts(double volts){
    hood.setVoltage(volts);
  }

  public void setHoodVolts(double volts){
    hood.setVoltage(volts);
  }

  public void setHoodPosition(double position){
    hood.setPosition(position);

  }

  public void setHoodPosition(Rotation2d rotation){
    hood.setPosition(rotation.getRadians());
  }

  public void resetHoodPosition(){
    hood.setPosition(0.0);
  }

    
  public void updateInputs(ShooterIOInputs inputs){
    leaderSignals.update(inputs.leader, leader);
    hoodSignals.update(inputs.hood, hood);

    inputs.actualHoodPosition = hood.getPosition().getValueAsDouble();
    inputs.actualLeaderVelocityRadPerSec = leader.getVelocity().getValueAsDouble();
  }


}
