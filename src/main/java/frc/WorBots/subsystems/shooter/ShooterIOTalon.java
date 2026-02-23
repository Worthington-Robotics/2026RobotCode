package frc.WorBots.subsystems.shooter;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.WorBots.CanIDs;
import frc.WorBots.Constants;
import frc.WorBots.util.HardwareUtils;
import frc.WorBots.util.HardwareUtils.TalonSignalsPositional;

public class ShooterIOTalon implements ShooterIO {

  private final TalonFX leader = new TalonFX(CanIDs.SuperStructure.HOOD_ID, Constants.SUPERSTRUCTURE_CANBUS);
  private final TalonFX follower = new TalonFX(CanIDs.SuperStructure.FOLLOWER_ID, Constants.SUPERSTRUCTURE_CANBUS);
  private final TalonFX hood = new TalonFX(CanIDs.SuperStructure.HOOD_ID, Constants.SUPERSTRUCTURE_CANBUS);
  
  //Linear filter is excluded for now, pending implementation.
  
  private final TalonSignalsPositional leaderSignals;
  private final TalonSignalsPositional hoodSignals;

  public ShooterIOTalon(){
    //TODO set motor PID values
    leader.setNeutralMode(NeutralModeValue.Brake);
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

  //Are these 2 actually supposed to be in this class or in Shooter.java?

  @Override
  public void setHoodPosition(double position){
    hood.setPosition(position);

  }

  @Override
  public void setHoodPosition(Rotation2d rotation){
    hood.setPosition(rotation.getRadians());
  }

  @Override
  public void resetHoodPosition(){
    hood.setPosition(0.0);
  }


  @Override  
  public void updateInputs(ShooterIOInputs inputs){
    inputs.isConnected = true;
    leaderSignals.update(inputs.leader, leader);
    hoodSignals.update(inputs.hood, hood);

    //TODO these return position in rotations and velocity as rps
    inputs.actualHoodPosition = Units.rotationsToRadians(hood.getPosition().getValueAsDouble());
    inputs.actualLeaderVelocityRadPerSec = leader.getVelocity().getValueAsDouble() * 2 * Math.PI;
    
    inputs.hoodCurrent = hood.getSupplyCurrent().getValueAsDouble();
    inputs.leaderCurrent = leader.getSupplyCurrent().getValueAsDouble();
  }


}
