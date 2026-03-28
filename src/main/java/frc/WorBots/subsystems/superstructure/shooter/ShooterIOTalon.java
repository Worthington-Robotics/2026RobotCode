package frc.WorBots.subsystems.superstructure.shooter;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.WorBots.CanIDs;
import frc.WorBots.Constants;
import frc.WorBots.util.HardwareUtils;
import frc.WorBots.util.HardwareUtils.TalonSignals;
import frc.WorBots.util.HardwareUtils.TalonSignalsPositional;

public class ShooterIOTalon implements ShooterIO {

  private final TalonFX leader = new TalonFX(CanIDs.SuperStructure.LEADER_ID, CanIDs.SuperStructure.CAN_BUS);
  private final TalonFX follower = new TalonFX(CanIDs.SuperStructure.FOLLOWER_ID, CanIDs.SuperStructure.CAN_BUS);
  private final TalonFX hood = new TalonFX(CanIDs.SuperStructure.HOOD_ID, CanIDs.SuperStructure.CAN_BUS);
  
  //Linear filter is excluded for now, pending implementation.
  
  private final TalonSignals leaderSignals;
  private final TalonSignals followerSignals;
  private final TalonSignalsPositional hoodSignals;

  private final double kv = 0.020;

  public ShooterIOTalon(){
    //TODO set motor PID values
    leader.setNeutralMode(NeutralModeValue.Coast);
    follower.setNeutralMode(NeutralModeValue.Coast);
    hood.setNeutralMode(NeutralModeValue.Brake);


    HardwareUtils.setInverted(leader, true);
    //TODO: Make sure that they actually are aligned 
    follower.setControl(new Follower(CanIDs.SuperStructure.LEADER_ID, MotorAlignmentValue.Opposed));

    leaderSignals = new TalonSignals(leader);
    followerSignals = new TalonSignals(follower);
    hoodSignals = new TalonSignalsPositional(hood);

    //TODO: Actually set these values
    HardwareUtils.setCurrentLimit(leader, Constants.TurretShooterConstants.FLYWHEEL_CURRENT_LIMIT);
    HardwareUtils.setCurrentLimit(follower, Constants.TurretShooterConstants.FLYWHEEL_CURRENT_LIMIT);
    // HardwareUtils.setCurrentLimit(hood, Constants.TurretShooterConstants.HOOD_CURRENT_LIMIT);

    hood.setPosition(0.0);
    leader.setPosition(0);
    follower.setPosition(0);

    var slot0Configs = new Slot0Configs();
    slot0Configs.kS = 0.021;
    slot0Configs.kV = 0.0; //3.5
    slot0Configs.kP = 0.06 * 2 * Math.PI;
    slot0Configs.kI = 0.0;
    slot0Configs.kD = 0.0;
    leader.getConfigurator().apply(slot0Configs);
    follower.getConfigurator().apply(slot0Configs);
  }
  
  @Override
  public void setLeaderVolts(double volts){
    leader.setVoltage(volts);
  }

  @Override
  public void setHoodVolts(double volts){
    SmartDashboard.putNumber("Voltage", volts);
    volts = MathUtil.clamp(volts, -1, 1);
    hood.setVoltage(volts);
  }

  @Override
  public void setFlywheelVelocity(double velocity){
    final VelocityVoltage request = new VelocityVoltage(velocity / (2 * Math.PI)).withSlot(0).withFeedForward(velocity * kv);
    leader.setControl(request);
  }

  @Override  
  public void updateInputs(ShooterIOInputs inputs){
    leaderSignals.update(inputs.leader, leader);
    followerSignals.update(inputs.follower, follower);
    hoodSignals.update(inputs.hood, hood);
    inputs.actualHoodPosition = Units.rotationsToRadians(hood.getPosition().getValueAsDouble()) / Constants.TurretShooterConstants.Hood_GEAR_RATIO;
    inputs.actualLeaderVelocityRadPerSec = leader.getVelocity().getValueAsDouble() * 2 * Math.PI;
  }


}
