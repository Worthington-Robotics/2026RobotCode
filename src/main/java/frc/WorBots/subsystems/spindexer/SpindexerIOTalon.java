// Copyright (c) 2026 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.subsystems.spindexer;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import frc.WorBots.CanIDs;
import frc.WorBots.Constants;
import frc.WorBots.util.HardwareUtils;
import frc.WorBots.util.HardwareUtils.TalonSignals;
import frc.WorBots.util.HardwareUtils.TalonSignalsPositional;

public class SpindexerIOTalon implements SpindexerIO{

  private TalonFX talon = new TalonFX(CanIDs.SuperStructure.SPINDEXER_ID, CanIDs.SuperStructure.CAN_BUS);
  private TalonFX kicker = new TalonFX(CanIDs.SuperStructure.KICKER_ID, CanIDs.SuperStructure.CAN_BUS);

  private TalonSignalsPositional spinSignal = new TalonSignalsPositional(talon);
  private TalonSignals kickerSignal = new TalonSignals(kicker);

  private double spinVoltage = 0.0;
  private double kickerVoltage = 0.0;

  public SpindexerIOTalon(){
    talon.setNeutralMode(NeutralModeValue.Coast);
    kicker.setNeutralMode(NeutralModeValue.Coast);
    kicker.setPosition(0);
    talon.setPosition(0);

    HardwareUtils.setCurrentLimit(kicker, Constants.SpindexerConstants.KICKER_CURRENT_LIMIT);
    HardwareUtils.setCurrentLimit(talon, Constants.SpindexerConstants.SPINDEXER_CURRENT_LIMIT);
  }

  @Override
  public void updateInputs(SpindexerIOInputs inputs){
    inputs.active = isActive();
    inputs.jammed = isJammed();
    //Modifies the talons output velocity to be in radians and be the spindexers velocity
    inputs.spinVelocity = talon.getVelocity().getValueAsDouble() * 2 * Math.PI * Constants.SpindexerConstants.SPINDEXER_GEAR_RATIO;
    inputs.kickerVelocity = kicker.getVelocity().getValueAsDouble() * 2 * Math.PI * Constants.SpindexerConstants.KICKER_GEAR_RATIO;
    spinSignal.update(inputs.talon, talon);
    kickerSignal.update(inputs.follower, kicker);

  }

  @Override
  public void setSpinVoltage(double volts){
    volts = MathUtil.clamp(volts, -10, 10);
    this.spinVoltage = volts;
    talon.setVoltage(spinVoltage);
  }

  @Override
  public void setKickerVoltage(double volts){
    volts = MathUtil.clamp(volts, -10, 10);
    this.kickerVoltage = volts;
    kicker.setVoltage(kickerVoltage);
  }

  @Override
  public void stop(){
    setSpinVoltage(0);
    setKickerVoltage(0);
  }

  private boolean isActive(){
    //Checks if the spindexer is rotating more than 15 degrees per second
    //talon.getVelocity returns its value in rotations, the 2Pi is there to convert to radians
    if(talon.getVelocity().getValueAsDouble() * 2 * Math.PI > Units.degreesToRadians(15)){
      return true;
    }
      return false;
  }

  private boolean isJammed(){
    return talon.getTorqueCurrent().getValueAsDouble() > Constants.SpindexerConstants.SPINDEXER_STALL_CURRENT && 
      talon.getVelocity().getValueAsDouble() < Constants.SpindexerConstants.SPINDEXER_STALL_SPEED;
  }
}
