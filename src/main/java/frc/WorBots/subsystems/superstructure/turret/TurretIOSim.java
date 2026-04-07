package frc.WorBots.subsystems.superstructure.turret;

import frc.WorBots.Constants;

public class TurretIOSim implements TurretIO {
  public double position = 0.0;
  public double velocity = 0.0;

  public void setVoltage(double volts) {
    velocity = volts * 0.25;
  }

  public void updateInputs(TurretIOInputs inputs) {
    position = position + velocity * Constants.RobotConstants.ROBOT_PERIOD;
    inputs.turretFusedAngle = position * Constants.TurretShooterConstants.TURRET_GEAR_RATIO;
    inputs.turret.isConnected = true;
    inputs.turretVelocity = 0.0;
  }

  public void resetOffset() {
    position = 0.0;
  }
}
