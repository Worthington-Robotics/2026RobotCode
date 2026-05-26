// Copyright (c) 2026 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.subsystems.superstructure.turret;

import java.util.Optional;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import frc.WorBots.CanIDs;
import frc.WorBots.util.HardwareUtils;
import frc.WorBots.util.HardwareUtils.TalonSignalsPositional;
import frc.WorBots.Constants;

public class TurretIOTalon implements TurretIO {
  // Initialize motors
  private TalonFX turretMotor;
  private CANcoder turretAbsEncoder;
  private TurretIOInputs inputs;

  private final StatusSignal<Angle> turretAbsEncoderSignal;
  private final StatusSignal<Angle> turretMotorPositionSignal;

  private final StatusSignal<AngularVelocity> turretVelocitySignal;

  private final StatusSignal<Current> motorCurrentSignal;
  private final TalonSignalsPositional motorSignal;

  private double fusEncoderOffset;
  private boolean shouldSetEncoderOffset = true;

  final TrapezoidProfile profile = new TrapezoidProfile(
      new TrapezoidProfile.Constraints((20 * Math.PI), (20 * Math.PI)));
  TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

  private final double kv = 0.2;

  public TurretIOTalon() {
    inputs = new TurretIOInputs();
    turretMotor = new TalonFX(CanIDs.SuperStructure.TURRET_ID, CanIDs.SuperStructure.CAN_BUS);

    turretAbsEncoder = new CANcoder(CanIDs.SuperStructure.TURRET_ABS_ENCODER_ID, CanIDs.SuperStructure.CAN_BUS);

    HardwareUtils.setCurrentLimit(turretMotor, Constants.TurretShooterConstants.TURRET_CURRENT_LIMIT);

    turretAbsEncoderSignal = turretAbsEncoder.getAbsolutePosition();
    turretMotorPositionSignal = turretMotor.getPosition();
    motorCurrentSignal = turretMotor.getSupplyCurrent();
    turretVelocitySignal = turretMotor.getVelocity();

    turretAbsEncoderSignal.setUpdateFrequency(Constants.RobotConstants.ROBOT_FREQUENCY);
    turretMotorPositionSignal.setUpdateFrequency(Constants.RobotConstants.ROBOT_FREQUENCY);
    turretVelocitySignal.setUpdateFrequency(Constants.RobotConstants.ROBOT_FREQUENCY);
    turretMotor.optimizeBusUtilization();

    motorSignal = new TalonSignalsPositional(turretMotor);

    turretMotor.setNeutralMode(NeutralModeValue.Brake);
    turretMotor.setPosition(0);

    HardwareUtils.setMotorPidSlot0(turretMotor, 9.5, 0.0, 0.1, 0.29, 0.0, 0.0);

    updateInputs(inputs);
    // Set rotation limits
    var limitConfigs = new SoftwareLimitSwitchConfigs();
    limitConfigs.ForwardSoftLimitThreshold = fusedToRelMotor(Constants.TurretShooterConstants.TURRET_MAX_ANGLE);
    limitConfigs.ReverseSoftLimitThreshold = fusedToRelMotor(Constants.TurretShooterConstants.TURRET_MIN_ANGLE);

    limitConfigs.ForwardSoftLimitEnable = true;
    limitConfigs.ReverseSoftLimitEnable = true;

    turretMotor.getConfigurator().apply(limitConfigs);

    // Set voltage limits
    HardwareUtils.setMotorVoltageLimits(turretMotor, Constants.TurretShooterConstants.TURRET_MAX_VOLTAGE);
  }

  public void setVoltage(double volts) {
    turretMotor.setVoltage(volts);
  }

  @Override
  public void setPosition(TrapezoidProfile.State goalState) {
    setpoint = goalState;
    PositionVoltage request = new PositionVoltage(0).withSlot(0).withFeedForward(setpoint.velocity * kv);
    request.Position = fusedToRelMotor(setpoint.position);
    request.Velocity = fusedToRelMotor(setpoint.velocity);
    turretMotor.setControl(request);
  }

  public void resetOffset() {
    fusEncoderOffset = (new Rotation2d(turretAbsEncoderSignal.getValue()).getRadians()
        / Constants.TurretShooterConstants.TURRET_ABS_GEAR_RATIO)
        - Constants.TurretShooterConstants.TURRET_ABS_ENCODER_TRUE_ZERO;
  }

  public void updateInputs(TurretIOInputs inputs) {
    turretMotorPositionSignal.refresh();
    turretAbsEncoderSignal.refresh();
    turretMotorPositionSignal.refresh();
    motorCurrentSignal.refresh();
    turretVelocitySignal.refresh();

    final double relReading = turretMotorPositionSignal.getValue().in(edu.wpi.first.units.Units.Radians);
    inputs.turretRelAngle = relReading;

    final Optional<Double> absReading = Optional.ofNullable(turretAbsEncoderSignal.getValue())
        .map(
            reading -> {
              return MathUtil.angleModulus(
                  reading.in(edu.wpi.first.units.Units.Radians));
            });

    if (absReading.isPresent()) {
      inputs.turretAbsAngle = (absReading.get() / Constants.TurretShooterConstants.TURRET_ABS_GEAR_RATIO)
          - Constants.TurretShooterConstants.TURRET_ABS_ENCODER_TRUE_ZERO;
    }

    if (shouldSetEncoderOffset) {
      if (absReading.isPresent()) {
        resetOffset();
        shouldSetEncoderOffset = false;
      }
    }

    inputs.turretFusedAngle = (relReading / Constants.TurretShooterConstants.TURRET_GEAR_RATIO) + fusEncoderOffset;
    inputs.absEncoderConnected = turretAbsEncoder.isConnected();
    inputs.turretVelocity = turretVelocitySignal.getValueAsDouble() * 2 * Math.PI;
    motorSignal.update(inputs.turret, turretMotor);
  }

  public TurretIOInputs getInputs() {
    return inputs;
  }

  /**
   * Converts a fused angle in radians to an angle in rotations that can be given
   * to the turret motor
   */
  public double fusedToRelMotor(double angle) {
    return Units.radiansToRotations((angle - fusEncoderOffset) * Constants.TurretShooterConstants.TURRET_GEAR_RATIO);
  }
}
