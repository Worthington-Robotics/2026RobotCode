package frc.WorBots.subsystems.superstructure.turret;

import java.util.Optional;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import frc.WorBots.CanIDs;
import frc.WorBots.util.HardwareUtils;
import frc.WorBots.util.HardwareUtils.TalonSignalsPositional;
import frc.WorBots.Constants;

public class TurretIOTalon implements TurretIO {
  // electronics
  private TalonFX turretMotor;
  private CANcoder turretAbsEncoder;
  private TurretIOInputs inputs;

  private final StatusSignal<Angle> turretAbsEncoderSignal;
  private final StatusSignal<Angle> turretMotorPositionSignal;

  private final StatusSignal<Current> motorCurrentSignal;
  private final TalonSignalsPositional motorSignal;

  private double fusEncoderOffset;
  private boolean shouldSetEncoderOffset = true;

  public TurretIOTalon() {

    inputs = new TurretIOInputs();
    turretMotor = new TalonFX(CanIDs.SuperStructure.TURRET_ID, CanIDs.SuperStructure.CAN_BUS);

    turretAbsEncoder = new CANcoder(CanIDs.SuperStructure.TURRET_ABS_ENCODER_ID, CanIDs.SuperStructure.CAN_BUS);

    HardwareUtils.setCurrentLimit(turretMotor, Constants.TurretShooterConstants.TURRET_CURRENT_LIMIT);

    turretAbsEncoderSignal = turretAbsEncoder.getAbsolutePosition();
    turretMotorPositionSignal = turretMotor.getPosition();
    motorCurrentSignal = turretMotor.getSupplyCurrent();

    turretAbsEncoderSignal.setUpdateFrequency(Constants.RobotConstants.ROBOT_FREQUENCY);
    turretMotorPositionSignal.setUpdateFrequency(Constants.RobotConstants.ROBOT_FREQUENCY);
    turretMotor.optimizeBusUtilization();

    motorSignal = new TalonSignalsPositional(turretMotor);

    turretMotor.setNeutralMode(NeutralModeValue.Brake);
    turretMotor.setPosition(0);
     HardwareUtils.setCurrentLimit(turretMotor, Constants.TurretShooterConstants.FLYWHEEL_CURRENT_LIMIT);
  }

  public void setVoltage(double volts) {
    turretMotor.setVoltage(volts);
  }

  // TODO figure out how we're implementing this
  // public void resetZero(TurretIOInputs inputs, double position) {
  // fusEncoderOffset += position - turretInputs.turretFusedAngle;
  // }

  public void resetOffset() {
    fusEncoderOffset = (new Rotation2d(turretAbsEncoderSignal.getValue()).getRadians() / Constants.TurretShooterConstants.TURRET_ABS_GEAR_RATIO) - Constants.TurretShooterConstants.TURRET_ABS_ENCODER_TRUE_ZERO;
  }

  public void updateInputs(TurretIOInputs inputs) {
    turretMotorPositionSignal.refresh();
    turretAbsEncoderSignal.refresh();
    turretMotorPositionSignal.refresh();
    motorCurrentSignal.refresh();
    

    final double relReading = turretMotorPositionSignal.getValue().in(edu.wpi.first.units.Units.Radians);
    inputs.turretRelAngle = relReading;

    final Optional<Double> absReading = Optional.ofNullable(turretAbsEncoderSignal.getValue())
        .map(
            reading -> {
              return MathUtil.angleModulus(
                  reading.in(edu.wpi.first.units.Units.Radians));
            });

    // TODO take a look at this
    if (absReading.isPresent()) {
      inputs.turretAbsAngle = (absReading.get() / Constants.TurretShooterConstants.TURRET_ABS_GEAR_RATIO) - Constants.TurretShooterConstants.TURRET_ABS_ENCODER_TRUE_ZERO;
    }

    if (shouldSetEncoderOffset) {
      if (absReading.isPresent()) {
        resetOffset();
        shouldSetEncoderOffset = false;
      }
    }

    inputs.turretFusedAngle = (relReading / Constants.TurretShooterConstants.TURRET_GEAR_RATIO) + fusEncoderOffset;
    // if(turretAbsEncoder.isConnected()){
    //   inputs.turretFusedAngle = inputs.turretAbsAngle;
    // }
    inputs.absEncoderConnected = turretAbsEncoder.isConnected();
    motorSignal.update(inputs.turret, turretMotor);
  }

  public TurretIOInputs getInputs() {
    return inputs;
  }
}
