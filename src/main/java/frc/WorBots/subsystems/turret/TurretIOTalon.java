package frc.WorBots.subsystems.turret;

import java.util.Optional;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import frc.WorBots.CanIDs;
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
    turretMotor = new TalonFX(CanIDs.SuperStructure.TURRET_ID);

    turretAbsEncoder = new CANcoder(CanIDs.SuperStructure.TURRET_ABS_ENCODER_ID);

    turretAbsEncoderSignal = turretAbsEncoder.getAbsolutePosition();
    turretMotorPositionSignal = turretMotor.getPosition();
    motorCurrentSignal = turretMotor.getSupplyCurrent();

    turretAbsEncoderSignal.setUpdateFrequency(Constants.ROBOT_FREQUENCY);
    turretMotorPositionSignal.setUpdateFrequency(Constants.ROBOT_FREQUENCY);
    turretMotor.optimizeBusUtilization();

    motorSignal = new TalonSignalsPositional(turretMotor);

    // TODO use hardware utils to do this
    turretMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  public void setVoltage(double volts) {
    turretMotor.setVoltage(volts);
  }

  // TODO figure out how we're implementing this
  // public void resetZero(TurretIOInputs inputs, double position) {
  // fusEncoderOffset += position - turretInputs.turretFusedAngle;
  // }

  public void resetOffset() {
    fusEncoderOffset = inputs.turretAbsAngle - inputs.turretRelAngle;
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
      inputs.turretAbsAngle = absReading.get();
    }

    if (shouldSetEncoderOffset) {
      if (absReading.isPresent()) {
        resetOffset();
        shouldSetEncoderOffset = false;

      } else if (absReading.isEmpty()) {
        fusEncoderOffset = 0.0; //Make this not reset offset instead of setting it to 0 
      }
    }

    inputs.turretFusedAngle = relReading + fusEncoderOffset;
    inputs.absEncoderConnected = turretAbsEncoder.isConnected();
  }

  public TurretIOInputs getInputs() {
    return inputs;
  }
}
