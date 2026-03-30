package frc.WorBots.subsystems.intake;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Current;
import frc.WorBots.util.HardwareUtils.OptimalStatusSignal;
import frc.WorBots.util.HardwareUtils.TalonSignalsPositional;
import frc.WorBots.util.HardwareUtils;
import frc.WorBots.Constants;
import frc.WorBots.energy.PowerLogger.SubsystemLog;
import frc.WorBots.CanIDs;

public class IntakeIOTalon implements IntakeIO {
  private final TalonFX intakeMotor;
  private final TalonFX extendingMotor;
  //Set up request to command motor position
  final PositionVoltage request = new PositionVoltage(0).withSlot(0);

  private final TalonSignalsPositional intakeMotorSignals;
  private final TalonSignalsPositional extendingMotorSignals;

  private final StatusSignal<Current> intakeCurrentDrawSignal;
  private final OptimalStatusSignal<Current> extendingCurrentDrawSignal;

  public IntakeIOTalon() {

    // Instantiating the TalonFXs.
    intakeMotor = new TalonFX(CanIDs.SuperStructure.INTAKE_MOTOR_ID, CanIDs.SuperStructure.CAN_BUS);
    extendingMotor = new TalonFX(CanIDs.SuperStructure.EXTENDING_MOTOR_ID, CanIDs.SuperStructure.CAN_BUS);
    extendingMotor.setPosition(0);
    intakeMotor.setPosition(0);

    // TODO Actually find out whether or not to invert the two motors

    /*
     * Setting the neutral modes and inversion states of both the
     * extending and intaking motors.
     */
    intakeMotor.setNeutralMode(NeutralModeValue.Coast);
    HardwareUtils.setInverted(extendingMotor, true);
    extendingMotor.setNeutralMode(NeutralModeValue.Brake);
    HardwareUtils.setInverted(intakeMotor, false);

    // Sets Talon signals for intaking and extending motors.
    intakeMotorSignals = new TalonSignalsPositional(intakeMotor);
    extendingMotorSignals = new TalonSignalsPositional(extendingMotor);

    // Sets the current draw signal and current limits for intaking and extending
    // motors.
    intakeCurrentDrawSignal = intakeMotor.getStatorCurrent();
    intakeCurrentDrawSignal.setUpdateFrequency(Constants.RobotConstants.ROBOT_FREQUENCY);
    extendingCurrentDrawSignal = new OptimalStatusSignal<>(extendingMotor.getStatorCurrent(), Constants.RobotConstants.ROBOT_PERIOD);

    HardwareUtils.setCurrentLimit(intakeMotor, Constants.IntakeConstants.INTAKE_CURRENT_LIMIT);
    HardwareUtils.setCurrentLimit(extendingMotor, Constants.IntakeConstants.EXTENDER_CURRENT_LIMIT);

    intakeMotor.optimizeBusUtilization();
    extendingMotor.optimizeBusUtilization();

    extendingMotor.setPosition(0);

  }

  public void updateInputs(IntakeIOInputs inputs) {
    intakeMotorSignals.update(inputs.intakeMotor, intakeMotor);
    extendingMotorSignals.update(inputs.extendingMotor, extendingMotor);

    // Redefine input position
    inputs.extendPosition = Units.degreesToRadians(90) + (inputs.extendingMotor.positionRads / Constants.IntakeConstants.INTAKE_PIVOT_GR);
    inputs.intakeCurrent = intakeCurrentDrawSignal.refresh().getValue().in(edu.wpi.first.units.Units.Amps);
    inputs.extendingCurrent = extendingCurrentDrawSignal.getValue().in(edu.wpi.first.units.Units.Amps);

  }

  // TODO Set actual max voltage

  public void setIntakeMotorVolts(double volts) {
    intakeMotor.setVoltage(volts);
  }

  public void setExtendingMotorVolts(double volts) {
    extendingMotor.setVoltage(volts);
  }
}
