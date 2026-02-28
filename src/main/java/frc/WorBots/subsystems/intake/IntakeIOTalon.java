package frc.WorBots.subsystems.intake;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Current;
import frc.WorBots.util.HardwareUtils.OptimalStatusSignal;
import frc.WorBots.util.HardwareUtils.TalonSignalsPositional;
import frc.WorBots.util.HardwareUtils;
import frc.WorBots.Constants;
import frc.WorBots.CanIDs;

public class IntakeIOTalon implements IntakeIO {
  private final TalonFX intakeMotor;
  private final TalonFX extendingMotor;
  //Set up request to command motor position
  final PositionVoltage request = new PositionVoltage(0).withSlot(0);

  private final TalonSignalsPositional intakeMotorSignals;
  private final TalonSignalsPositional extendingMotorSignals;

  private final OptimalStatusSignal<Current> intakeCurrentDrawSignal;
  private final OptimalStatusSignal<Current> extendingCurrentDrawSignal;

  public IntakeIOTalon() {

    // Instantiating the TalonFXs.
    intakeMotor = new TalonFX(CanIDs.SuperStructure.INTAKE_MOTOR_ID, CanIDs.SuperStructure.CAN_BUS);
    extendingMotor = new TalonFX(CanIDs.SuperStructure.EXTENDING_MOTOR_ID, CanIDs.SuperStructure.CAN_BUS);

    // TODO Actually find out whether or not to invert the two motors

    /*
     * Setting the neutral modes and inversion states of both the
     * extending and intaking motors.
     */
    intakeMotor.setNeutralMode(NeutralModeValue.Brake);
    HardwareUtils.setInverted(extendingMotor, false);
    extendingMotor.setNeutralMode(NeutralModeValue.Brake);
    HardwareUtils.setInverted(intakeMotor, false);

    // Sets Talon signals for intaking and extending motors.
    intakeMotorSignals = new TalonSignalsPositional(intakeMotor);
    extendingMotorSignals = new TalonSignalsPositional(extendingMotor);

    // Sets the current draw signal and current limits for intaking and extending
    // motors.
    intakeCurrentDrawSignal = new OptimalStatusSignal<>(intakeMotor.getStatorCurrent(), Constants.ROBOT_PERIOD);
    extendingCurrentDrawSignal = new OptimalStatusSignal<>(extendingMotor.getStatorCurrent(), Constants.ROBOT_PERIOD);

    HardwareUtils.setCurrentLimit(intakeMotor, 160);
    HardwareUtils.setCurrentLimit(extendingMotor, 160);

    //Set up the extending motor PID
    var slot0Configs = new Slot0Configs();
    slot0Configs.kP = Constants.INTAKE_EXTENDING_KP;
    slot0Configs.kI = 0.0;
    slot0Configs.kD = Constants.INTAKE_EXTENDING_KD;

    extendingMotor.getConfigurator().apply(slot0Configs);

    intakeMotor.optimizeBusUtilization();
    extendingMotor.optimizeBusUtilization();

  }

  public void updateInputs(IntakeIOInputs inputs) {
    intakeMotorSignals.update(inputs.intakeMotor, intakeMotor);
    extendingMotorSignals.update(inputs.extendingMotor, extendingMotor);

    inputs.intakeCurrent = intakeCurrentDrawSignal.getValue().in(edu.wpi.first.units.Units.Amps);
    inputs.extendingCurrent = extendingCurrentDrawSignal.getValue().in(edu.wpi.first.units.Units.Amps);

  }

  // TODO Set actual max voltage

  public void setIntakeVolts(double volts) {
    intakeMotorSignals.setVoltage(intakeMotor, volts, 10);
  }

  public void setExtendingMotorVolts(double volts) {
    extendingMotorSignals.setVoltage(extendingMotor, volts, 10);
  }

  public void setPosition(IntakePoses pose){
    extendingMotor.setControl(request.withPosition(pose.get()));
  }

}
