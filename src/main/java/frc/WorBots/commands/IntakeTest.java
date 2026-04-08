package frc.WorBots.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.WorBots.subsystems.intake.Intake;
import frc.WorBots.util.debug.TunableDouble;

/** Contains commands to test the intake */
public class IntakeTest {
  // A tunable double to be used to adjust how much voltage the intake is run at
  public TunableDouble voltage = new TunableDouble("Tuning", "Intake", "Voltage Mult");

  /**
   * Controls the extend motor using stick input
   * 
   * @param intake The intake to control
   * @param volts  The supplier providing the voltage to run the motor at
   */
  public Command extendWithVoltStick(Intake intake, Supplier<Double> volts) {
    return intake.runOnce(() -> {
      intake.setVoltsExtending(volts.get() * 4);
    });
  }

  /**
   * Runs the extend motor at a constant voltage
   * 
   * @param intake The intake to run
   * @param volts  The voltage to run the motor at
   */
  public Command extendWithVolt(Intake intake, double volts) {
    return intake.runOnce(() -> {
      intake.setVoltsExtending(volts);
    });
  }

  /**
   * Commands the intake motor using stick input
   * 
   * @param intake The intake to control
   * @param volts  The supplier providing the voltage to run the motor at
   * @return
   */
  public Command intakeWithVoltStick(Intake intake, Supplier<Double> volts) {
    return intake.runOnce(() -> {
      intake.setVoltsIntake(volts.get() * voltage.get());
    });
  }

  /**
   * Runs the intake motor at a constant voltage
   * 
   * @param intake The intake to command
   * @param volts  The voltage to run the intake at
   */
  public Command intakeWithVolt(Intake intake, double volts) {
    return intake.runOnce(() -> {
      intake.setVoltsIntake(volts);
    });
  }

}
