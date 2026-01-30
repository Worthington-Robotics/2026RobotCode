// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.util;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
// import com.playingwithfusion.TimeOfFlight;
import edu.wpi.first.hal.PowerDistributionFaults;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.WorBots.Constants;
import frc.WorBots.util.cache.CountingCache;
import frc.WorBots.util.debug.DebugValue;
import frc.WorBots.util.debug.DebugValue.DebugBool;
import frc.WorBots.util.debug.DebugValue.DebugDouble;
import frc.WorBots.util.math.GeneralMath;

/** Utility functions for working with hardware devices */
public class HardwareUtils {
  // Constants
  /** The maximum temperature in celsius that we want to run our motors at */
  public static final double MAX_MOTOR_TEMP = 80.0;

  public static final double IDEAL_BATTERY_VOLTAGE = 12.3;

  /**
   * Checks if a time of flight device is connected
   *
   * @param tof The time of flight
   * @return True if the device is working properly, false if it is not
   */
  // public static boolean getTimeOfFlightStatus(TimeOfFlight tof) {
  //   final TimeOfFlight.Status status = tof.getStatus();
  //   return status == TimeOfFlight.Status.Valid;
  // }

  /**
   * Safely sets a motor voltage with voltage limiting and temperature checks
   *
   * @param talon The motor
   * @param volts The voltage to set
   * @param maxVolts The maximum magnitude for the voltage
   * @param temp The temperature of the motor in celsius
   */
  public static void setTalonVoltage(TalonFX talon, double volts, double maxVolts, double temp) {
    if (temp < MAX_MOTOR_TEMP) {
      volts = GeneralMath.clampMagnitude(volts, maxVolts);
      talon.setVoltage(volts);
    } else {
      talon.stopMotor();
    }
  }

  /**
   * Prepares a motor to be set as a leader
   *
   * @param talon The motor to prepare as a leader
   */
  public static void prepareLeader(TalonFX talon) {
    talon.getTorqueCurrent().setUpdateFrequency(50);
    talon.getDutyCycle().setUpdateFrequency(50);
  }

  /** Base inputs for a TalonFX */
  public static class TalonInputs {
    private final DebugDouble voltsPub;
    private final DebugDouble supplyVoltsPub;
    private final DebugBool connectedPub;

    public double appliedPowerVolts = 0.0;
    public double supplyVoltage = 0.0;
    public double currentDrawAmps = 0.0;
    public double temperatureCelsius = 0.0;
    public boolean isConnected = false;

    public TalonInputs(String table, String subtable) {
      voltsPub = DebugValue.compDouble(table, subtable + "/Applied Voltage");
      supplyVoltsPub = DebugValue.compDouble(table, subtable + "/Supply Voltage");
      connectedPub = DebugValue.compBool(table, subtable + "/Connected");
    }

    public void publish() {
      voltsPub.set(appliedPowerVolts);
      supplyVoltsPub.set(supplyVoltage);
      connectedPub.set(isConnected);
    }
  }

  /** Inputs for a TalonFX with position and velocity readings */
  public static class TalonInputsPositional extends TalonInputs {
    private final DebugDouble posPub;
    private final DebugDouble velPub;

    public double positionRads = 0.0;
    public double velocityRadsPerSec = 0.0;

    public TalonInputsPositional(String table, String subtable) {
      super(table, subtable);
      posPub = DebugValue.compDouble(table, subtable + "/Position");
      velPub = DebugValue.compDouble(table, subtable + "/Velocity");
    }

    public void publish() {
      super.publish();
      posPub.set(positionRads);
      velPub.set(velocityRadsPerSec);
    }
  }

  /** Base status signals for a TalonFX */
  public static class TalonSignals {
    private final OptimalStatusSignal<Voltage> voltsSignal;

    public TalonSignals(TalonFX motor) {
      voltsSignal = new OptimalStatusSignal<>(motor.getSupplyVoltage(), 15);

      // For .get() calls we need the duty cycle
      motor.getDutyCycle().setUpdateFrequency(15);
    }

    protected void refresh() {}

    public void update(TalonInputs inputs, TalonFX motor) {
      refresh();
      final double volts = voltsSignal.getValue().in(edu.wpi.first.units.Units.Volts);
      inputs.appliedPowerVolts = volts * motor.get();
      inputs.supplyVoltage = volts;
      inputs.isConnected = voltsSignal.isOK && inputs.temperatureCelsius < MAX_MOTOR_TEMP;
    }

    /**
     * Safely sets a motor voltage with voltage limiting and temperature checks
     *
     * @param talon The motor
     * @param volts The voltage to set
     * @param maxVolts The maximum magnitude for the voltage
     */
    public void setVoltage(TalonFX talon, double volts, double maxVolts) {
      HardwareUtils.setTalonVoltage(talon, volts, maxVolts, 0.0);
    }
  }

  /** Status signals for a TalonFX with positional readings */
  public static class TalonSignalsPositional extends TalonSignals {
    private final StatusSignal<Angle> posSignal;
    private final StatusSignal<AngularVelocity> velSignal;

    public TalonSignalsPositional(TalonFX motor) {
      super(motor);
      posSignal = motor.getPosition();
      velSignal = motor.getVelocity();
      posSignal.setUpdateFrequency(Constants.ROBOT_PERIOD);
      velSignal.setUpdateFrequency(Constants.ROBOT_PERIOD);
    }

    @Override
    protected void refresh() {
      StatusSignal.refreshAll(posSignal, velSignal);
    }

    public void update(TalonInputsPositional inputs, TalonFX motor) {
      super.update(inputs, motor);
      this.refresh();
      inputs.positionRads = posSignal.getValue().in(edu.wpi.first.units.Units.Radians);
      inputs.velocityRadsPerSec =
          velSignal.getValue().in(edu.wpi.first.units.Units.RadiansPerSecond);
    }
  }

  public static class OptimalStatusSignal<T> {
    private final StatusSignal<T> signal;
    private final CountingCache<T> cache;
    private boolean isOK = false;

    public OptimalStatusSignal(StatusSignal<T> signal, double frequency) {
      signal.setUpdateFrequency(frequency);
      this.signal = signal;
      // Calculate the count based on the ratio between the signal frequency and robot
      // period
      int count = (int) Math.floor(Constants.ROBOT_PERIOD / frequency);
      // We don't want to limit periods that are only half as that will just introduce
      // too much latency
      if (count <= 2) {
        count = 0;
      }
      this.cache =
          new CountingCache<>(
              () -> {
                this.isOK = StatusSignal.refreshAll(this.signal).isOK();
                return this.signal.getValue();
              },
              count);
    }

    public StatusSignal<T> getSignal() {
      return this.signal;
    }

    public T getValue() {
      return this.cache.get();
    }

    public boolean isOK() {
      return this.isOK;
    }
  }

  /**
   * Sets a motor to inverted or not
   *
   * @param motor The motor to set
   * @param inverted Whether or not the motor is inverted
   */
  public static StatusCode setInverted(TalonFX motor, boolean inverted) {
    /* First read the configs so they're up-to-date */
    var configs = new MotorOutputConfigs();
    StatusCode retval = motor.getConfigurator().refresh(configs);
    if (retval.isOK()) {
      /* Then set the invert config to the appropriate value */
      configs.Inverted =
          inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
      retval = motor.getConfigurator().apply(configs);
    }
    return retval;
  }

  /**
   * Sets the stator current limit for a motor, enabling it as well. Will clear any other current
   * limit configs back to their defaults.
   *
   * @param motor The motor to configure
   * @param limit The configured current limit, in amps
   */
  public static void setCurrentLimit(TalonFX motor, double limit) {
    CurrentLimitsConfigs limitsConfig = new CurrentLimitsConfigs();
    limitsConfig.StatorCurrentLimit = limit;
    limitsConfig.StatorCurrentLimitEnable = true;
    motor.getConfigurator().apply(limitsConfig);
  }

  /**
   * Checks if a PDP has any breaker faults
   *
   * @param faults The faults from the PDP
   * @return Whether the PDP has any faults
   */
  public static boolean pdpHasBreakerFault(PowerDistributionFaults faults) {
    final boolean breakerFault =
        faults.Channel0BreakerFault
            || faults.Channel1BreakerFault
            || faults.Channel2BreakerFault
            || faults.Channel3BreakerFault
            || faults.Channel4BreakerFault
            || faults.Channel5BreakerFault
            || faults.Channel6BreakerFault
            || faults.Channel7BreakerFault
            || faults.Channel8BreakerFault
            || faults.Channel9BreakerFault
            || faults.Channel10BreakerFault
            || faults.Channel11BreakerFault
            || faults.Channel12BreakerFault
            || faults.Channel13BreakerFault
            || faults.Channel14BreakerFault
            || faults.Channel15BreakerFault
            || faults.Channel16BreakerFault
            || faults.Channel17BreakerFault
            || faults.Channel18BreakerFault
            || faults.Channel19BreakerFault
            || faults.Channel20BreakerFault
            || faults.Channel21BreakerFault
            || faults.Channel22BreakerFault
            || faults.Channel23BreakerFault;

    return breakerFault;
  }

  /** State for an intake */
  public enum IntakeState {
    Idle,
    Intake,
    Spit,
  }

  private static final LinearFilter batteryVoltageFilter = LinearFilter.singlePoleIIR(1.0, 0.02);

  public static void updateBatteryVoltage() {
    batteryVoltageFilter.calculate(RobotController.getBatteryVoltage());
    SmartDashboard.putNumber("Filtered Battery Voltage", batteryVoltageFilter.lastValue());
  }

  public static double getBatteryVoltage() {
    return batteryVoltageFilter.lastValue();
  }
}
