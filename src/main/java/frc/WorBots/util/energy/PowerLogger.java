package frc.WorBots.util.energy;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import frc.WorBots.Constants;

/***
 * A class to log the power usage of different subsystems and motors on the
 * robot
 */
public class PowerLogger {
  // Flags
  private final boolean LOG_INDIVIDUAL_MOTORS = true; // Determines whether the data from individual motors is logged

  private DoublePublisher batteryVoltPub;

  NetworkTable table = NetworkTableInstance.getDefault().getTable("Energy Management");

  /***
   * A class to store the data that a subsystem should send to the power logger
   * when updating power logging
   */
  public record SubsystemLog(
      String systemId,
      String[] motorIds,
      double[] motorVolts,
      double[] motorCurrents) {
  }

  /**
   * A class to store the data associated with a single motor
   */
  class MotorData {
    String subsystem;
    double energy;
    DoublePublisher powerPub;
    DoublePublisher energyPub;
    DoublePublisher currentPub;

    /**
     * A class to store the data associated with a single motor
     * 
     * @param subsystem  The subsystem the motor is part of
     * @param energy     The cumulative energy consumption of the motor
     * @param powerPub   The publisher to be used to publish motor power usage
     *                   information
     * @param energyPub  The publisher to be used to publish motor energy usage
     *                   information
     * @param currentPub The publisher to be used to publish motor current draw
     *                   information
     */
    public MotorData(String subsystem, double energy, DoublePublisher powerPub, DoublePublisher energyPub,
        DoublePublisher currentPub) {
      this.subsystem = subsystem;
      this.energy = energy;
      this.powerPub = powerPub;
      this.energyPub = energyPub;
      this.currentPub = currentPub;
    }
  }

  /***
   * A class to store the data associated with a subsystem
   */
  class SubsystemData {
    double energy;
    DoublePublisher powerPub;
    DoublePublisher energyPub;

    /**
     * A class to store the data associated with a subsystem
     * 
     * @param energy    The cumulative energy consumption of the subsystem
     * @param powerPub  The publisher to be used to publish subsystem power usage
     *                  information
     * @param energyPub The publisher to be used to publish subsystem energy usage
     *                  information
     */
    public SubsystemData(double energy, DoublePublisher powerPub, DoublePublisher energyPub) {
      this.energy = energy;
      this.powerPub = powerPub;
      this.energyPub = energyPub;
    }
  }

  // Create hash maps and lists to store all subsystem and motor data
  List<String> subsystems;
  HashMap<String, SubsystemData> subsystemDataMap;
  List<String> motors;
  HashMap<String, MotorData> motorDataMap;

  /***
   * The publisher to be used to publish motor power usage information
   * 
   * @param numberSubsystems The number of subsystems on the robot
   * @param numberMotors     The number of motors on the robot
   */
  public PowerLogger(int numberSubsystems, int numberMotors) {
    // Initializes the lists and hashmaps with a capacity equal to the amount of
    // entries they will contain
    subsystems = new ArrayList<>(numberSubsystems);
    subsystemDataMap = new HashMap<>(numberSubsystems);
    motors = new ArrayList<>(numberMotors);
    motorDataMap = new HashMap<>(numberMotors);
    // Creates a publisher to log battery voltage
    batteryVoltPub = table.getDoubleTopic("Battery Voltage").publish();
  }

  /**
   * Registers a subsystem to be logged
   * 
   * @param systemId The id of the subsystem
   * @param motorIds The ids of all motors in the subsystem
   */
  public void registerSubsystem(String systemId, String... motorIds) {
    subsystems.add(systemId);
    subsystemDataMap.put(systemId, new SubsystemData(0.0,
        table.getDoubleTopic(systemId + "/Total Power draw").publish(),
        table.getDoubleTopic(systemId + "/Total Energy draw").publish()));
    for (String motorId : motorIds) {
      motors.add(motorId);
      motorDataMap.put(motorId, new MotorData(systemId, 0.0,
          (LOG_INDIVIDUAL_MOTORS) ? table.getDoubleTopic(systemId + "/Motors/" + motorId + "/Power draw").publish()
              : null,
          (LOG_INDIVIDUAL_MOTORS) ? table.getDoubleTopic(systemId + "/Motors/" + motorId + "/Energy draw").publish()
              : null,
          (LOG_INDIVIDUAL_MOTORS) ? table.getDoubleTopic(systemId + "/Motors/" + motorId + "/Current draw").publish()
              : null));
    }
  }

  /**
   * Integrates a log into the logger. This should be run once per subsystem per
   * loop.
   * 
   * @param log The subsystem log to integrate. If the log is null it will not be
   *            integrated.
   */
  public void integrateLog(SubsystemLog log) {
    if (log == null) {
      return;
    }
    double totalSystemPowerDraw = 0.0;
    for (int i = 0; i < log.motorIds.length; i++) {
      double motorPowerDraw = log.motorCurrents[i] * Math.abs(log.motorVolts[i]);
      if (LOG_INDIVIDUAL_MOTORS) {
        MotorData previousData = motorDataMap.get(log.motorIds[i]);
        previousData.energy = previousData.energy + motorPowerDraw * Constants.RobotConstants.ROBOT_PERIOD;
        previousData.powerPub.set(motorPowerDraw);
        previousData.currentPub.set(log.motorCurrents[i]);
      }
      totalSystemPowerDraw += motorPowerDraw;
    }
    SubsystemData previousData = subsystemDataMap.get(log.systemId);
    previousData.energy = previousData.energy + totalSystemPowerDraw * Constants.RobotConstants.ROBOT_PERIOD;
    previousData.powerPub.set(totalSystemPowerDraw);
  }

  /**
   * Publish the energy and battery voltage logs. This should be run once per loop
   * after all logs are integrated.
   */
  public void publishLogs() {
    batteryVoltPub.set(RobotController.getBatteryVoltage());
    if (LOG_INDIVIDUAL_MOTORS) {
      for (String i : motors) {
        motorDataMap.get(i).energyPub.set(motorDataMap.get(i).energy);
      }
    }
    for (String i : subsystems) {
      subsystemDataMap.get(i).energyPub.set(subsystemDataMap.get(i).energy);
    }
  }
}
