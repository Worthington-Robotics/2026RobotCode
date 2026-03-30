package frc.WorBots.energy;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.WorBots.Constants;

public class PowerLogger {
  // Flags
  private final boolean LOG_INDIVIDUAL_MOTORS = true;

  NetworkTable table = NetworkTableInstance.getDefault().getTable("Energy Management");

  public record SubsystemLog(
      String systemId,
      String[] motorIds,
      double[] motorVolts,
      double[] motorCurrents) {
  }

  class MotorData{
      String subsystem;
      double energy;
      DoublePublisher powerPub;
      DoublePublisher energyPub;
      public MotorData(String subsystem, double energy, DoublePublisher powerPub, DoublePublisher energyPub){
        this.subsystem = subsystem;
        this.energy = energy;
        this.powerPub = powerPub;
        this.energyPub = energyPub;
      }
    }

  class SubsystemData{
      double energy;
      DoublePublisher powerPub;
      DoublePublisher energyPub;
      public SubsystemData(double energy, DoublePublisher powerPub, DoublePublisher energyPub){
        this.energy = energy;
        this.powerPub = powerPub;
        this.energyPub = energyPub;
      }
    }

  List<String> subsystems;
  HashMap<String, SubsystemData> subsystemDataMap;
  List<String> motors;
  HashMap<String, MotorData> motorDataMap;

  public PowerLogger(int numberSubsystems, int numberMotors) {
    subsystems = new ArrayList<>(numberSubsystems);
    subsystemDataMap = new HashMap<>(numberSubsystems);
    motors = new ArrayList<>(numberMotors);
    motorDataMap = new HashMap<>(numberMotors);
  }

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
              : null));
    }
  }

  public void integrateLog(SubsystemLog log) {
    if(log == null){
      return;
    }
    double totalSystemPowerDraw = 0.0;
    for (int i = 0; i < log.motorIds.length; i++) {
      double motorPowerDraw = log.motorCurrents[i] * log.motorVolts[i];
      if (LOG_INDIVIDUAL_MOTORS) {
        MotorData previousData = motorDataMap.get(log.motorIds[i]);
        previousData.energy = previousData.energy  + motorPowerDraw * Constants.RobotConstants.ROBOT_PERIOD;
        previousData.powerPub.set(motorPowerDraw);
      }
      totalSystemPowerDraw += motorPowerDraw;
    }
    SubsystemData previousData = subsystemDataMap.get(log.systemId);
    previousData.energy = previousData.energy + totalSystemPowerDraw * Constants.RobotConstants.ROBOT_PERIOD;
    previousData.powerPub.set(totalSystemPowerDraw);
  }

  public void publishLogs() {
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
