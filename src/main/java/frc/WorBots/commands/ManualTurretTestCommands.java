package frc.WorBots.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.WorBots.subsystems.superstructure.Superstructure;

/** A class containing commands used to manually test the turret */
public class ManualTurretTestCommands {

  /**
   * Commands the turrets voltage with a stick input
   * 
   * @param superstructure The superstructure containing the turret
   * @param stickVal       The value from the stick being used to command the
   *                       turret
   * @param voltageMult    The multiplier from one that applies to the voltage
   */
  public Command commandTurretWithStick(Superstructure superstructure, Supplier<Double> stickVal, double voltageMult) {
    return superstructure.runEnd(() -> {
      superstructure.setTurretVolts(stickVal.get() * voltageMult);
    }, () -> {
      superstructure.setTurretVolts(0);
    });
  }

  /**
   * Snaps turret to its left as viewed from behind the robot, will not let you go
   * right
   * 
   * @param superstructure The superstructure containing the turret
   * @param heading        The angle in radians to go to
   */
  public Command snapLeft(Superstructure superstructure, double heading) {
    return superstructure.runOnce(() -> {
      superstructure.setTurretPose(Math.abs(heading));
    });
  }

  /**
   * Sets turret goal pose to 0
   * 
   * @param superstructure The superstructure containing the turret
   */
  public Command goToZero(Superstructure superstructure) {
    return superstructure.runOnce(() -> {
      superstructure.setTurretPose(0);
    });
  }

  /**
   * Snaps turret to its right as viewed from behind the robot, will not let you
   * go left
   * 
   * @param superstructure The superstructure containing the turret
   * @param heading        The angle in radians to go to
   */
  public Command snapRight(Superstructure superstructure, double heading) {
    return superstructure.runOnce(() -> {
      superstructure.setTurretPose(-Math.abs(heading));
    });
  }

  /**
   * Commands turret pose with a stick
   * 
   * @param superstructure The superstructure containing the turret
   * @param stickVal       The value from the stick
   */
  public Command commandTurretSetpointWithStick(Superstructure superstructure, Supplier<Double> stickVal) {
    return superstructure.runOnce(() -> {
      superstructure
          .setTurretPose(superstructure.getDesiredTurretPose() + (stickVal.get() * Units.degreesToRadians(5)));
    });
  }
}
