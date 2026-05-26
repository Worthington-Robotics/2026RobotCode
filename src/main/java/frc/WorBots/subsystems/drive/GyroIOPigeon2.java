// Copyright (c) 2026 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.subsystems.drive;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import frc.WorBots.CanIDs;
import frc.WorBots.Constants;
import frc.WorBots.util.HardwareUtils.OptimalStatusSignal;
import frc.WorBots.util.OdometryThread;
import java.util.Queue;

/** IO implementation for Pigeon2 */
public class GyroIOPigeon2 implements GyroIO {
  private final Pigeon2 pigeon;

  private final OptimalStatusSignal<AngularVelocity> yawVelSignal;
  private final StatusSignal<LinearAcceleration> accelSignalX;
  private final StatusSignal<LinearAcceleration> accelSignalY;

  private final Queue<Double> yawQueue;

  public GyroIOPigeon2() {
    pigeon = new Pigeon2(CanIDs.Swerve.PIGEON_ID, CanIDs.Swerve.CAN_BUS);
    pigeon.getConfigurator().apply(new Pigeon2Configuration());
    yawVelSignal =
        new OptimalStatusSignal<>(
            pigeon.getAngularVelocityZDevice(), Constants.RobotConstants.ROBOT_PERIOD / 2.0);

    final var yawSignal = pigeon.getYaw();
    accelSignalX = pigeon.getAccelerationX();
    accelSignalY = pigeon.getAccelerationY();
    yawSignal.setUpdateFrequency(1.0 / OdometryThread.PERIOD);
    accelSignalX.setUpdateFrequency(1.0 / OdometryThread.PERIOD);
    accelSignalY.setUpdateFrequency(1.0 / OdometryThread.PERIOD);
    yawQueue = OdometryThread.getInstance().registerSignal(yawSignal);

    pigeon.optimizeBusUtilization();
    pigeon.reset();
  }

  public void updateInputs(GyroIOInputs inputs) {

    inputs.connected = yawVelSignal.isOK();

    // Update yaws
    inputs.yawPositionUpdates.clear();
    if (!yawQueue.isEmpty()) {
      while (yawQueue.size() > 0) {
        final double angle = yawQueue.poll();
        inputs.yawPositionUpdates.add(Units.degreesToRadians(angle));
      }
      inputs.yawPositionRad = inputs.yawPositionUpdates.get(inputs.yawPositionUpdates.size() - 1);
    }

    inputs.yawVelocityRadPerSec =
        yawVelSignal.getValue().in(edu.wpi.first.units.Units.RadiansPerSecond); 
  }

  @Override
  public void resetHeading(Rotation2d heading) {
    pigeon.reset();
    if (heading.getRadians() != 0.0) {
      pigeon.setYaw(heading.getDegrees());
    }
  }

}
