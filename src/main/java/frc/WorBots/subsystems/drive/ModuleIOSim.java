package frc.WorBots.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.WorBots.Constants;
import frc.WorBots.util.debug.TunablePIDController;
import frc.WorBots.util.debug.TunablePIDController.TunablePIDGains;
import frc.WorBots.util.math.GeneralMath;

public class ModuleIOSim implements ModuleIO {
  private ModuleIOInputs inputs;

  private final SimpleMotorFeedforward driveFeedforward =
      new SimpleMotorFeedforward(0.116970, 0.133240);
  private static final TunablePIDGains driveFeedbackGains =
      new TunablePIDGains("Drive/Gains", "SModule Drive Feedback");
  private final TunablePIDController driveFeedback = new TunablePIDController(driveFeedbackGains);
  private static final TunablePIDGains turnFeedbackGains =
      new TunablePIDGains("Drive/Gains", "SModule Turn Feedback");
  private final TunablePIDController turnFeedback = new TunablePIDController(turnFeedbackGains);

  private final FlywheelSim driveSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60(1), 0.025, 6.75),
          DCMotor.getKrakenX60(1));
  private final FlywheelSim turnSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60(1), 0.004, 150.0 / 7.0),
          DCMotor.getKrakenX60(1));
  private final double wheelRadius = Units.inchesToMeters(2.0);

  private double turnRelativePositionRad = 0.0;
  private double turnAbsolutePositionRad = Math.random() * 2.0 * Math.PI;
  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts = 0.0;

  private final int index;

  public ModuleIOSim(int index) {
    this.inputs = new ModuleIOInputs(index);
    driveFeedbackGains.setGains(0.08, 0.0, 0.0);
    turnFeedbackGains.setGains(9.5, 0.0, 0.0);
    turnFeedback.pid.enableContinuousInput(-Math.PI, Math.PI);
    this.index = index;
  }

  public void updateInputs() {
    driveFeedback.update();
    turnFeedback.update();

    driveSim.update(Constants.ROBOT_PERIOD);
    turnSim.update(Constants.ROBOT_PERIOD);

    final double angleVelocityRadsPerSec = turnSim.getAngularVelocityRadPerSec();
    final double angleDiffRad = angleVelocityRadsPerSec * Constants.ROBOT_PERIOD;
    turnRelativePositionRad += angleDiffRad;
    turnAbsolutePositionRad += angleDiffRad;
    inputs.turnAbsoluteVelocityRadsPerSec = angleVelocityRadsPerSec;
    turnAbsolutePositionRad = MathUtil.angleModulus(turnAbsolutePositionRad);

    inputs.turnPositionErrorRad = turnFeedback.pid.getError();

    inputs.drive.positionRads += (driveSim.getAngularVelocityRadPerSec() * Constants.ROBOT_PERIOD);
    inputs.drive.velocityRadsPerSec = driveSim.getAngularVelocityRadPerSec();
    inputs.driveDistanceMeters = inputs.drive.positionRads * wheelRadius;
    inputs.driveVelocityMetersPerSec = inputs.drive.velocityRadsPerSec * wheelRadius;
    inputs.driveDistanceUpdates.clear();
    inputs.driveDistanceUpdates.add(inputs.driveDistanceMeters);
    inputs.drive.appliedPowerVolts = driveAppliedVolts;
    inputs.drive.currentDrawAmps = Math.abs(driveSim.getCurrentDrawAmps());
    inputs.drive.temperatureCelsius = 30.0;

    inputs.turnAbsolutePositionRad = turnAbsolutePositionRad;
    inputs.turn.positionRads = turnRelativePositionRad;
    inputs.turn.velocityRadsPerSec = turnSim.getAngularVelocityRadPerSec();
    inputs.turnPositionUpdates.clear();
    inputs.turnPositionUpdates.add(inputs.turnAbsolutePositionRad);
    inputs.turn.appliedPowerVolts = turnAppliedVolts;
    inputs.turn.currentDrawAmps = Math.abs(turnSim.getCurrentDrawAmps());
    inputs.turn.temperatureCelsius = 30.0;
    inputs.isConnected = true;
  }

  public ModuleIOInputs getInputs() {
    return this.inputs;
  }

  public void setDriveSpeed(double speedMetersPerSecond) {
    final double velocityRadPerSec = speedMetersPerSecond / wheelRadius;
    final double driveVolts =
        driveFeedforward.calculate(velocityRadPerSec)
            + driveFeedback.pid.calculate(inputs.drive.velocityRadsPerSec, velocityRadPerSec);
    setDriveVoltage(driveVolts);
  }

  public void setAngle(double angleRadians) {
    // final double setpoint =
    //     GeneralMath.getOptimalRotationalPIDGoal(
    //         inputs.turnAbsolutePositionRad,
    //         angleRadians,
    //         inputs.turnAbsoluteVelocityRadsPerSec,
    //         12 * Math.PI);
    // SmartDashboard.putNumber("Outputs/Setpoint", Units.radiansToDegrees(setpoint));
    SmartDashboard.putNumber(
        "Outputs/Velocity " + index, Units.radiansToDegrees(inputs.turnAbsoluteVelocityRadsPerSec));
    setTurnVoltage(turnFeedback.pid.calculate(inputs.turnAbsolutePositionRad, angleRadians));
    // setpointAngle = angleRadians;
    // setTurnVoltage(turnFeedback.pid.getP() * setpoint);
  }

  public void setDriveVoltage(double volts) {
    driveAppliedVolts = GeneralMath.clampMagnitude(volts, 11.0);
    driveSim.setInputVoltage(driveAppliedVolts);
  }

  public void setTurnVoltage(double volts) {
    turnAppliedVolts = GeneralMath.clampMagnitude(volts, 11.0);
    turnSim.setInputVoltage(turnAppliedVolts);
  }
}
