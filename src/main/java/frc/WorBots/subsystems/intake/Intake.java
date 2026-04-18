package frc.WorBots.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.WorBots.util.control.DerivativeFilter;
import frc.WorBots.util.debug.NTLogger;
import frc.WorBots.util.debug.StatusPage;
import frc.WorBots.Constants;
import frc.WorBots.energy.PowerLogger.SubsystemLog;
import frc.WorBots.subsystems.intake.IntakeIO.IntakeIOInputs;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputs inputs = new IntakeIOInputs();

  private enum ExtendControlMode {
    Disabled,
    Voltage,
    Position,
    Agitate
  }

  private enum IntakeMotorControlMode {
    Disabled,
    Voltage
  }

  private ExtendControlMode extendControlMode = ExtendControlMode.Disabled;
  private IntakeMotorControlMode intakeMotorControlMode = IntakeMotorControlMode.Disabled;

  private ProfiledPIDController extendController = new ProfiledPIDController(
      Constants.IntakeConstants.INTAKE_EXTENDING_KP, Constants.IntakeConstants.INTAKE_EXTENDING_KI,
      Constants.IntakeConstants.INTAKE_EXTENDING_KD,
      new Constraints(Constants.IntakeConstants.INTAKE_EXTENDING_MAX_VEL,
          Constants.IntakeConstants.INTAKE_EXTENDING_MAX_ACEL));
  private ArmFeedforward extendFeedforwardController = new ArmFeedforward(Constants.IntakeConstants.EXTENDER_KS,
      Constants.IntakeConstants.EXTENDER_KG,
      Constants.IntakeConstants.EXTENDER_KV);

  // The setpoint voltages for both the intaking and extending motors
  private double setPointVoltageIntake = 0.0;

  private DerivativeFilter intakeFilter = new DerivativeFilter(
      Constants.IntakeConstants.INTAKE_VOLTAGE / (Constants.IntakeConstants.INTAKE_SPIN_UP_SECONDS));

  private double setPointVoltageExtending = 0.0;
  private double setPointPositionExtending = 0.0;

  private double agitatePoseMod = 0.0;

  // Current draw and setpoint publishers.
  private final NetworkTableInstance instance = NetworkTableInstance.getDefault();
  private final NetworkTable intakeTable = instance.getTable("Intake");
  private final DoublePublisher setpointIntakePub = intakeTable.getDoubleTopic("Intake Setpoint Volts").publish();
  private final DoublePublisher setpointExtendingPub = intakeTable.getDoubleTopic("Extending Setpoint Volts").publish();
  private final DoublePublisher extendPositionPub = intakeTable.getDoubleTopic("Extend Position").publish();
  private final DoublePublisher extendGoalPub = intakeTable.getDoubleTopic("Extend Goal Position").publish();
  private final DoublePublisher currentDrawIntakePub = intakeTable.getDoubleTopic("Intake Current Draw").publish();
  private final DoublePublisher currentDrawExtendingPub = intakeTable.getDoubleTopic("Extending Current Draw")
      .publish();
  private final DoublePublisher voltsPublisher = intakeTable.getDoubleTopic("Requested Voltage").publish();

  public Intake(IntakeIO io) {
    this.io = io;
    StatusPage.reportStatus(StatusPage.INTAKE_SUBSYSTEM, true);
    extendController.setTolerance(Constants.IntakeConstants.EXTENDER_POS_TOLERANCE);
    extendController.setTolerance(Constants.IntakeConstants.INTAKE_EXTEND_TOLERANCE);
  }

  /***
   * Updates every period, reports Status, checks if the motors are too hot,
   * sets the voltages supplied to each motor, and publishes the motors, current
   * draw,
   * and setpoints voltages.
   */
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    inputs.intakeMotor.publish();
    inputs.extendingMotor.publish();

    // If the heat exceeds the max, turn it off.
    if (inputs.intakeMotor.temperatureCelsius > Constants.IntakeConstants.INTAKE_MAX_TEMP || DriverStation.isDisabled()
        || inputs.extendingMotor.temperatureCelsius > Constants.IntakeConstants.INTAKE_MAX_TEMP) {
      extendControlMode = ExtendControlMode.Disabled;
    }

    StatusPage.reportStatus(
        StatusPage.INTAKE_CONNECTED,
        inputs.extendingMotor.isConnected && inputs.intakeMotor.isConnected
            && inputs.intakeMotor.temperatureCelsius <= Constants.IntakeConstants.INTAKE_MAX_TEMP);

    NTLogger.putString("Intake", "Intake Control Mode", extendControlMode.toString());

    // Control intaking motor
    if (intakeMotorControlMode == IntakeMotorControlMode.Disabled) {
      setPointVoltageIntake = 0;
      io.setIntakeMotorVolts(0);
    } else {
      double finalSetpointIntake = 0;
      // If the intake is trying to intake, filter it to reduce power draw
      if (setPointVoltageIntake < 0.0) {
        finalSetpointIntake = setPointVoltageIntake;
      } else {
        finalSetpointIntake = intakeFilter.calculate(setPointVoltageIntake);
      }
      // Command the motors
      io.setIntakeMotorVolts(finalSetpointIntake);
      setpointIntakePub.set(finalSetpointIntake);
    }

    // Control the extending motor
    if (extendControlMode == ExtendControlMode.Disabled) {
      setPointVoltageExtending = 0;
      io.setExtendingMotorVolts(0);
    } else if (extendControlMode == ExtendControlMode.Voltage) {
      // Set motor voltage
      io.setExtendingMotorVolts(setPointVoltageExtending);
      setpointExtendingPub.set(setPointVoltageExtending);
    } else {
      // Position control mode
      final double goal;
      // If in agigate control mode do agitate logic
      if (extendControlMode == ExtendControlMode.Agitate) {
        // Slowly raise the intake
        agitatePoseMod += (IntakePoses.RETRACTED.pose - IntakePoses.EXTENDED.pose)
            / (Constants.IntakeConstants.INTAKE_SECONDS_TO_AUTO_AGITATE * Constants.RobotConstants.ROBOT_FREQUENCY);
        io.setIntakeMotorVolts(7);
        goal = MathUtil.clamp(setPointPositionExtending + agitatePoseMod, Constants.IntakeConstants.EXTENDER_MIN_LIMIT,
            IntakePoses.HALF.pose + .1);
      } else {
        goal = MathUtil.clamp(setPointPositionExtending, Constants.IntakeConstants.EXTENDER_MIN_LIMIT,
            Constants.IntakeConstants.EXTENDER_MAX_LIMIT);
      }
      final double feedback = extendController.calculate(inputs.extendPosition, goal);
      final double feedforward = extendFeedforwardController.calculate(extendController.getSetpoint().position,
          extendController.getSetpoint().velocity);
      double volts = feedforward + feedback;
      if (extendController.atGoal() && goal != IntakePoses.HALF.pose) {
        volts = 0;
      }
      voltsPublisher.set(volts);
      io.setExtendingMotorVolts(MathUtil.clamp(volts, -8, 8));
    }
    // Publishing the current draw for extending and intaking motors.
    extendPositionPub.set(inputs.extendPosition);
    currentDrawIntakePub.set(inputs.intakeCurrent);
    currentDrawExtendingPub.set(inputs.extendingCurrent);
    inputs.extendingMotor.publish();
    extendGoalPub.set(setPointPositionExtending);
  }

  /**
   * Returns the voltage setpoint for the intake motor
   */
  public double getSetPointVoltageIntake() {
    return setPointVoltageIntake;
  }

  /**
   * Returns the voltage setpoint for the extend motor
   */
  public double getSetPointVoltageExtending() {
    return setPointVoltageExtending;
  }

  /**
   * Sets the intak motor to run at a voltage
   * 
   * @param voltage The voltage to run the intake motor at
   */
  public void setVoltsIntake(double voltage) {
    intakeMotorControlMode = IntakeMotorControlMode.Voltage;
    setPointVoltageIntake = voltage;
  }

  /**
   * Sets the extend motor to run at a voltage
   * 
   * @param voltage The voltage to run the extend motor at
   */
  public void setVoltsExtending(double voltage) {
    extendControlMode = ExtendControlMode.Voltage;
    setPointVoltageExtending = voltage;
  }

  /**
   * Moves the intake to the extended position
   */
  public void extend() {
    extendControlMode = ExtendControlMode.Position;
    setPointPositionExtending = IntakePoses.EXTENDED.get();
    extendController.reset(inputs.extendPosition);
  }

  /**
   * Moves the intake to the retracted position
   */
  public void retract() {
    extendControlMode = ExtendControlMode.Position;
    setPointPositionExtending = IntakePoses.RETRACTED.get();
    extendController.reset(inputs.extendPosition);
  }

  /**
   * Raises the intake to the agitate position
   */
  public void agitate() {
    extendControlMode = ExtendControlMode.Position;
    setPointPositionExtending = IntakePoses.HALF.get();
    extendController.reset(inputs.extendPosition);
  }

  /**
   * Slowly raise the intake to agitate balls; designed for use during autonomous
   */
  public void agigateAuto() {
    extendControlMode = ExtendControlMode.Agitate;
    agitatePoseMod = 0.0;
  }

  /**
   * Stops slowly raising the intake for agigate in auto
   */
  public void stopAgitateAuto() {
    extendControlMode = ExtendControlMode.Position;
  }

  /**
   * Returns if the intake is set to be in its extended position
   */
  public boolean isExtended() {
    return setPointPositionExtending == IntakePoses.EXTENDED.get();
  }

  /**
   * Returns if the extend motor is at its goal
   */
  public boolean atGoal() {
    return extendController.atGoal();
  }

  /**
   * Disables the subsystem
   */
  public void disable() {
    extendControlMode = ExtendControlMode.Disabled;
    intakeMotorControlMode = IntakeMotorControlMode.Disabled;
  }

  /**
   * Code to run at the start of teleop; extends the intake
   */
  public void teleopInit() {
    extend();
  }

  /**
   * Returns the subsystem's power log to update the power logger
   */
  public SubsystemLog getPowerLog() {
    return new SubsystemLog("Intake", new String[] { "Intake Motor", "Extend Motor" },
        new double[] { inputs.intakeMotor.appliedPowerVolts, inputs.extendingMotor.appliedPowerVolts },
        new double[] { inputs.intakeMotor.currentDrawAmps, inputs.extendingMotor.currentDrawAmps });
  }
}
