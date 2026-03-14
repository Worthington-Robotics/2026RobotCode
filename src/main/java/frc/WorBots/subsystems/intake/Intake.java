package frc.WorBots.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.WorBots.util.debug.StatusPage;
import frc.WorBots.Constants;
import frc.WorBots.subsystems.intake.IntakeIO.IntakeIOInputs;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputs inputs = new IntakeIOInputs();

  // TODO add a velocity PID to the intake run motor
  private enum ControlMode {
    Disabled,
    Voltage,
    Position
  }

  private ControlMode controlMode = ControlMode.Disabled;

  private ProfiledPIDController extendController = new ProfiledPIDController(
      Constants.IntakeConstants.INTAKE_EXTENDING_KP, Constants.IntakeConstants.INTAKE_EXTENDING_KI,
      Constants.IntakeConstants.INTAKE_EXTENDING_KD,
      new Constraints(Constants.IntakeConstants.INTAKE_EXTENDING_MAX_VEL,
          Constants.IntakeConstants.INTAKE_EXTENDING_MAX_ACEL));
  // private ArmFeedforward feedforwardController = new ArmFeedforward(Constants.IntakeConstants.EXTENDER_KS,
  //     Constants.IntakeConstants.EXTENDER_KG,
  //     Constants.IntakeConstants.EXTENDER_KV);

  // The setpoint voltages for both the intaking and extending motors
  private double setPointVoltageIntake = 0.0;

  private double setPointVoltageExtending = 0.0;

  private double setPointPositionExtending = 0.0;

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

  // TODO add softstops

  /***
   * Updates every period, reports Status, checks if the motors are too hot,
   * sets the voltages supplied to each motor, and publishes the motors, current
   * draw,
   * and setpoints voltages.
   */
  @Override
  public void periodic() {
    io.updateInputs(inputs);

    // If the heat exceeds the max, turn it off.
    if (inputs.intakeMotor.temperatureCelsius > Constants.IntakeConstants.INTAKE_MAX_TEMP || DriverStation.isDisabled()
        || inputs.extendingMotor.temperatureCelsius > Constants.IntakeConstants.INTAKE_MAX_TEMP) {
      controlMode = ControlMode.Disabled;
    }

    StatusPage.reportStatus(
        StatusPage.INTAKE_CONNECTED,
        inputs.extendingMotor.isConnected && inputs.intakeMotor.isConnected
            && inputs.intakeMotor.temperatureCelsius <= Constants.IntakeConstants.INTAKE_MAX_TEMP);

    SmartDashboard.putString("Intake Control Mode", controlMode.toString());
    if (controlMode == ControlMode.Disabled) {
      setPointVoltageIntake = 0;
      setPointVoltageExtending = 0;
    } else {
      double finalSetpointIntake = 0;
      // Don't try to intake when we are too high, grinds gears
      if (inputs.extendPosition <= IntakePoses.HALF.get()){
        finalSetpointIntake = setPointVoltageIntake;
      }
      io.setIntakeMotorVolts(finalSetpointIntake);
      inputs.intakeMotor.publish();
      setpointIntakePub.set(finalSetpointIntake);

      if (controlMode == ControlMode.Voltage) {
        // Setting the voltages of the motors
        io.setExtendingMotorVolts(setPointVoltageExtending);
        // Publishing the setpoint voltage for extending and intaking motors.
        setpointExtendingPub.set(setPointVoltageExtending);

      } else {
        final double goal = MathUtil.clamp(setPointPositionExtending, Constants.IntakeConstants.EXTENDER_MIN_LIMIT,
            Constants.IntakeConstants.EXTENDER_MAX_LIMIT);
        // double feedforward = feedforwardController.calculate(
        //     extendController.getSetpoint().position,
        //     extendController.getSetpoint().velocity);

        // if (inputs.extendPosition > Constants.IntakeConstants.EXTENDER_FRICTION_ZONE) {
        //   feedforward += Constants.IntakeConstants.EXTENDER_FRICTION_ZONE_KS
        //       * Math.signum(extendController.getSetpoint().velocity);
        // }

        // SmartDashboard.putNumber("Intake Setpoint", extendController.getSetpoint().position);
        // SmartDashboard.putNumber("Intake PID output", feedback);
        // SmartDashboard.putNumber("Intake FF output", feedforward);
        // double volts = GeneralMath.hardLimitVelocity(feedback + feedforward, inputs.extendPosition,
        //     Constants.IntakeConstants.EXTENDER_MIN_LIMIT,
        //     Constants.IntakeConstants.EXTENDER_MAX_LIMIT);

        // if (extendController.atGoal() && extendController.getGoal().position == IntakePoses.EXTENDED.pose){
        //   volts = 0;
        // }
        // io.setExtendingMotorVolts(volts);
        final double feedback = extendController.calculate(inputs.extendPosition, goal);
        double volts;
        if(goal == IntakePoses.RETRACTED.pose){
          volts = Math.cos(inputs.extendPosition)  * Constants.IntakeConstants.RETRACT_MULT + feedback;
        } else if(goal == IntakePoses.HALF.pose){
          volts = Math.cos(inputs.extendPosition)  * Constants.IntakeConstants.RETRACT_MULT + feedback;
        } else if(goal == IntakePoses.EXTENDED.pose){
          volts = (Math.cos(inputs.extendPosition) * Constants.IntakeConstants.EXTEND_MULT_UPWARD) + (-Math.sin(inputs.extendPosition)*Constants.IntakeConstants.EXTEND_MULT_DOWNWARD);
          if(Math.abs(setPointPositionExtending - inputs.extendPosition) < Constants.IntakeConstants.INTAKE_EXTEND_EXTEND_POSE_TOLERANCE){
            volts = 0;
          }
        } else {
          volts = 0;
        }
        if (extendController.atGoal() && goal != IntakePoses.HALF.pose){
          volts = 0;
        }
        voltsPublisher.set(volts);
        io.setExtendingMotorVolts(MathUtil.clamp(volts, -8, 8));

        // Publishing the motor signals.

      }
    }
    // Publishing the current draw for extending and intaking motors.
    extendPositionPub.set(inputs.extendPosition);
    currentDrawIntakePub.set(inputs.intakeCurrent);
    currentDrawExtendingPub.set(inputs.extendingCurrent);
    inputs.extendingMotor.publish();
    extendGoalPub.set(setPointPositionExtending);

  }

  // Getters and Setters

  public double getSetPointVoltageIntake() {
    return setPointVoltageIntake;
  }

  public double getSetPointVoltageExtending() {
    return setPointVoltageExtending;
  }

  public void setVoltsIntake(double voltage) {
    controlMode = ControlMode.Voltage;
    setPointVoltageIntake = voltage;

  }

  public void setVoltsExtending(double voltage) {
    controlMode = ControlMode.Voltage;
    setPointVoltageExtending = voltage;
  }

  public void extend() {
    controlMode = ControlMode.Position;
    setPointPositionExtending = IntakePoses.EXTENDED.get();
    extendController.reset(inputs.extendPosition);
  }

  public void retract() {
    controlMode = ControlMode.Position;
    setPointPositionExtending = IntakePoses.RETRACTED.get();
    extendController.reset(inputs.extendPosition);
  }

  public void agitate(){
    controlMode = ControlMode.Position;
    setPointPositionExtending = IntakePoses.HALF.get();
    extendController.reset(inputs.extendPosition);
  }

  public boolean isExtended() {
    return setPointPositionExtending == IntakePoses.EXTENDED.get();
  }

  public boolean atGoal() {
    return extendController.atGoal();
  }

  public void disable(){
    controlMode = ControlMode.Disabled;
    io.setIntakeMotorVolts(0);
  }

}
