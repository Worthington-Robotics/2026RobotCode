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
import frc.WorBots.util.FireController;
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
    Position,
    Agitate
  }

  private enum IntakeMotorControlMode {
    Disabled,
    Voltage,
    Pulse
  }

  private ControlMode controlMode = ControlMode.Disabled;
  private IntakeMotorControlMode intakeMotorControlMode = IntakeMotorControlMode.Disabled;

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

  private int pulseCount = 0;

  private boolean unJamming = false;
  private boolean agitateInAuto = false;
  private boolean agitatedLast = false;

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
    inputs.intakeMotor.publish();

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
    //Control intaking motor
    if(intakeMotorControlMode == IntakeMotorControlMode.Disabled){
      setPointVoltageIntake = 0;
      io.setIntakeMotorVolts(0);
    } else if (intakeMotorControlMode == IntakeMotorControlMode.Voltage){
      double finalSetpointIntake = 0;
      // Don't try to intake when we are too high, grinds gears
      if (inputs.extendPosition <= IntakePoses.HALF.get()){
        finalSetpointIntake = setPointVoltageIntake;
      }
      if(isJammed()){
        pulseCount = (pulseCount+1)%51;
        if(pulseCount>50){
          unJamming = false;
          finalSetpointIntake = -Math.abs(finalSetpointIntake);
        } else {
          unJamming = true;
          finalSetpointIntake = -Math.abs(finalSetpointIntake);
        }
      }
      io.setIntakeMotorVolts(finalSetpointIntake);
      setpointIntakePub.set(finalSetpointIntake);
    } else {
      //Pulse control mode
      double finalSetpointIntake = 0;
      // Don't try to intake when we are too high, grinds gears
      if (inputs.extendPosition <= IntakePoses.HALF.get()){
        finalSetpointIntake = setPointVoltageIntake;
      }
      // Don't try to intake when we are too high, grinds gears
      if (inputs.extendPosition <= IntakePoses.HALF.get()){
        pulseCount = (pulseCount+1)%(int) (100 * (Constants.IntakeConstants.INTAKE_PULSE_INTAKING_SEC + Constants.IntakeConstants.INTAKE_PULSE_SPIT_SEC));
        if(pulseCount < (int)(100 * Constants.IntakeConstants.INTAKE_PULSE_INTAKING_SEC)){
          io.setIntakeMotorVolts(finalSetpointIntake);
        } else {
          io.setExtendingMotorVolts(-finalSetpointIntake);
        }
      }
    }

    //Control extension
    if (controlMode == ControlMode.Disabled) {
      setPointVoltageExtending = 0;
      io.setExtendingMotorVolts(0);
    } else if (controlMode == ControlMode.Voltage){
      // Setting the voltages of the motors
        io.setExtendingMotorVolts(setPointVoltageExtending);
        // Publishing the setpoint voltage for extending and intaking motors.
        setpointExtendingPub.set(setPointVoltageExtending);
    }else {
      //Position control mode
      if(controlMode == ControlMode.Agitate && (agitateInAuto || !DriverStation.isAutonomous())){
        //Agitate control mode running on top of position control mode
        if(FireController.getInstance().shouldAgitate() && setPointVoltageIntake == 0){
          agitatedLast = true;
          if(setPointPositionExtending == IntakePoses.EXTENDED.pose && atGoal()){
            setPointPositionExtending = IntakePoses.HALF.pose;
          } else if(setPointPositionExtending == IntakePoses.HALF.pose && atGoal()) {
            setPointPositionExtending = IntakePoses.EXTENDED.pose;
          }
        } else if (agitatedLast){
          setPointPositionExtending = IntakePoses.EXTENDED.pose;
          agitatedLast = false;
        }
      }
        final double goal = MathUtil.clamp(setPointPositionExtending, Constants.IntakeConstants.EXTENDER_MIN_LIMIT,
            Constants.IntakeConstants.EXTENDER_MAX_LIMIT);
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
    intakeMotorControlMode = IntakeMotorControlMode.Voltage;
    setPointVoltageIntake = voltage;
  }

  
  public void pulse(double voltage){
    intakeMotorControlMode = IntakeMotorControlMode.Pulse;
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
    intakeMotorControlMode = IntakeMotorControlMode.Disabled;
  }

  public void teleopInit(){
    extend();
  }
  public boolean isJammed(){
    return inputs.extendingMotor.velocityRadsPerSec < Constants.IntakeConstants.INTAKE_JAMMED_THRESHHOLD || unJamming;
  }

  public void setAgigateInAuto(boolean agitateInAuto){
    this.agitateInAuto = agitateInAuto;
  }

}
