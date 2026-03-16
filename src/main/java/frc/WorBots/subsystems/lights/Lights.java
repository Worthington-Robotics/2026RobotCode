package frc.WorBots.subsystems.lights;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.WorBots.Constants;
import frc.WorBots.subsystems.lights.LightUtils.ColorSequence;
import frc.WorBots.subsystems.lights.LightsIO.LightStrip;
import frc.WorBots.subsystems.lights.LightsIO.LightSection;
import frc.WorBots.util.RebuiltUtils;
import frc.WorBots.util.cache.Cache.TimeCache;
import frc.WorBots.util.debug.StatusPage;

public class Lights extends SubsystemBase {
    public static Lights instance = new Lights();

    public static Lights getInstance(){
        return instance;
    }
    
    private LightStrip lights;
    private LightSection turretStrip;
    private LightSection leftStrip;
    private LightSection rightStrip;


    //What should be run if no override is present
    private LightModes currentMode = LightModes.Disabled;
    private LightEffects currentEffect = LightEffects.none;

    //These are what actually get run, they're here for the override
    private LightModes activeMode = currentMode;
    private LightEffects activeEffect = currentEffect;

    /** The override for the current mode */
    private Optional<LightModes> modeOverride = Optional.empty();

    /** The temporary flash effect */
    private Optional<LightEffects> effectOverride = Optional.empty();

    /** Timer for the temporary flash effect, restarting when the effect is applied */
    private final Timer effectTimer = new Timer();

    /**Util for measuring how long till hub switch */
    RebuiltUtils rebuiltUtils = new RebuiltUtils();
    
    //Robot State Varriables 

    /**What the turret is aiming at*/
    public enum Target{
      Hub,
      Pass,
      None;
    }
    /**Current Target the Turret is aiming at */
    private Target currentTarget = Target.None;

    private boolean climbing = false;

    private boolean superStar = false;

    /**Turret Status */
    private Optional<Boolean> shotPreped = Optional.empty();

    /**Vision Status */
    private Optional<Boolean> visionStatus = Optional.empty();

    /**Has a system fault been detected */
    private Optional<Boolean> sysFault = Optional.empty();

    /**Has a spindexer jam been detected */
    private Optional<Boolean> spinJammed = Optional.empty();


    //Mode specific varriables
    //solid mode
    private Color solidColor = Color.kBlack;

    //Color varriables
    private static final ColorSequence WORBOTS_FLAME_COLORS =
      new ColorSequence(
          Color.kWhite,
          Color.kCadetBlue,
          Color.kBlue,
          Color.kBlue,
          Color.kIndigo,
          Color.kIndigo,
          Color.kRed,
          Color.kRed,
          Color.kRed,
          Color.kRed,
          Color.kRed,
          Color.kBlack);

    public static enum LightModes{
        TurretDisplay,
        Climbing,
        SpinJam,
        SysFault,
        VisionLost,

        Disabled,
        Solid,
        PitLight;
    }

    public static enum LightEffects{
        none,
        invalidShotFlash,
        timePulse,
        superStar;
    }

    private Lights(){
        //TODO Add light Strips here
        lights = new LightStrip(0, 40);
        turretStrip = new LightSection(lights, 0, 13);
        leftStrip = new LightSection(lights, 14, 26);
        rightStrip = new LightSection(lights, 27, 39);
        StatusPage.reportStatus(StatusPage.LIGHTS_SUBSYSTEM, true);
    }

    public void periodic(){
        if(Constants.getSim()){
          addVisionStatus(true);
        }

        SmartDashboard.putString("Lights Mode", currentMode.toString());
        SmartDashboard.putString("Lights Effect", currentEffect.toString());
        SmartDashboard.putString("Lights Mode Override", modeOverride.toString());
        SmartDashboard.putString("Lights Effect Override", effectOverride.toString());

        if(DriverStation.isDisabled()){
          currentMode = LightModes.Disabled;

          if(SmartDashboard.getBoolean("DB/Button 1", false)){
            currentMode = LightModes.PitLight;
          }
        } else {
          //Priority from lowest to hightest, Turret Display, Vision Down, Spin Jam, Climbing, Sys Fault
          currentMode = LightModes.TurretDisplay;

          if(spinJammed.isPresent() && spinJammed.get() == true){
            currentMode = LightModes.SpinJam;
          }

          if(climbing){
            currentMode = LightModes.Climbing;
          }

        }
 
        if(visionStatus.isPresent() && visionStatus.get() == false){
          currentMode = LightModes.VisionLost;
        }

        if(sysFault.isPresent() && sysFault.get() == true){
          currentMode = LightModes.SysFault;
        }

        activeMode = currentMode;
        if(modeOverride.isPresent()){
          activeMode = modeOverride.get();
        }

        switch (activeMode) {
          case TurretDisplay:
            //Display turret status, yellow for aiming, green for hub lock, purple for passing
            if(shotPreped.isPresent() && shotPreped.get() == true){
              if(currentTarget == Target.Hub){
                Color deepGreen = new Color(0, 255, 10);
                solidColor = deepGreen;
              } else {
                solidColor = Color.kPurple;
              } 
            } else {
              Color orangeYellow = new Color(255, 100, 0);
              solidColor = orangeYellow;
            }

            LightUtils.solid(turretStrip, solidColor);
            LightUtils.solid(leftStrip, solidColor);
            LightUtils.solid(rightStrip, solidColor);

            if(rebuiltUtils.timeToAcivationSwitch() < 5){
              runEffect(LightEffects.timePulse);
            }

            if(superStar){
              runEffect(LightEffects.superStar);
            }

            break;
          case Climbing:
            //Displays blue light when climb is active to remind the drivers to chill
            Color deepBlue = new Color(0, 0, 255);
            solidColor = deepBlue;
            LightUtils.solid(turretStrip, solidColor);
            LightUtils.solid(leftStrip, solidColor);
            LightUtils.solid(rightStrip, solidColor);

            break;
          case SpinJam:
            //Displays orange light when Spindexer jam is detetcted
            Color trueOrange = new Color(255, 25, 0);
            LightUtils.blink(turretStrip, trueOrange, Color.kBlack,0.25, TimeCache.getInstance().get());
            LightUtils.blink(leftStrip, trueOrange, Color.kBlack,0.25, TimeCache.getInstance().get());
            LightUtils.blink(rightStrip, trueOrange, Color.kBlack,0.25, TimeCache.getInstance().get());

            break;
          case SysFault:
            //Displays solid red when a System Fault occurs
            solidColor = Color.kRed;
            LightUtils.solid(turretStrip, solidColor);
            LightUtils.solid(leftStrip, solidColor);
            LightUtils.solid(rightStrip, solidColor);

            break;
          case VisionLost:
            //Displays blinking white light if vision is lost
            LightUtils.blink(turretStrip, Color.kWhite, Color.kBlack,0.25, TimeCache.getInstance().get());
            LightUtils.blink(leftStrip, Color.kWhite, Color.kBlack,0.25, TimeCache.getInstance().get());
            LightUtils.blink(rightStrip, Color.kWhite, Color.kBlack,0.25, TimeCache.getInstance().get());

            break;
          case Solid:
            //Diplays a solid light of a set color, mostly a debug mode
            LightUtils.solid(turretStrip, solidColor);
            LightUtils.solid(leftStrip, solidColor);
            LightUtils.solid(rightStrip, solidColor);

            break;
          case PitLight:
            //Displays solid white light to make it easier to see while working on the robot
            solidColor = new Color(100, 100, 100);
            LightUtils.solid(turretStrip, solidColor);
            LightUtils.solid(leftStrip, solidColor);
            LightUtils.solid(rightStrip, solidColor);

            break;
          case Disabled:
            LightUtils.worbotsBounce(turretStrip);
            LightUtils.worbotsBounce(leftStrip);
            LightUtils.worbotsBounce(rightStrip);
            break;
        }

        lights.periodic();
    }

    public void runEffect(LightEffects effect){
      currentEffect = effect;
      activeEffect = effect;
      if(effectOverride.isPresent()){
        activeEffect = effectOverride.get();
      }

      switch (activeEffect) {
        case invalidShotFlash:
          //Flashes the lights yellow if the drivers try to shoot before the robot has a lock
          final double flashPeriod = 0.125;
          LightUtils.blink(turretStrip, Color.kGold, Color.kBlack, flashPeriod, effectTimer.get());
          LightUtils.blink(leftStrip, Color.kGold, Color.kBlack, flashPeriod, effectTimer.get());
          LightUtils.blink(rightStrip, Color.kGold, Color.kBlack, flashPeriod, effectTimer.get());
          if (effectTimer.get() > flashPeriod * 3.0) {
            effectOverride = Optional.empty();
          }

          break;
        case timePulse:
          final double minBrightness = 0.1;
          final double totalTime = 5.0;
          LightUtils.dopplerEffect(turretStrip, solidColor, minBrightness, totalTime, rebuiltUtils.timeToAcivationSwitch(), 0.8);
          LightUtils.dopplerEffect(leftStrip, solidColor, minBrightness, totalTime, rebuiltUtils.timeToAcivationSwitch(), 0.8);
          LightUtils.dopplerEffect(rightStrip, solidColor, minBrightness, totalTime, rebuiltUtils.timeToAcivationSwitch(), 0.8);

          break;
        case superStar:
          LightUtils.rainbow(turretStrip, 0.5, 1);
          LightUtils.rainbow(leftStrip, 0.5, 1);
          LightUtils.rainbow(rightStrip, 0.5, 1);

          break;
        case none:

          break;
        default:
          break;
        }
    }

    /**
   * Sets the mode of the lights
   *
   * @param mode The mode to set
   */
  public void setMode(LightModes mode) {
    this.currentMode = mode;
  }

  /**
   * Sets the effect of the lights
   * @param effect the effect to be played
   */
  public void setEffect(LightEffects effect) {
    this.currentEffect = effect;
    effectTimer.restart();
  }

  /**
   * Sets the target the lights believe the turret to be aiming at
   * @param target the target (Hub or passing)
   */
  public void setTarget(Target target){
    this.currentTarget = target;
  }

  /**
   * Tells the lights if the robot is climbing
   * @param status if the robot is climbing
   */
  public void setClimb(boolean status){
    this.climbing = status;
  }

  /**
   * Passes the turret's status to the lights
   * @param turretStatus is the turret at it's goal point
   */
  public void addShotStatus(boolean shotStatus){
    this.shotPreped = Optional.of(shotStatus);
  }

  /**
   * Passes the spindexer's status to the lights
   * @param jammed Is the spindexer jammed
   */
  public void addSpinStatus(boolean jammed){
    this.spinJammed = Optional.of(jammed);
  }

  /**
   * Passes visions status to the lights
   * @param visionStatus Is vision operational
   */
  public void addVisionStatus(boolean visionStatus){
    this.visionStatus = Optional.of(visionStatus);
  }

  /**
   * Alerts the lights of a SysFault
   * @param status Is a SysFault occuring (True if yes, False if everythings fine)
   */
  public void sysFault(boolean status){
    this.sysFault = Optional.of(status);
  }

  public void superStar(boolean superStar){
    this.superStar = superStar;
  }

  /**
   * Sets a Override for the lights
   * @param mode the override mode the lights will display reguardless of input
   */
  public void setOverride(LightModes mode) {
    this.modeOverride = Optional.of(mode);
  }

  /**
   * Sets a Override for the lights
   * @param effect the override effect the lights will play reguardless of other instructions
   */
  public void setOverride(LightEffects effect) {
    this.effectOverride = Optional.of(effect);
    effectTimer.reset();
  }

  /**
   * Sets a Override for the lights
   * @param mode the override mode the lights will display reguardless of input
   * @param effect the override effect the lights will play reguardless of other instructions
   */
  public void setOverride(LightModes mode, LightEffects effect){
    this.modeOverride = Optional.of(mode);
    this.effectOverride = Optional.of(effect);
    effectTimer.reset();
  }

  /**
   * Clears the mode override for the lights
   */
  public void clearModeOverride() {
    this.modeOverride = Optional.empty();
  }

  /**
   * Clears the effect override for the lights
   */
  public void clearEffectOverride(){
    this.effectOverride = Optional.empty();
  }

  /**
   * Clears all overrides for the lights
   */
  public void clearOverride(){
    this.modeOverride = Optional.empty();
    this.effectOverride = Optional.empty();
  }
}
