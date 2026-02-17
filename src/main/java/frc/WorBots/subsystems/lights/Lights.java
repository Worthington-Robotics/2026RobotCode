package frc.WorBots.subsystems.lights;

import java.util.Optional;

import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.WorBots.subsystems.drive.Drive;

public class Lights extends SubsystemBase {
    public static Lights instance = new Lights();

    public static Lights getInstance(){
        return instance;
    }

    private LightModes currentMode = LightModes.Disabled;
    private LightEffects currentEffects = LightEffects.none;

    /** The override for the current mode */
    private Optional<LightModes> modeOverride = Optional.empty();

    /** The temporary flash effect */
    private Optional<LightEffects> effectOverride = Optional.empty();

    /** Timer for the temporary flash effect, restarting when the effect is applied */
    private final Timer effectTimer = new Timer();

    /**true if robot is in our alliance zone, false otherwise */
    private boolean isNear;

    //Used for isNear
    private Drive drive;

    //Mode specific varriables

    //solid mode
    private Color solidColor = Color.kBlack;

    public static enum LightModes{
        Aiming,
        HubLocked,
        Passing,
        Climbing,
        SpinJam,
        SysFault,
        VisionLost,

        Disabled,
        Solid,
        PitTest;
    }

    public static enum LightEffects{
        none,
        invalidShotFlash;
    }

    private Lights(){
        //TODO Add light Strips here
    }

    public void periodic(){
        SmartDashboard.putString("Lights Mode", currentMode.toString());
        SmartDashboard.putString("Lights Effect", currentEffects.toString());
        SmartDashboard.putString("Lights Mode Override", modeOverride.toString());
        SmartDashboard.putString("Lights Effect Override", effectOverride.toString());

        isNear = drive.isNear();
    }

    public void runEffect(LightEffects effect){
        switch (effect) {
            case invalidShotFlash:
                
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

  public void setEffect(LightEffects effect) {
    this.currentEffects = effect;
    effectTimer.restart();
  }

  public void setOverride(LightModes mode) {
    this.modeOverride = Optional.of(mode);
  }

  public void setOverride(LightEffects effect) {
    this.effectOverride = Optional.of(effect);
    effectTimer.reset();
  }

  public void clearModeOverride() {
    this.modeOverride = Optional.empty();
  }

  public void clearEffectOverride(){
    this.effectOverride = Optional.empty();
  }

  public void clearOverride(){
    this.modeOverride = Optional.empty();
    this.effectOverride = Optional.empty();
  }
}
