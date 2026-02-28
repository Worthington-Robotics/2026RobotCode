package frc.WorBots.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.WorBots.subsystems.shooter.Shooter;
import frc.WorBots.subsystems.turret.Turret;
import frc.WorBots.util.debug.TunableDouble;

public class ShotTuning extends Command{
  private Shooter shooter;
  private Turret turret;

  private final static double ANGLE_ADJUSTMENT_AMOUNT = 1;
  private final static double RADS_PER_MIN_ADJUSTMENT_AMOUNT = 0.1;

  private Supplier<Boolean> povUpSupplier;
  private Supplier<Boolean> povDownSupplier;
  private Supplier<Boolean> aSupplier;
  private Supplier<Boolean> bSupplier;

  private boolean upToggle = true;
  private boolean downToggle = true;
  private boolean aToggle = true;
  private boolean bToggle = true;

  private TunableDouble radspm = new TunableDouble("Tuning", "Shooter", "rpm");
  private TunableDouble hoodAngle = new TunableDouble("Tuning", "Shooter", "hood angle");

  public ShotTuning(Shooter shooter, Turret turret, Supplier<Boolean> povUpSupplier, Supplier<Boolean> povDownSupplier, Supplier<Boolean> aSupplier, Supplier<Boolean> bSupplier){
    addRequirements(shooter, turret);
    this.shooter = shooter;
    this.turret = turret;
    this.povUpSupplier = povUpSupplier;
    this.povDownSupplier = povDownSupplier;
    this.aSupplier = aSupplier;
    this.bSupplier = bSupplier;
    radspm.set(0);
    hoodAngle.set(0);
  }
  
  @Override
  public void execute(){
    turret.setPositionMinDistFromZero(0);
    if(upToggle && povUpSupplier.get()){
      upToggle = false;
      radspm.set(radspm.get() + RADS_PER_MIN_ADJUSTMENT_AMOUNT);
    }
    if(downToggle && povDownSupplier.get()){
      downToggle = false;
      radspm.set(radspm.get() - RADS_PER_MIN_ADJUSTMENT_AMOUNT);
    }
    if(aToggle && aSupplier.get()){
      aToggle = false;
      hoodAngle.set(hoodAngle.get() - ANGLE_ADJUSTMENT_AMOUNT);
    }
    if(bToggle && bSupplier.get()){
      bToggle = false;
      hoodAngle.set(hoodAngle.get() + ANGLE_ADJUSTMENT_AMOUNT);
    }
    if(aToggle && aSupplier.get())
    if(!povUpSupplier.get()){upToggle = true;}
    if(!povDownSupplier.get()){downToggle = true;}
    if(!aSupplier.get()){aToggle = true;}
    if(!bSupplier.get()){bToggle = true;}
    shooter.setFlywheelSpeed(radspm.get());
    shooter.setHoodPose(hoodAngle.get());
  }
  
}
