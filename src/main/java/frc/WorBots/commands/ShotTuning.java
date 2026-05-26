// Copyright (c) 2026 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.WorBots.subsystems.superstructure.Superstructure;
import frc.WorBots.subsystems.superstructure.Superstructure.SuperstructureControlMode;
import frc.WorBots.util.debug.TunableDouble;

/** A command that is used to tune shots */
public class ShotTuning extends Command {
  private Superstructure superstructure;

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

  /**
   * A command that is used to tune shots
   * 
   * @param superstructure  The superstructure to use
   * @param povUpSupplier   A supplier providing the POV up value. Used to
   *                        increase flywheel speed.
   * @param povDownSupplier A supplier providing the POV down value. Used to
   *                        decrease flywheel speed.
   * @param aSupplier       A supplier providing the a button value. Used to
   *                        decrease hood angle.
   * @param bSupplier       A supplier providing the b button value. Used to
   *                        increase hood angle.
   */
  public ShotTuning(Superstructure superstructure, Supplier<Boolean> povUpSupplier, Supplier<Boolean> povDownSupplier,
      Supplier<Boolean> aSupplier, Supplier<Boolean> bSupplier) {
    addRequirements(superstructure);
    this.superstructure = superstructure;
    this.povUpSupplier = povUpSupplier;
    this.povDownSupplier = povDownSupplier;
    this.aSupplier = aSupplier;
    this.bSupplier = bSupplier;
    radspm.set(0);
    hoodAngle.set(0);
  }

  @Override
  public void execute() {
    superstructure.setControlMode(SuperstructureControlMode.ManualShot);
    if (upToggle && povUpSupplier.get()) {
      upToggle = false;
      radspm.set(radspm.get() + RADS_PER_MIN_ADJUSTMENT_AMOUNT);
    }
    if (downToggle && povDownSupplier.get()) {
      downToggle = false;
      radspm.set(radspm.get() - RADS_PER_MIN_ADJUSTMENT_AMOUNT);
    }
    if (aToggle && aSupplier.get()) {
      aToggle = false;
      hoodAngle.set(hoodAngle.get() - ANGLE_ADJUSTMENT_AMOUNT);
    }
    if (bToggle && bSupplier.get()) {
      bToggle = false;
      hoodAngle.set(hoodAngle.get() + ANGLE_ADJUSTMENT_AMOUNT);
    }
    if (aToggle && aSupplier.get())
      if (!povUpSupplier.get()) {
        upToggle = true;
      }
    if (!povDownSupplier.get()) {
      downToggle = true;
    }
    if (!aSupplier.get()) {
      aToggle = true;
    }
    if (!bSupplier.get()) {
      bToggle = true;
    }

  }

}
