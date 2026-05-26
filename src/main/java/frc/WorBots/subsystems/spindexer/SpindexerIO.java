// Copyright (c) 2026 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.subsystems.spindexer;

import frc.WorBots.util.HardwareUtils.TalonInputs;
import frc.WorBots.util.HardwareUtils.TalonInputsPositional;

public interface SpindexerIO {
    public static class SpindexerIOInputs{
        TalonInputsPositional talon = new TalonInputsPositional("Spindexer", "Lead Signals");
        TalonInputs follower = new TalonInputs("Spindexer", "Follower Signals");
        public double spinVelocity = 0;
        public double kickerVelocity = 0;
        public boolean active = false;
        public boolean jammed = false;
    }

    public default void updateInputs(SpindexerIOInputs inputs){}

    public default void setKickerVoltage(double volts){}

    public default void setSpinVoltage(double volts){}

    public default void stop(){}
}
