// Copyright (c) 2026 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots;

public class CanIDs {
  /**A class that contains all of the swerve CANids */
  public class Swerve{
    /**CanBus for swerve and pigeon */
    public static final String CAN_BUS = "rio";

    public static final int PIGEON_ID = 0;

    public static final int FRONT_LEFT_DRIVE_ID = 1;
    public static final int FRONT_RIGHT_DRIVE_ID = 4;
    public static final int BACK_LEFT_DRIVE_ID = 7;
    public static final int BACK_RIGHT_DRIVE_ID = 10;

    public static final int FRONT_LEFT_TURN_ID = 2;
    public static final int FRONT_RIGHT_TURN_ID = 5;
    public static final int BACK_LEFT_TURN_ID = 8;
    public static final int BACK_RIGHT_TURN_ID = 11;

    public static final int FRONT_LEFT_ENCODER_ID = 3; 
    public static final int FRONT_RIGHT_ENCODER_ID = 6;
    public static final int BACK_LEFT_ENCODER_ID = 9;
    public static final int BACK_RIGHT_ENCODER_ID = 12;
  }

  /**
   * A class that contains all superstrucure CanIDs
   */
  public class SuperStructure{
    /**CanBus for everything that isn't swerve */
    public static String CAN_BUS = "Main";
    //Spindexer
    public static final int SPINDEXER_ID = 16;
    public static final int KICKER_ID = 17;

    //Turret
    public static final int TURRET_ID = 14;
    public static final int TURRET_ABS_ENCODER_ID = 15;

    //Shooter
    public static final int HOOD_ID = 18;
    public static final int LEADER_ID = 19;
    public static final int FOLLOWER_ID = 20;

    //Intake
    public static final int EXTENDING_MOTOR_ID = 21;
    public static final int INTAKE_MOTOR_ID = 22;
    //public static final int TOF_ID = 0;

    //Climber
    public static final int CLIMBER_ID = 13;
  }


  
}
