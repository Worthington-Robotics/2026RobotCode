package frc.WorBots.util;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

//TODO make some tool to standize the time data from DriverStation.getMatchTime() across all conditions

public class RebuiltUtils {
  public RebuiltUtils() {
  }

  /**
   * Returns if the hub is currently active
   */
  public boolean isHubActive() {
    return timeToHubActive() == 0;
  }

  /**
   * Returns the time until the hub is active
   */
  public double timeToHubActive() {
    boolean allianceStarts = false;
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isEmpty()) {
      return 99;
    }
    if (DriverStation.isAutonomous()) {
      return 0;
    }
    if (!DriverStation.isTeleopEnabled()) {
      return 99; 
    }

    double matchTime = MatchTime.getInstance().getTimeRemaining();
    if (matchTime > 130 || matchTime < 30) {
      return 0; // If in endgame or transition period the hub is always active
    }
    String gameData = DriverStation.getGameSpecificMessage();

    if (gameData.isEmpty()) {
      return 0; // Currently set to assume that it is on if no data is recieved, this can be
                // changed if this becomes a problem
    }

    // Checking for bad game data
    if (!(gameData.charAt(0) == 'R' || gameData.charAt(0) == 'B')) {
      return 0;
    }

    if ((alliance.get() == Alliance.Red && gameData.charAt(0) == 'R')
        || (alliance.get() == Alliance.Blue && gameData.charAt(0) == 'B')) {
      allianceStarts = true;
    }

    if (matchTime > 105) {
      if (allianceStarts) {
        return 0;
      }
      return matchTime - 105;
    } else if (matchTime > 80) {
      if (!allianceStarts) {
        return 0;
      }
      return matchTime - 80;
    } else if (matchTime > 55) {
      if (allianceStarts) {
        return 0;
      }
      return matchTime - 55;
    } else if (matchTime > 30) {
      if (!allianceStarts) {
        return 0;
      }
      return matchTime - 30;
    }
    return 999; // This should never trigger
  }

  /**
   * @return The time until the next hub switch
   */
  public double timeToAcivationSwitch() {
    double matchTime = MatchTime.getInstance().getTimeRemaining();
    if (DriverStation.isTeleop()) {
      if (matchTime > 130) {
        return matchTime - 130;

      } else if (matchTime > 105) {
        return matchTime - 105;

      } else if (matchTime > 80) {
        return matchTime - 80;

      } else if (matchTime > 55) {
        return matchTime - 55;

      } else if (matchTime > 30) {
        return matchTime - 30;
      } else {
        // This really shouldn't trigger
        return 999;
      }
    } else if(DriverStation.isAutonomous()) {
      return matchTime;
    } else {
      return 999;
    }
  }
}
