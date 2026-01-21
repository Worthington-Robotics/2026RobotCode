package frc.WorBots.subsystems.vision.apriltags;

import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import frc.WorBots.subsystems.vision.apriltags.TagVisionIO.TagVisionIOInputs;
import frc.WorBots.util.cache.Cache.TimeCache;

public class TagVisionIOPhoton {
    
  private final NetworkTableInstance defaultInstance = NetworkTableInstance.getDefault();
  private final NetworkTable table;
  private final NetworkTable subTable;
  private final DoubleArraySubscriber data;
  private final DoubleSubscriber subFps;

  /** NT connection timeout after which to report a disconnect */
  private static final double CONNECTION_TIMEOUT = 1.5;

  public TagVisionIOPhoton(String camera) {
    table = defaultInstance.getTable("photonvision/" + camera);
    subTable = table.getSubTable("output");
    data =
        subTable
            .getDoubleArrayTopic("data")
            .subscribe(
                new double[] {}, PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true));
    subFps =
        subTable
            .getDoubleTopic("fps")
            .subscribe(0.0, PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true));
  }

  public void updateInputs(TagVisionIOInputs inputs) {
    final TimestampedDoubleArray[] frames = data.readQueue();
    final int length = frames.length;

    inputs.timestamps = new double[length];
    inputs.frames = new double[length][];

    for (int i = 0; i < length; i++) {
      inputs.timestamps[i] = frames[i].timestamp / 1000000.0;
      inputs.frames[i] = frames[i].value;
    }

    inputs.fps = subFps.get();
    final double lastUpdate = subFps.getLastChange() / 1000000.0;
    inputs.isConnected = (TimeCache.getInstance().get() - lastUpdate < CONNECTION_TIMEOUT);
  }
    
}
