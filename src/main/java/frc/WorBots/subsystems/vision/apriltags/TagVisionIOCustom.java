package frc.WorBots.subsystems.vision.apriltags;

import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import frc.WorBots.util.cache.Cache.TimeCache;

//Camera FOV = about 75 degree
//Partial credit to Team 6328
public class TagVisionIOCustom implements TagVisionIO {
  private final NetworkTableInstance defaultInstance = NetworkTableInstance.getDefault();
  private final NetworkTable table;
  private final NetworkTable subTable;
  private final DoubleArraySubscriber data;
  private final DoubleSubscriber subFps;

  private final IntegerPublisher detectionModePublisher;
  private final IntegerPublisher detectionTagPublisher;

  /** NT connection timeout after which to report a disconnect */
  private static final double CONNECTION_TIMEOUT = 1.5;

  public TagVisionIOCustom(int index) {
    table = defaultInstance.getTable("module" + (index + 1));
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

    final NetworkTable detectionTable = table.getSubTable("detection");
    detectionModePublisher = detectionTable.getIntegerTopic("mode").publish();
    detectionTagPublisher = detectionTable.getIntegerTopic("tag").publish();
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

  @Override
  public void setDetectionConfig(DetectionMode mode, int specificTag) {
    detectionModePublisher.set(mode.ordinal());
    detectionTagPublisher.set(specificTag);
  }
}
