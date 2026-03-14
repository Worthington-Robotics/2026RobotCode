package frc.WorBots.subsystems.vision.apriltags;

import java.nio.ByteBuffer;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StructArraySubscriber;
import edu.wpi.first.networktables.StructSubscriber;
import edu.wpi.first.networktables.TimestampedObject;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj.Timer;
import frc.WorBots.util.cache.Cache.TimeCache;

public class TagVisionIONew implements TagVisionIO {
  private final NetworkTableInstance defaultInstance = NetworkTableInstance.getDefault();
  private final StructSubscriber<TagVisionPoseUpdate> data;
  private final StructArraySubscriber<TagVisionDetection> detections;
  private final DoubleSubscriber subFps;
  /** NT connection timeout after which to report a disconnect */
  private static final double CONNECTION_TIMEOUT = 1.5;

  public TagVisionIONew(String camName) {
    data = defaultInstance
        .getStructTopic("TagVision/" + camName + "/Data", TagVisionPoseUpdate.struct)
        .subscribe(
            null, PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true));
    detections = defaultInstance
        .getStructArrayTopic("TagVision/" + camName + "/Detections", TagVisionDetection.struct)
        .subscribe(
            new TagVisionDetection[] {}, PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true));
    subFps = defaultInstance.getDoubleTopic("TagVision/FPS")
        .subscribe(0.0, PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true));
  }

  public void updateInputs(TagVisionIOInputs inputs) {
    final TimestampedObject<TagVisionPoseUpdate>[] frames = data.readQueue();
    final int length = frames.length;

    inputs.timestamps = new double[length];
    inputs.frames = new double[length][];
    inputs.covariances = new Pose3d[length];

    for (int i = 0; i < length; i++) {
      TagVisionPoseUpdate update = frames[i].value;
      inputs.timestamps[i] = Timer.getFPGATimestamp() - 0.05;

      double[] frame = new double[10];
      frame[0] = 1.0;
      frame[1] = update.error;
      frame[2] = update.pose.getX();
      frame[3] = update.pose.getY();
      frame[4] = update.pose.getZ();
      frame[5] = update.pose.getRotation().getQuaternion().getW();
      frame[6] = update.pose.getRotation().getQuaternion().getX();
      frame[7] = update.pose.getRotation().getQuaternion().getY();
      frame[8] = update.pose.getRotation().getQuaternion().getZ();
      frame[9] = update.tag;
      inputs.frames[i] = frame;

      inputs.covariances[i] = new Pose3d(update.covx, update.covy, update.covz, new Rotation3d(0.0, 0.0, update.covrz));
    }

    inputs.fps = subFps.get();
    final double lastUpdate = subFps.getLastChange() / 1000000.0;
    inputs.isConnected = (TimeCache.getInstance().get() - lastUpdate < CONNECTION_TIMEOUT);
  }

  static class TagVisionPoseUpdate implements StructSerializable {
    Pose3d pose;
    double error;
    double timestamp;
    short tag;
    double covx;
    double covy;
    double covz;
    double covrx;
    double covry;
    double covrz;

    public static final TagVisionPoseUpdateStruct struct = new TagVisionIONew.TagVisionPoseUpdateStruct();

    public TagVisionPoseUpdate(Pose3d pose, double error, double timestamp, short tag, double covx, double covy, double covz, double covrx, double covry, double covrz) {
      this.pose = pose;
      this.error = error;
      this.timestamp = timestamp;
      this.tag = tag;
      this.covx = covx;
      this.covy = covy;
      this.covz = covz;
      this.covrx = covrx;
      this.covry = covry;
      this.covrz = covrz;
    }
  }

  static class TagVisionPoseUpdateStruct implements Struct<TagVisionPoseUpdate> {
    @Override
    public Class<TagVisionPoseUpdate> getTypeClass() {
      return TagVisionPoseUpdate.class;
    }

    @Override
    public String getTypeName() {
      return "TagVisionPoseUpdate";
    }

    @Override
    public int getSize() {
      return Pose3d.struct.getSize() + kSizeDouble + kSizeDouble + kSizeInt8 + kSizeDouble * 6;
    }

    @Override
    public String getSchema() {
      return "Pose3d pose;double error;double timestamp;int8 tag;double covx;double covy;double covz;double covrx;double covry;double covrz;";
    }

    @Override
    public Struct<?>[] getNested() {
      return new Struct<?>[] { Pose3d.struct };
    }

    @Override
    public TagVisionPoseUpdate unpack(ByteBuffer bb) {
      Pose3d pose = Pose3d.struct.unpack(bb);
      double error = bb.getDouble();
      double timestamp = bb.getDouble();
      short tag = bb.getShort();
      double covx = bb.getDouble();
      double covy = bb.getDouble();
      double covz = bb.getDouble();
      double covrx = bb.getDouble();
      double covry = bb.getDouble();
      double covrz = bb.getDouble();
      return new TagVisionPoseUpdate(pose, error, timestamp, tag, covx, covy, covz, covrx, covry, covrz);
    }

    @Override
    public void pack(ByteBuffer bb, TagVisionPoseUpdate value) {
      Pose3d.struct.pack(bb, value.pose);
      bb.putDouble(value.error);
      bb.putDouble(value.timestamp);
    }
  }

  static class TagVisionDetection implements StructSerializable {
    Pose3d pose;
    int id;
    Translation3d c1;
    Translation3d c2;
    Translation3d c3;
    Translation3d c4;

    public static final TagVisionDetectionStruct struct = new TagVisionIONew.TagVisionDetectionStruct();

    public TagVisionDetection(Pose3d pose, int id, Translation3d c1, Translation3d c2, Translation3d c3, Translation3d c4) {
      this.pose = pose;
      this.id = id;
      this.c1 = c1;
      this.c2 = c2;
      this.c3 = c3;
      this.c4 = c4;
    }
  }

  static class TagVisionDetectionStruct implements Struct<TagVisionDetection> {
    @Override
    public Class<TagVisionDetection> getTypeClass() {
      return TagVisionDetection.class;
    }

    @Override
    public String getTypeName() {
      return "TagVisionDetection";
    }

    @Override
    public int getSize() {
      return Pose3d.struct.getSize() + kSizeInt32 + Translation3d.struct.getSize() * 4;
    }

    @Override
    public String getSchema() {
      return "Pose3d pose;int32 id;Translation3d c1;Translation3d c2;Translation3d c3;Translation3d c4;";
    }

    @Override
    public Struct<?>[] getNested() {
      return new Struct<?>[] { Pose3d.struct, Translation3d.struct, Translation3d.struct, Translation3d.struct, Translation3d.struct };
    }

    @Override
    public TagVisionDetection unpack(ByteBuffer bb) {
      Pose3d pose = Pose3d.struct.unpack(bb);
      int id = bb.getInt();
      Translation3d c1 = Translation3d.struct.unpack(bb);
      Translation3d c2 = Translation3d.struct.unpack(bb);
      Translation3d c3 = Translation3d.struct.unpack(bb);
      Translation3d c4 = Translation3d.struct.unpack(bb);
      return new TagVisionDetection(pose, id, c1, c2, c3, c4);
    }

    @Override
    public void pack(ByteBuffer bb, TagVisionDetection value) {
      Pose3d.struct.pack(bb, value.pose);
      bb.putInt(value.id);
    }
  }
}
