package frc.WorBots.util.debug;

import java.util.HashMap;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.GenericPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.Topic;

public class NTLogger {
  private static HashMap<String, GenericPublisher> publisherMap = new HashMap<>();

  private static final NetworkTableInstance instance = NetworkTableInstance.getDefault();

  public static void putNumber(String table, String key, double value) {
    getPublisher(table, key, "double").setDouble(value);
  }

  public static void putBoolean(String table, String key, boolean value) {
    getPublisher(table, key, "boolean").setBoolean(value);
  }
  
  public static void putString(String table, String key, String value) {
    getPublisher(table, key, "string").setString(value);
  }

  public static void putNumberArray(String table, String key, double[] value) {
    getPublisher(table, key, "double[]").setDoubleArray(value);
  }

  public static void putBooleanArray(String table, String key, boolean[] value) {
    getPublisher(table, key, "boolean[]").setBooleanArray(value);
  }

  public static void putStringArray(String table, String key, String[] value) {
    getPublisher(table, key, "string[]").setStringArray(value);
  }


  private static GenericPublisher getPublisher(String table, String key, String type){
    Optional<GenericPublisher> publisher = Optional.ofNullable(publisherMap.get(table + "/" + key));
    if(publisher.isPresent()){
      return publisher.get();
    }
    Topic topic = instance.getTopic(table + "/" + key);
    publisher = Optional.ofNullable(topic.genericPublish(type));
    publisherMap.put(table + "/" + key, publisher.get());
    return publisher.get();
  }
  
}
