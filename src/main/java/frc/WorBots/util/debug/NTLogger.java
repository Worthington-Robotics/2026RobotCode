// Copyright (c) 2026 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.util.debug;

import java.util.HashMap;
import java.util.Optional;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.Topic;
import edu.wpi.first.util.struct.Struct;

/** A utility class to log values to NetworkTables */
public class NTLogger {
  private static HashMap<String, GenericPublisher> publisherMap = new HashMap<>();
  private static HashMap<String, StructPublisher<?>> structPublisherMap = new HashMap<>();
  private static HashMap<String, StructArrayPublisher<?>> structArrayPublisherMap = new HashMap<>();

  private static final NetworkTableInstance instance = NetworkTableInstance.getDefault();

  /**
   * Publishes a numeric value
   * 
   * @param table The NetworkTables table name
   * @param key   The entry key within the table
   * @param value The value to be stored
   */
  public static void putNumber(String table, String key, double value) {
    getPublisher(table, key, "double").setDouble(value);
  }

  /**
   * Publishes a boolean value
   * 
   * @param table The NetworkTables table name
   * @param key   The entry key within the table
   * @param value The value to be stored
   */
  public static void putBoolean(String table, String key, boolean value) {
    getPublisher(table, key, "boolean").setBoolean(value);
  }

  /**
   * Publishes a string
   * 
   * @param table The NetworkTables table name
   * @param key   The entry key within the table
   * @param value The value to be stored
   */
  public static void putString(String table, String key, String value) {
    getPublisher(table, key, "string").setString(value);
  }

  /**
   * Publishes a double array
   * 
   * @param table The NetworkTables table name
   * @param key   The entry key within the table
   * @param value The value to be stored
   */
  public static void putNumberArray(String table, String key, double[] value) {
    getPublisher(table, key, "double[]").setDoubleArray(value);
  }

  /**
   * Publishes a boolean array
   * 
   * @param table The NetworkTables table name
   * @param key   The entry key within the table
   * @param value The value to be stored
   */
  public static void putBooleanArray(String table, String key, boolean[] value) {
    getPublisher(table, key, "boolean[]").setBooleanArray(value);
  }

  /**
   * Publishes a string array
   * 
   * @param table The NetworkTables table name
   * @param key   The entry key within the table
   * @param value The value to be stored
   */
  public static void putStringArray(String table, String key, String[] value) {
    getPublisher(table, key, "string[]").setStringArray(value);
  }

  /**
   * Publishes a Struct-serializable value to NetworkTables.
   * The value type must match the provided Struct serializer.
   * 
   * @param table  The NetworkTables table name
   * @param key    The entry key within the table
   * @param value  The value to be stored
   * @param struct The struct serializer for the value type
   */
  public static void putStruct(String table, String key, Object value, Struct<?> struct) {
    getStructPublisher(table, key, struct).set(value);
  }

  /**
   * Publishes an array of Struct-serializable values to NetworkTables.
   * The value type must match the provided Struct serializer.
   * 
   * @param table  The NetworkTables table name
   * @param key    The entry key within the table
   * @param value  The value to be stored
   * @param struct The struct serializer for the value type
   */
  public static void putStructArray(String table, String key, Object[] value, Struct<?> struct) {
    getStructArrayPublisher(table, key, struct).set(value);
  }

  /**
   * Returns a publisher if it already exists, otherwise creates, stores, and
   * returns the publisher
   * 
   * @param table TheNetworkTables table name
   * @param key   The entry key within the table
   * @param type  The type of the publisher
   */
  private static GenericPublisher getPublisher(String table, String key, String type) {
    Optional<GenericPublisher> publisher = Optional.ofNullable(publisherMap.get(table + "/" + key));
    if (publisher.isPresent()) {
      return publisher.get();
    }
    Topic topic = instance.getTopic(table + "/" + key);
    publisher = Optional.ofNullable(topic.genericPublish(type));
    publisherMap.put(table + "/" + key, publisher.get());
    return publisher.get();
  }

  /**
   * Returns a struct publisher if it already exists, otherwise creates, stores,
   * and returns the publisher
   * 
   * @param table  The NetworkTables table name
   * @param key    The entry key within the table
   * @param struct The struct serializer matching the type of the value that the
   *               publisher will publish
   */
  private static <T> StructPublisher<T> getStructPublisher(String table, String key, Struct<?> struct) {
    @SuppressWarnings("unchecked")
    Optional<StructPublisher<T>> publisher = Optional
        .ofNullable((StructPublisher<T>) structPublisherMap.get(table + "/" + key));
    if (publisher.isPresent()) {
      return publisher.get();
    }
    @SuppressWarnings("unchecked")
    StructPublisher<T> pub = (StructPublisher<T>) instance.getStructTopic(table + "/" + key, struct).publish();
    structPublisherMap.put(table + "/" + key, pub);
    return pub;
  }

  /**
   * Returns a struct array publisher if it already exists, otherwise creates,
   * stores, and returns the publisher
   * 
   * @param table  The NetworkTables table name
   * @param key    The entry key within the table
   * @param struct The struct serializer matching the type of the value that the
   *               publisher will publish
   */
  private static <T> StructArrayPublisher<T> getStructArrayPublisher(String table, String key, Struct<?> struct) {
    @SuppressWarnings("unchecked")
    Optional<StructArrayPublisher<T>> publisher = Optional
        .ofNullable((StructArrayPublisher<T>) structArrayPublisherMap.get(table + "/" + key));
    if (publisher.isPresent()) {
      return publisher.get();
    }
    @SuppressWarnings("unchecked")
    StructArrayPublisher<T> pub = (StructArrayPublisher<T>) instance.getStructArrayTopic(table + "/" + key, struct)
        .publish();
    structArrayPublisherMap.put(table + "/" + key, pub);
    return pub;
  }

  /**
   * Publishes a value of a common type
   * 
   * @param table The NetworkTables table name
   * @param key   The entry key within the table
   * @param value The value to publish
   * @apiNote For publishing of unsupported types use {@code putStruct} or
   *          {@code putStructArray}
   */
  public static void put(String table, String key, double value) {
    putNumber(table, key, value);
  }

  /**
   * Publishes a value of a common type
   * 
   * @param table The NetworkTables table name
   * @param key   The entry key within the table
   * @param value The value to publish
   * @apiNote For publishing of unsupported types use {@code putStruct} or
   *          {@code putStructArray}
   */
  public static void put(String table, String key, boolean value) {
    putBoolean(table, key, value);
  }

  /**
   * Publishes a value of a common type
   * 
   * @param table The NetworkTables table name
   * @param key   The entry key within the table
   * @param value The value to publish
   * @apiNote For publishing of unsupported types use {@code putStruct} or
   *          {@code putStructArray}
   */
  public static void put(String table, String key, String value) {
    putString(table, key, value);
  }

  /**
   * Publishes a value of a common type
   * 
   * @param table The NetworkTables table name
   * @param key   The entry key within the table
   * @param value The value to publish
   * @apiNote For publishing of unsupported types use {@code putStruct} or
   *          {@code putStructArray}
   */
  public static void put(String table, String key, double[] value) {
    putNumberArray(table, key, value);
  }

  /**
   * Publishes a value of a common type
   * 
   * @param table The NetworkTables table name
   * @param key   The entry key within the table
   * @param value The value to publish
   * @apiNote For publishing of unsupported types use {@code putStruct} or
   *          {@code putStructArray}
   */
  public static void put(String table, String key, boolean[] value) {
    putBooleanArray(table, key, value);
  }

  /**
   * Publishes a value of a common type
   * 
   * @param table The NetworkTables table name
   * @param key   The entry key within the table
   * @param value The value to publish
   * @apiNote For publishing of unsupported types use {@code putStruct} or
   *          {@code putStructArray}
   */
  public static void put(String table, String key, String[] value) {
    putStringArray(table, key, value);
  }

  /**
   * Publishes a value of a common type
   * 
   * @param table The NetworkTables table name
   * @param key   The entry key within the table
   * @param value The value to publish
   * @apiNote For publishing of unsupported types use {@code putStruct} or
   *          {@code putStructArray}
   */
  public static void put(String table, String key, ChassisSpeeds value) {
    putStruct(table, key, value, ChassisSpeeds.struct);
  }

  /**
   * Publishes a value of a common type
   * 
   * @param table The NetworkTables table name
   * @param key   The entry key within the table
   * @param value The value to publish
   * @apiNote For publishing of unsupported types use {@code putStruct} or
   *          {@code putStructArray}
   */
  public static void put(String table, String key, SwerveModuleState value) {
    putStruct(table, key, value, SwerveModuleState.struct);
  }

  /**
   * Publishes a value of a common type
   * 
   * @param table The NetworkTables table name
   * @param key   The entry key within the table
   * @param value The value to publish
   * @apiNote For publishing of unsupported types use {@code putStruct} or
   *          {@code putStructArray}
   */
  public static void put(String table, String key, SwerveModuleState[] value) {
    putStructArray(table, key, value, SwerveModuleState.struct);
  }
}
