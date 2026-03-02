// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.util;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.WorBots.Constants;
import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

/* Taken and modified from team 6328. */
public class OdometryThread extends Thread {
  private static final boolean IS_CANFD = false;
  public static final double PERIOD = 1.0 / 320.0;

  private final Lock signalsLock =
      new ReentrantLock(); // Prevents conflicts when registering signals
  private BaseStatusSignal[] signals = new BaseStatusSignal[0];
  private final List<Queue<Double>> queues = new ArrayList<>();

  public static final Lock odometryLock = new ReentrantLock();
  public static final Queue<Double> timestampQueue = new ArrayBlockingQueue<>(20);

  private static OdometryThread instance = null;

  public static OdometryThread getInstance() {
    if (instance == null) {
      instance = new OdometryThread();
    }
    return instance;
  }

  public OdometryThread() {
    setName("Odometry");
    setDaemon(true);
    start();
  }

  public Queue<Double> registerSignal(StatusSignal<Angle> signal) {
    Queue<Double> queue = new ArrayBlockingQueue<>(20);
    signalsLock.lock();
    odometryLock.lock();
    try {
      BaseStatusSignal[] newSignals = new BaseStatusSignal[signals.length + 1];
      System.arraycopy(signals, 0, newSignals, 0, signals.length);
      newSignals[signals.length] = signal;
      signals = newSignals;
      queues.add(queue);
    } finally {
      signalsLock.unlock();
      odometryLock.unlock();
    }

    return queue;
  }

  @Override
  public void run() {
    while (true) {
      // Wait for updates from all signals
      signalsLock.lock();
      double latencyAverage = 0.0;
      try {
        if (IS_CANFD) {
          BaseStatusSignal.waitForAll(Constants.RobotConstants.ROBOT_PERIOD, signals);
        } else {
          Thread.sleep((long) (PERIOD * 1000.0));
          if (signals.length > 0) {
            BaseStatusSignal.refreshAll(signals);
          }

          // Get the average latency for latency compensation
          double[] latencies = new double[signals.length];
          double latencySum = 0.0;
          for (int i = 0; i < latencies.length; i++) {
            final double latency = signals[i].getAllTimestamps().getBestTimestamp().getLatency();
            latencies[i] = latency;
            latencySum += latency;
          }
          SmartDashboard.putNumberArray("Odometry Latencies", latencies);
          if (latencies.length == 0) {
            latencyAverage = 0.0;
          } else {
            latencyAverage = latencySum / latencies.length;
          }
        }
      } catch (InterruptedException e) {
        e.printStackTrace();
      } finally {
        signalsLock.unlock();
      }
      final double fpgaTimestamp = Timer.getFPGATimestamp();

      // Save new data to queues
      odometryLock.lock();
      try {
        for (int i = 0; i < signals.length; i++) {
          queues.get(i).offer(signals[i].getValueAsDouble());
        }
        timestampQueue.offer(fpgaTimestamp - latencyAverage);
      } finally {
        odometryLock.unlock();
      }
    }
  }
}
