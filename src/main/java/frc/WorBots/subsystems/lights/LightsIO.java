// Copyright (c) 2026 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.subsystems.lights;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

/* Interface for a section of LEDs */
public interface LightsIO {
  /**
   * Wrapper function that sets an LED
   *
   * @param index The index of the LED
   * @param color The desired color
   */
  public void setLED(int index, Color color);

  /**
   * Wrapper function that sets an LED
   *
   * @param index The index of the LED
   * @param h The hue of the LED
   * @param s The saturation of the LED
   * @param v The value of LED
   */
  public void setHSV(int index, int h, int s, int v);

  /** Gets the number of LEDs in the strip */
  public int getCount();

  /** Simple hardware wrapper around a set of addressable LEDs on one PWM port */
  public static class LightStrip implements LightsIO {
    /** Dimming factor to apply to the LED colors */
    private static final double DIMMING_FACTOR = 0.9;

    private final AddressableLED leds;
    private final AddressableLEDBuffer buffer;
    private final int count;

    public LightStrip(int id, int count) {
      // Initialize lights and buffer
      leds = new AddressableLED(id);
      buffer = new AddressableLEDBuffer(count);
      leds.setLength(count);
      leds.start();
      this.count = count;
    }

    public void periodic() {
      // Dim all the LEDs
      for (int i = 0; i < count; i++) {
        final Color color = buffer.getLED(i);
        final Color dimmed =
            new Color(
                color.red * DIMMING_FACTOR,
                color.green * DIMMING_FACTOR,
                color.blue * DIMMING_FACTOR);
        buffer.setLED(i, dimmed);
      }

      // Set the LEDs
      leds.setData(buffer);
    }

    @Override
    public void setLED(int index, Color color) {
      if (index < 0 || index >= count) {
        return;
      }

      buffer.setLED(index, color);
    }

    @Override
    public void setHSV(int index, int h, int s, int v) {
      if (index < 0 || index >= count) {
        return;
      }

      buffer.setHSV(index, h, s, v);
    }

    /** Gets the number of LEDs in the strip */
    @Override
    public int getCount() {
      return this.count;
    }

    /** Gets a section of lights from this strip */
    public LightSection getSection(int start, int end) {
      return new LightSection(this, start, end);
    }
  }

  public static class LightSection implements LightsIO {
    private final LightStrip strip;
    private final int start;
    private final int end;

    protected LightSection(LightStrip strip, int start, int end) {
      this.strip = strip;
      this.start = start;
      this.end = end;
    }

    /**
     * Wrapper function that sets an LED
     *
     * @param index The index of the LED
     * @param color The desired color
     */
    @Override
    public void setLED(int index, Color color) {
      if (index >= end) {
        return;
      }

      strip.setLED(index + start, color);
    }

    /**
     * Wrapper function that sets an LED
     *
     * @param index The index of the LED
     * @param h The hue of the LED
     * @param s The saturation of the LED
     * @param v The value of LED
     */
    @Override
    public void setHSV(int index, int h, int s, int v) {
      if (index >= end) {
        return;
      }

      strip.setHSV(index + start, h, s, v);
    }

    /** Gets the number of LEDs in the strip */
    @Override
    public int getCount() {
      return end - start;
    }
  }

  public static class DummyLights implements LightsIO {
    @Override
    public void setLED(int index, Color color) {}

    @Override
    public void setHSV(int index, int h, int s, int v) {}

    /** Gets the number of LEDs in the strip */
    @Override
    public int getCount() {
      return 1;
    }
  }
}
