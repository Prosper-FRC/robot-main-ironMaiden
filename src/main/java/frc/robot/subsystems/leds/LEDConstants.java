package frc.robot.subsystems.leds;

public class LEDConstants {

  // LED port - Must be a PWM header, not MXP or DIO - TO BE CONFIGURED:
  public static final int k_LEDPort = 0;

  // Length of LEDs:
  public static final int k_length = 32;

  // Wait Command length to set time for LEDs to blink:
  public static final double k_waitTime = 0.1;

  // Booleans of whether or not LEDs are orange, blue, or purple:
  public static boolean k_isOrange = false;
  public static boolean k_isBlue = false;
  public static boolean k_isRed = false;

  // Sensor DIO port:
  public static final int k_DIOPort = 0;
}
