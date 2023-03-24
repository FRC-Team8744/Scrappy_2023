// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDLights extends SubsystemBase {
  private final boolean DEBUG = false;

  /** Creates a new LEDLights. */
  private AddressableLED m_led;
  // private AddressableLED m_led2;
  private AddressableLEDBuffer m_ledBuffer; //m_ledBuffer2;
  // Store what the last hue of the first pixel is
  private int m_rainbowFirstPixelHue;
  
  public int LED_r, LED_g, LED_b;

  public LEDLights() {
  
    // PWM port 0
    // Must be a PWM header, not MXP or DIO
    m_led = new AddressableLED(0);
    // m_led2 = new AddressableLED(1);
    // Reuse buffer
    // Default to a length of 60, start empty output
    // Length is expensive to set, so only set it once, then just update data
    m_ledBuffer = new AddressableLEDBuffer(60);
    // m_ledBuffer2 = new AddressableLEDBuffer(60);
    m_led.setLength(m_ledBuffer.getLength());
    // m_led2.setLength(m_ledBuffer2.getLength());

    // Set the data
    m_led.setData(m_ledBuffer);
    m_led.start();

    // m_led2.setData(m_ledBuffer2);
    // m_led2.start();


    LED_r=0;
    LED_g=255;
    LED_b=0;

    if (DEBUG) {
      SmartDashboard.putNumber("LED_R", LED_r);
      SmartDashboard.putNumber("LED_G", LED_g);
      SmartDashboard.putNumber("LED_B", LED_b);
      }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Fill the buffer with a rainbow
    // rainbow();
    solidcolor(LED_r, LED_g, LED_b);
    // Set the LEDs
    m_led.setData(m_ledBuffer);
    // m_led2.setData(m_ledBuffer2);


    if (DEBUG) {
      // read colors from SmartDashBoard
      double r = SmartDashboard.getNumber("LED_R", 0);
      double g = SmartDashboard.getNumber("LED_G", 0);
      double b = SmartDashboard.getNumber("LED_B", 0);

      // if colors on SmartDashboard have changed, write new values
      if(r != LED_r) {LED_r = (int)r;}
      if(g != LED_g) {LED_g = (int)g;}
      if(b != LED_b) {LED_b = (int)b;}
      }
  }  

  private void rainbow() {
    // For every pixel
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
      // Set the value
      m_ledBuffer.setHSV(i, hue, 255, 128);
      // m_ledBuffer2.setHSV(i, hue, 255, 128);

    }
    // Increase by to make the rainbow "move"
    m_rainbowFirstPixelHue += 3;
    // Check bounds
    m_rainbowFirstPixelHue %= 180;
    
  }

  private void solidcolor(int r, int g, int b) {
    // For every pixel
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      // final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
      // // Set the value
      // m_ledBuffer.setHSV(i, hue, 255, 128);
      m_ledBuffer.setRGB(i, r, g, b);
      // m_ledBuffer2.setRGB(i, r, g, b);

    }
    // Increase by to make the rainbow "move"
    m_rainbowFirstPixelHue += 3;
    // Check bounds
    m_rainbowFirstPixelHue %= 180;

    
  }

  public void setcolor(int r, int g, int b) {
    LED_r=r;
    LED_g=g;
    LED_b=b;
    

    
  }

}
