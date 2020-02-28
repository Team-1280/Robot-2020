package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.Constants;

public class LEDs{

    private AddressableLED ledStrip;
    private AddressableLEDBuffer ledBuffer;

    private int m_rainbowFirstPixelHue = 0;

    public enum LEDState{

    }

    public LEDs(){
        ledStrip = new AddressableLED(Constants.PWMLED);

        ledBuffer = new AddressableLEDBuffer(Constants.LEDStripLength);
        ledStrip.setLength(ledBuffer.getLength());

        // Set the data
        ledStrip.setData(ledBuffer);
        ledStrip.start();
    }

    private void rainbow() {
        // For every pixel
        for (var i = 0; i < ledBuffer.getLength(); i++) {
          // Calculate the hue - hue is easier for rainbows because the color
          // shape is a circle so only one value needs to precess
          final int hue = (m_rainbowFirstPixelHue + (i * 180 / ledBuffer.getLength())) % 180;
          // Set the value
          ledBuffer.setHSV(i, hue, 255, 128);
        }
        // Increase by to make the rainbow "move"
        m_rainbowFirstPixelHue += 3;
        // Check bounds
        m_rainbowFirstPixelHue %= 180;
      }
}