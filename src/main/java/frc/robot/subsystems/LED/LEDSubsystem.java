package frc.robot.subsystems.LED;


import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

 public class LEDSubsystem extends SubsystemBase
 {
   /**
    * The led strip object
    */
  //  private final AddressableLED m_led;

  //  /**
  //   * The object that holds the data for the led strip
  //   */
  //  private final AddressableLEDBuffer m_ledBuffer;

   /**
    * The constructor initializes the LED strip
    */
   public LEDSubsystem()
   {
    //  m_led = new AddressableLED(Constants.LEDConstants.port);
    //  m_ledBuffer = new AddressableLEDBuffer(Constants.LEDConstants.length);
    //  m_led.setLength(m_ledBuffer.getLength());

    //  m_led.setData(m_ledBuffer);
    //  m_led.start();

   }

    /**
    * Sets the color of the LED using RGB values
    * @param red the Red in RGB
    * @param green the Green in RGB
    * @param blue the Blue in RGB
    */
  //  public void setLEDStrip(int red, int green, int blue){
  //    for(int i = 0; i< m_ledBuffer.getLength();i++){
  //      m_ledBuffer.setRGB(i,red,green,blue);
  //    }
  //  }

   /**
    * Sets the LEDs to a rainbow pattern, starting at red
    */
  //  public void rainbow(){
  //    int m_rainbowFirstPixelHue = 0;
  //    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
  //           final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
  //            m_ledBuffer.setHSV(i, hue, 255, 128);
  //          }
  //        m_rainbowFirstPixelHue += 3;
  //        m_rainbowFirstPixelHue %= 180;
  //  }
   /**
    * Turns off the LED lights
    */
  //  public void stopLED(){
  //    m_led.stop();
  //  }

  //  public Command disabledLED(){
  //      return run(() -> {
  //          setLEDStrip(0,76,153);
  //      });
  //  }

  //  public enum LEDState{

  //  }

  //    @Override
  //    public void periodic()
  //    {

    // }
 }