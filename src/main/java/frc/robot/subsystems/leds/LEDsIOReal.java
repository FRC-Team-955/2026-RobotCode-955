package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

import static frc.robot.subsystems.leds.LEDConstants.length;

public class LEDsIOReal extends LEDsIO {
    private final AddressableLED leds = new AddressableLED(6);

    public LEDsIOReal() {
        leds.setLength(length);
        leds.start();
    }

    @Override
    public void setData(AddressableLEDBuffer buffer) {
        leds.setData(buffer);
    }
}
