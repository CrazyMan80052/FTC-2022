package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.TickMotorMovement;

//TODO: convert inches to milimeters
public class ScanCone extends TickMotorMovement {
    /*
     * Change the I2C port below to match the connection of your color sensor
     */
    /*
    private final I2C.Port i2cPort = I2C.Port.kOnboard;

    /**
     * A Rev Color Sensor V3 object is constructed with an I2C port as a
     * parameter. The device will be automatically initialized with default
     * parameters.
     */
    //private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

    /**
     * A Rev Color Match object is used to register and detect known colors. This can
     * be calibrated ahead of time or during operation.
     *
     * This object uses a simple euclidian distance to estimate the closest match
     * with given confidence range.
     */
    //private final ColorMatch m_colorMatcher = new ColorMatch();

    /**
     * Note: Any example colors should be calibrated as the user needs, these
     * are here as a basic example.
     */
    /*private final Color kBlueTarget = new Color(0.143, 0.427, 0.429);
    private final Color kGreenTarget = new Color(0.197, 0.561, 0.240);
    private final Color kRedTarget = new Color(0.561, 0.232, 0.114);

    @Override
    public void robotInit() {
        m_colorMatcher.addColorMatch(kBlueTarget);
        m_colorMatcher.addColorMatch(kGreenTarget);
        m_colorMatcher.addColorMatch(kRedTarget);
        m_colorMatcher.addColorMatch(kYellowTarget);
    }

    @Override
    public void robotPeriodic() {
        /**
         * The method GetColor() returns a normalized color value from the sensor and can be
         * useful if outputting the color to an RGB LED or similar. To
         * read the raw color, use GetRawColor().
         *
         * The color sensor works best when within a few inches from an object in
         * well lit conditions (the built in LED is a big help here!). The farther
         * an object is the more light from the surroundings will bleed into the
         * measurements and make it difficult to accurately determine its color.
         */
        /*
        Color detectedColor = m_colorSensor.getColor();

        /**
         * Run the color match algorithm on our detected color

        ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

        if (match.color == kRedTarget) {
            positionOne();
        }
        else if (match.color == kGreenTarget) {
            positionTwo();
        }
        else if (match.color == kBlueTarget) {
            positionThree();
        }
    }
    */


    public void findPosition(int red, int green, int blue) {
        forward(33); //put outside this shit

        if(red > green && red > blue) {
            positionOne();
        }
        else if(blue > red && blue > green) {
            positionTwo();
        }
        else {
            positionThree();
        }

    }

    public void positionOne() {
        strafeLeft(609.6);
        forward(304.8);
    }

    public void positionTwo() {
        forward(133.35);
    }

    public void positionThree() {
        strafeRight(609.6);
        forward(304.8);
    }
}