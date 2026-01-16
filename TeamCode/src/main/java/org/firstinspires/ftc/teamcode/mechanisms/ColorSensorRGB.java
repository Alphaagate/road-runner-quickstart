package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class ColorSensorRGB extends OpMode {
    TestBenchColourSensing bench = new TestBenchColourSensing();
    private int ballCount;

    @Override
    public void init() {
        bench.init(hardwareMap);
    }


    @Override
    public void loop() {
        bench.getDetectedColor(telemetry);
        colorSensor();


    }
    NormalizedColorSensor colorSensor;


    public enum DetectedColor {
        RED,
        BLUE,
        YELLOW,
        UNKNOWN
    }
    protected void colorSensor() {

        if (((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM)<3){
            ballCount += 1;
        }
        if (ballCount ==1) {


            //rgblight
        } else if (ballCount == 2) {

        } else if (ballCount == 3) {
            //reset
        }
    }
    public void init(HardwareMap hwMap) {
        colorSensor = hwMap.get(NormalizedColorSensor.class, "sensor_color_distance");
    }


    public TestBenchColourSensing.DetectedColor getDetectedColor(Telemetry telemetry ) {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        float normRed, normGreen, normBlue;
        normRed = colors.red / colors.alpha;
        normGreen = colors.green / colors .alpha;
        normBlue = colors.blue / colors.alpha;

        telemetry.addData("red", normRed);
        telemetry.addData("green", normGreen);
        telemetry.addData("blue", normBlue);

        //TODO add if statements for specific colors added
        /*
        red, green, blue
        RED =
        GREEN =
        BLUE =
         */

        return TestBenchColourSensing.DetectedColor.UNKNOWN;

    }



}
