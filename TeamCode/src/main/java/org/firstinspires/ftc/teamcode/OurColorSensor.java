package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/*
 * This OpMode shows how to use a color sensor in a generic way.
 * It adds a color enum to store the current detected color (BLUE, YELLOW, RED, or OTHER).
 */

//@TeleOp(name = "Sensor: Color with Motor Control", group = "Sensor")
public class OurColorSensor{

  private NormalizedColorSensor colorSensor;
  private float gain = 2;
  private final float[] hsvValues = new float[3];
  private int relativeLayoutId = 0;
  private View relativeLayout = null;
  // Enum to represent the detected color
  public enum DetectedColor {
    BLUE, YELLOW, RED, OTHER
  }

  private DetectedColor detectedColor = DetectedColor.OTHER; // Enum variable

  public void init(HardwareMap hardwareMap) {
    relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
    relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);
    colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");

    try {
      if (colorSensor instanceof SwitchableLight) {
        ((SwitchableLight) colorSensor).enableLight(true);
      }
    } finally {
      relativeLayout.post(() -> relativeLayout.setBackgroundColor(Color.WHITE));
    }

  }


  public DetectedColor detectColor(Telemetry telemetry){
    NormalizedRGBA colors = colorSensor.getNormalizedColors();
    Color.colorToHSV(colors.toColor(), hsvValues);

    // Determine the detected color based on hue value
    if (hsvValues[0] >= 180 && hsvValues[0] <= 240) {
      detectedColor = DetectedColor.BLUE; // Hue range for blue
    } else if (hsvValues[0] >= 45 && hsvValues[0] <= 90) {
      detectedColor = DetectedColor.YELLOW; // Hue range for yellow
    } else if (hsvValues[0] >= 0 && hsvValues[0] <= 30) {
      detectedColor = DetectedColor.RED; // Hue range for red
    } else {
      detectedColor = DetectedColor.OTHER; // Any other color
    }

    // Display the detected color
    telemetry.addData("Detected Color", detectedColor);
    telemetry.addData("Hue Value", hsvValues[0]);
    telemetry.update();

    relativeLayout.post(() -> relativeLayout.setBackgroundColor(Color.HSVToColor(hsvValues)));

    return detectedColor;

  }
}
